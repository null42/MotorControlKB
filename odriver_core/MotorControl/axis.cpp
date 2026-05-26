#include <stdlib.h>
#include <functional>
#include "gpio.h"
#include "odrive_main.h"
#include "utils.hpp"
#include "communication/interface_can.hpp"
Axis::Axis(int axis_num,
           uint16_t default_step_gpio_pin,
           uint16_t default_dir_gpio_pin,
           osPriority thread_priority,
           Encoder& encoder,
           SensorlessEstimator& sensorless_estimator,
           Controller& controller,
           Motor& motor,
           TrapezoidalTrajectory& trap,
           Endstop& min_endstop,
           Endstop& max_endstop,
           MechanicalBrake& mechanical_brake)
    : axis_num_(axis_num),
      default_step_gpio_pin_(default_step_gpio_pin),
      default_dir_gpio_pin_(default_dir_gpio_pin),
      thread_priority_(thread_priority),
      encoder_(encoder),
      sensorless_estimator_(sensorless_estimator),
      controller_(controller),
      motor_(motor),
      trap_traj_(trap),
      min_endstop_(min_endstop),
      max_endstop_(max_endstop),
      mechanical_brake_(mechanical_brake)
{
    encoder_.axis_ = this;
    sensorless_estimator_.axis_ = this;
    controller_.axis_ = this;
    motor_.axis_ = this;
    trap_traj_.axis_ = this;
    min_endstop_.axis_ = this;
    max_endstop_.axis_ = this;
    mechanical_brake_.axis_ = this;
}
Axis::LockinConfig_t Axis::default_calibration() {
    Axis::LockinConfig_t config;
    config.current = 10.0f;           // [A]
    config.ramp_time = 0.4f;          // [s]
    config.ramp_distance = 1 * M_PI;  // [rad]
    config.accel = 20.0f;     // [rad/s^2]
    config.vel = 40.0f; // [rad/s]
    config.finish_distance = 100.0f * 2.0f * M_PI;  // [rad]
    config.finish_on_vel = false;
    config.finish_on_distance = true;
    config.finish_on_enc_idx = true;
    return config;
}
Axis::LockinConfig_t Axis::default_sensorless() {
    Axis::LockinConfig_t config;
    config.current = 10.0f;           // [A]
    config.ramp_time = 0.4f;          // [s]
    config.ramp_distance = 1 * M_PI;  // [rad]
    config.accel = 200.0f;     // [rad/s^2]
    config.vel = 400.0f; // [rad/s]
    config.finish_distance = 100.0f;  // [rad]
    config.finish_on_vel = true;
    config.finish_on_distance = false;
    config.finish_on_enc_idx = false;
    return config;
}
static void step_cb_wrapper(void* ctx) {
    reinterpret_cast<Axis*>(ctx)->step_cb();
}
bool Axis::apply_config() {
    config_.parent = this;
    decode_step_dir_pins();
    watchdog_feed();
    return true;
}
void Axis::clear_config() {
    config_ = {};
    config_.step_gpio_pin = default_step_gpio_pin_;
    config_.dir_gpio_pin = default_dir_gpio_pin_;
    config_.can.node_id = axis_num_;
}
static void run_state_machine_loop_wrapper(void* ctx) {
    reinterpret_cast<Axis*>(ctx)->run_state_machine_loop();
    reinterpret_cast<Axis*>(ctx)->thread_id_valid_ = false;
}
// @brief Starts run_state_machine_loop in a new thread
void Axis::start_thread() {
    osThreadDef(thread_def, run_state_machine_loop_wrapper, thread_priority_, 0, stack_size_ / sizeof(StackType_t));
    thread_id_ = osThreadCreate(osThread(thread_def), this);
    thread_id_valid_ = true;
}
/**
 * @brief Blocks until at least one complete control loop has been executed.
 */
bool Axis::wait_for_control_iteration() {
    osSignalWait(0x0001, osWaitForever); // this might return instantly
    osSignalWait(0x0001, osWaitForever); // this might be triggered at the
                                         // end of a control loop iteration
                                         // which was started before we entered
                                         // this function
    osSignalWait(0x0001, osWaitForever);
    return true;
}
// step/direction interface
void Axis::step_cb() {
    if (step_dir_active_) {
        dir_gpio_.read() ? ++steps_ : --steps_;
        controller_.input_pos_updated();
    }
}
void Axis::decode_step_dir_pins() {
    step_gpio_ = get_gpio(config_.step_gpio_pin);
    dir_gpio_ = get_gpio(config_.dir_gpio_pin);
}
// @brief (de)activates step/dir input
void Axis::set_step_dir_active(bool active) {
    if (active) {
        // Subscribe to rising edges of the step GPIO
        if (!step_gpio_.subscribe(true, false, step_cb_wrapper, this)) {
            odrv.misconfigured_ = true;
        }
        step_dir_active_ = true;
    } else {
        step_dir_active_ = false;
        // Unsubscribe from step GPIO
        // TODO: if we change the GPIO while the subscription is active and then
        // unsubscribe then the unsubscribe is for the wrong pin.
        step_gpio_.unsubscribe();
    }
}
// @brief Do axis level checks and call subcomponent do_checks
// Returns true if everything is ok.
bool Axis::do_checks(uint32_t timestamp) {
    // Sub-components should use set_error which will propegate to this error_
    motor_.effective_current_lim();
    motor_.do_checks(timestamp);
    // Check for endstop presses
    if (min_endstop_.config_.enabled && min_endstop_.rose() && !(current_state_ == AXIS_STATE_HOMING)) {
        error_ |= ERROR_MIN_ENDSTOP_PRESSED;
    } else if (max_endstop_.config_.enabled && max_endstop_.rose() && !(current_state_ == AXIS_STATE_HOMING)) {
        error_ |= ERROR_MAX_ENDSTOP_PRESSED;
    }
    return check_for_errors();
}
// @brief Feed the watchdog to prevent watchdog timeouts.
void Axis::watchdog_feed() {
    watchdog_current_value_ = get_watchdog_reset();
}
// @brief Check the watchdog timer for expiration. Also sets the watchdog error bit if expired.
bool Axis::watchdog_check() {
    if (!config_.enable_watchdog) return true;
    // explicit check here to ensure that we don't underflow back to UINT32_MAX
    if (watchdog_current_value_ > 0) {
        watchdog_current_value_--;
        return true;
    } else {
        error_ |= ERROR_WATCHDOG_TIMER_EXPIRED;
        return false;
    }
}
bool Axis::run_lockin_spin(const LockinConfig_t &lockin_config, bool remain_armed,
        std::function<bool(bool)> loop_cb) {
    CRITICAL_SECTION() {
        // Reset state variables
        open_loop_controller_.Idq_setpoint_ = {0.0f, 0.0f};
        open_loop_controller_.Vdq_setpoint_ = {0.0f, 0.0f};
        open_loop_controller_.phase_ = 0.0f;
        open_loop_controller_.phase_vel_ = 0.0f;
        open_loop_controller_.max_current_ramp_ = lockin_config.current / lockin_config.ramp_time;
        open_loop_controller_.max_voltage_ramp_ = lockin_config.current / lockin_config.ramp_time;
        open_loop_controller_.max_phase_vel_ramp_ = lockin_config.accel;
        open_loop_controller_.target_current_ = motor_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL ? lockin_config.current : 0.0f;
        open_loop_controller_.target_voltage_ = motor_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL ? 0.0f : lockin_config.current;
        open_loop_controller_.target_vel_ = lockin_config.vel;
        open_loop_controller_.total_distance_ = 0.0f;
        motor_.current_control_.enable_current_control_src_ = motor_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL;
        motor_.current_control_.Idq_setpoint_src_.connect_to(&open_loop_controller_.Idq_setpoint_);
        motor_.current_control_.Vdq_setpoint_src_.connect_to(&open_loop_controller_.Vdq_setpoint_);
        motor_.current_control_.phase_src_.connect_to(&open_loop_controller_.phase_);
        acim_estimator_.rotor_phase_src_.connect_to(&open_loop_controller_.phase_);
        
        motor_.phase_vel_src_.connect_to(&open_loop_controller_.phase_vel_);
        motor_.current_control_.phase_vel_src_.connect_to(&open_loop_controller_.phase_vel_);
        acim_estimator_.rotor_phase_vel_src_.connect_to(&open_loop_controller_.phase_vel_);
    }
    wait_for_control_iteration();
    motor_.arm(&motor_.current_control_);
    bool subscribed_to_idx_once = false;
    bool success = false;
    float dir = lockin_config.vel >= 0.0f ? 1.0f : -1.0f;
    while ((requested_state_ == AXIS_STATE_UNDEFINED) && motor_.is_armed_) {
        bool reached_target_vel = std::abs(open_loop_controller_.phase_vel_.any().value_or(0.0f) - lockin_config.vel) <= std::numeric_limits<float>::epsilon();
        bool reached_target_dist = open_loop_controller_.total_distance_.any().value_or(0.0f) * dir >= lockin_config.finish_distance * dir;
        // Check if terminal condition is reached
        bool terminal_condition = (reached_target_vel && lockin_config.finish_on_vel)
                               || (reached_target_dist && lockin_config.finish_on_distance)
                               || (encoder_.index_found_ && lockin_config.finish_on_enc_idx);
        if (terminal_condition) {
            success = true;
            break;
        }
        // Activate index pin as soon as target velocity was reached. This is
        // to avoid hitting the index from the wrong direction.
        if (reached_target_vel && !encoder_.index_found_ && !subscribed_to_idx_once) {
            encoder_.set_idx_subscribe(true);
            subscribed_to_idx_once = true;
        }
        if (loop_cb)
            if (!loop_cb(reached_target_vel))
                break;
        // TODO: use new sync function instead
        asm volatile ("" ::: "memory");
        osDelay(1);
    }
    if (!success || !remain_armed) {
        motor_.disarm();
    }
    return success;
}
bool Axis::start_closed_loop_control() {
    bool sensorless_mode = config_.enable_sensorless_mode;
    if (sensorless_mode) {
        // TODO: restart if desired
        if (!run_lockin_spin(config_.sensorless_ramp, true)) {
            return false;
        }
    }
    // Hook up the data paths between the components
    CRITICAL_SECTION() {
        if (sensorless_mode) {
            controller_.pos_estimate_linear_src_.disconnect();
            controller_.pos_estimate_circular_src_.disconnect();
            controller_.pos_wrap_src_.disconnect();
            controller_.vel_estimate_src_.connect_to(&sensorless_estimator_.vel_estimate_);
        } else if (controller_.config_.load_encoder_axis < AXIS_COUNT) {
            Axis* ax = &axes[controller_.config_.load_encoder_axis];
            controller_.pos_estimate_circular_src_.connect_to(&ax->encoder_.pos_circular_);
            controller_.pos_wrap_src_.connect_to(&controller_.config_.circular_setpoint_range);
            controller_.pos_estimate_linear_src_.connect_to(&ax->encoder_.pos_estimate_);
            controller_.vel_estimate_src_.connect_to(&ax->encoder_.vel_estimate_);
        } else {
            controller_.pos_estimate_circular_src_.disconnect();
            controller_.pos_estimate_linear_src_.disconnect();
            controller_.pos_wrap_src_.disconnect();
            controller_.vel_estimate_src_.disconnect();
            controller_.set_error(Controller::ERROR_INVALID_LOAD_ENCODER);
            return false;
        }
        // To avoid any transient on startup, we intialize the setpoint to be the current position
        controller_.control_mode_updated();
        controller_.input_pos_updated();
        // Avoid integrator windup issues
        controller_.vel_integrator_torque_ = 0.0f;
        motor_.torque_setpoint_src_.connect_to(&controller_.torque_output_);
        motor_.direction_ = sensorless_mode ? 1.0f : encoder_.config_.direction;
        motor_.current_control_.enable_current_control_src_ = motor_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL;
        motor_.current_control_.Idq_setpoint_src_.connect_to(&motor_.Idq_setpoint_);
        motor_.current_control_.Vdq_setpoint_src_.connect_to(&motor_.Vdq_setpoint_);
        bool is_acim = motor_.config_.motor_type == Motor::MOTOR_TYPE_ACIM;
        // phase
        OutputPort<float>* phase_src = sensorless_mode ? &sensorless_estimator_.phase_ : &encoder_.phase_;
        acim_estimator_.rotor_phase_src_.connect_to(phase_src);
        OutputPort<float>* stator_phase_src = is_acim ? &acim_estimator_.stator_phase_ : phase_src;
        motor_.current_control_.phase_src_.connect_to(stator_phase_src);
        // phase vel
        OutputPort<float>* phase_vel_src = sensorless_mode ? &sensorless_estimator_.phase_vel_ : &encoder_.phase_vel_;
        acim_estimator_.rotor_phase_vel_src_.connect_to(phase_vel_src);
        OutputPort<float>* stator_phase_vel_src = is_acim ? &acim_estimator_.stator_phase_vel_ : phase_vel_src;
        motor_.phase_vel_src_.connect_to(stator_phase_vel_src);
        motor_.current_control_.phase_vel_src_.connect_to(stator_phase_vel_src);
        
        if (sensorless_mode) {
            // Make the final velocity of the loĉk-in spin the setpoint of the
            // closed loop controller to allow for smooth transition.
            float vel = config_.sensorless_ramp.vel / (2.0f * M_PI * motor_.config_.pole_pairs);
            controller_.input_vel_ = vel;
            controller_.vel_setpoint_ = vel;
        }
    }
    // In sensorless mode the motor is already armed.
    if (!motor_.is_armed_) {
        wait_for_control_iteration();
        motor_.arm(&motor_.current_control_);
    }
    return true;
}
bool Axis::stop_closed_loop_control() {
    motor_.disarm();
    return check_for_errors();
}
bool Axis::run_closed_loop_control_loop() {
    start_closed_loop_control();
    set_step_dir_active(config_.enable_step_dir);
    while ((requested_state_ == AXIS_STATE_UNDEFINED) && motor_.is_armed_) {
        osDelay(1);
    }
    set_step_dir_active(config_.enable_step_dir && config_.step_dir_always_on);
    stop_closed_loop_control();
    return check_for_errors();
}
// Slowly drive in the negative direction at homing_speed until the min endstop is pressed
// When pressed, set the linear count to the offset (default 0), and then go to position 0
bool Axis::run_homing() {
    // TODO: theoretically this check should be inside the update loop,
    // otherwise someone could disable the endstop while homing is in progress.
    if (!min_endstop_.config_.enabled) {
        return error_ |= ERROR_HOMING_WITHOUT_ENDSTOP, false;
    }
    controller_.config_.control_mode = Controller::CONTROL_MODE_VELOCITY_CONTROL;
    controller_.config_.input_mode = Controller::INPUT_MODE_VEL_RAMP;
    controller_.input_pos_ = 0.0f;
    controller_.input_pos_updated();
    controller_.input_vel_ = -controller_.config_.homing_speed;
    controller_.input_torque_ = 0.0f;
    homing_.is_homed = false;
    error_ &= ~ERROR_MIN_ENDSTOP_PRESSED;
    bool done = false;
    start_closed_loop_control();
    // Driving toward the endstop
    while ((requested_state_ == AXIS_STATE_UNDEFINED) && motor_.is_armed_ && !(done = min_endstop_.get_state())) {
        osDelay(1);
    }
    stop_closed_loop_control();
    
    controller_.input_vel_ = 0.0f;
    if (!done) {
        return false;
    }
    error_ &= ~ERROR_MIN_ENDSTOP_PRESSED; // clear this error since we deliberately drove into the endstop
    std::optional<float> pos_estimate_local = encoder_.pos_estimate_.any();
    if (pos_estimate_local == std::nullopt || !pos_estimate_local.has_value()){
        return error_ |= ERROR_UNKNOWN_POSITION, false;
    }
    
    controller_.config_.control_mode = Controller::CONTROL_MODE_POSITION_CONTROL;
    controller_.config_.input_mode = Controller::INPUT_MODE_TRAP_TRAJ;
    // Initialize closed loop control, and then set the desired location.
    start_closed_loop_control();
    
    controller_.input_pos_ = pos_estimate_local.value() + min_endstop_.config_.offset;
    controller_.pos_setpoint_ = pos_estimate_local.value();
    controller_.vel_setpoint_ = 0.0f;
    controller_.input_pos_updated();
    // Synchronization issue.  Ensure trajectory_done is false prior to the while loop, so that
    // the controller has time to run move_to_pos() on the next update()
    controller_.trajectory_done_ = false; 
    
    while ((requested_state_ == AXIS_STATE_UNDEFINED) && motor_.is_armed_ && !(done = controller_.trajectory_done_)) {
        osDelay(1);
    }
    stop_closed_loop_control();
    if (!done) {
        return false;
    }
    // Set the current position to 0, the target to zero,
    controller_.pos_setpoint_ = min_endstop_.config_.offset;
    encoder_.pos_abs_ = controller_.pos_setpoint_;
    encoder_.pos_abs_latched_ = true;
    encoder_.pos_estimate_ = controller_.pos_setpoint_ + encoder_.config_.offset;
    homing_.is_homed = true;
    return true;
}
bool Axis::run_idle_loop() {
    while (requested_state_ == AXIS_STATE_UNDEFINED) {
        osDelay(1);
    }
    return check_for_errors();
}
// @brief Tune the current controller based on phase resistance and inductance
// This uses the approach described in:
// https://e2e.ti.com/blogs_/b/motordrivecontrol/archive/2015/07/20/teaching-your-pi-controller-to-behave-part-ii
// 
// The summary is that the controller gains can be approximated as follows:
// Kp = L * bandwidth
// Ki = R * bandwidth
// 
// Where L is the inductance, R is the resistance, and bandwidth is the 
// desired cutoff frequency.
bool Axis::run_current_controller_tuning() {
    // Note: we don't need to calculate Kp and Ki directly
    // because the current controller uses normalized gains:
    // Kp_norm = Kp * current / voltage
    // Ki_norm = Ki * current / voltage
    return true;
}