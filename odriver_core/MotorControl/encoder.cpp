#include "odrive_main.h"
#include <Drivers/STM32/stm32_system.h>
#include <bitset>
Encoder::Encoder(TIM_HandleTypeDef* timer, Stm32Gpio index_gpio,
                 Stm32Gpio hallA_gpio, Stm32Gpio hallB_gpio, Stm32Gpio hallC_gpio,
                 Stm32SpiArbiter* spi_arbiter) :
        timer_(timer), index_gpio_(index_gpio),
        hallA_gpio_(hallA_gpio), hallB_gpio_(hallB_gpio), hallC_gpio_(hallC_gpio),
        spi_arbiter_(spi_arbiter)
{
}
static void enc_index_cb_wrapper(void* ctx) {
    reinterpret_cast<Encoder*>(ctx)->enc_index_cb();
}
bool Encoder::apply_config(ODriveIntf::MotorIntf::MotorType motor_type) {
    config_.parent = this;
    update_pll_gains();
    if (config_.pre_calibrated) {
        if (config_.mode == Encoder::MODE_HALL && config_.hall_polarity_calibrated)
            is_ready_ = true;
        if (config_.mode == Encoder::MODE_SINCOS)
            is_ready_ = true;
        if (motor_type == Motor::MOTOR_TYPE_ACIM)
            is_ready_ = true;
    }
    return true;
}
void Encoder::setup() {
    HAL_TIM_Encoder_Start(timer_, TIM_CHANNEL_ALL);
    set_idx_subscribe();
    mode_ = config_.mode;
    spi_task_.config = {
        .Mode = SPI_MODE_MASTER,
        .Direction = SPI_DIRECTION_2LINES,
        .DataSize = SPI_DATASIZE_16BIT,
        .CLKPolarity = (mode_ == MODE_SPI_ABS_AEAT || mode_ == MODE_SPI_ABS_MA732) ? SPI_POLARITY_HIGH : SPI_POLARITY_LOW,
        .CLKPhase = SPI_PHASE_2EDGE,
        .NSS = SPI_NSS_SOFT,
        .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16,
        .FirstBit = SPI_FIRSTBIT_MSB,
        .TIMode = SPI_TIMODE_DISABLE,
        .CRCCalculation = SPI_CRCCALCULATION_DISABLE,
        .CRCPolynomial = 10,
    };
    if (mode_ == MODE_SPI_ABS_MA732) {
        abs_spi_dma_tx_[0] = 0x0000;
    }
    if(mode_ & MODE_FLAG_ABS){
        abs_spi_cs_pin_init();
        if (axis_->controller_.config_.anticogging.pre_calibrated) {
            axis_->controller_.anticogging_valid_ = true;
        }
    }
}
void Encoder::set_error(Error error) {
    vel_estimate_valid_ = false;
    pos_estimate_valid_ = false;
    error_ |= error;
    axis_->error_ |= Axis::ERROR_ENCODER_FAILED;
}
bool Encoder::do_checks(){
    return error_ == ERROR_NONE;
}
//--------------------
// Hardware Dependent
//--------------------
// Triggered when an encoder passes over the "Index" pin
// TODO: only arm index edge interrupt when we know encoder has powered up
// (maybe by attaching the interrupt on start search, synergistic with following)
void Encoder::enc_index_cb() {
    if (config_.use_index) {
        set_circular_count(0, false);
        if (config_.use_index_offset)
            set_linear_count((int32_t)(config_.index_offset * config_.cpr));
        if (config_.pre_calibrated) {
            is_ready_ = true;
            if(axis_->controller_.config_.anticogging.pre_calibrated){
                axis_->controller_.anticogging_valid_ = true;
            }
        } else {
            // We can't use the update_offset facility in set_circular_count because
            // we also set the linear count before there is a chance to update. Therefore:
            // Invalidate offset calibration that may have happened before idx search
            is_ready_ = false;
        }
        index_found_ = true;
    }
    // Disable interrupt
    index_gpio_.unsubscribe();
}
void Encoder::set_idx_subscribe(bool override_enable) {
    if (config_.use_index && (override_enable || !config_.find_idx_on_lockin_only)) {
        if (!index_gpio_.subscribe(true, false, enc_index_cb_wrapper, this)) {
            odrv.misconfigured_ = true;
        }
    } else if (!config_.use_index || config_.find_idx_on_lockin_only) {
        index_gpio_.unsubscribe();
    }
}
void Encoder::update_pll_gains() {
    pll_kp_ = 2.0f * config_.bandwidth;  // basic conversion to discrete time
    pll_ki_ = 0.25f * (pll_kp_ * pll_kp_); // Critically damped
    // Check that we don't get problems with discrete time approximation
    if (!(current_meas_period * pll_kp_ < 1.0f)) {
        set_error(ERROR_UNSTABLE_GAIN);
    }
}
void Encoder::check_pre_calibrated() {
    // TODO: restoring config from python backup is fragile here (ACIM motor type must be set first)
    if (axis_->motor_.config_.motor_type != Motor::MOTOR_TYPE_ACIM) {
        if (!is_ready_)
            config_.pre_calibrated = false;
        if (mode_ == MODE_INCREMENTAL && !index_found_)
            config_.pre_calibrated = false;
    }
}
// Function that sets the current encoder count to a desired 32-bit value.
void Encoder::set_linear_count(int32_t count) {
    // Disable interrupts to make a critical section to avoid race condition
    uint32_t prim = cpu_enter_critical();
    // Update states
    shadow_count_ = count;
    pos_estimate_counts_ = (float)count;
    tim_cnt_sample_ = count;
    //Write hardware last
    timer_->Instance->CNT = count;
    cpu_exit_critical(prim);
}
// Function that sets the CPR circular tracking encoder count to a desired 32-bit value.
// Note that this will get mod'ed down to [0, cpr)
void Encoder::set_circular_count(int32_t count, bool update_offset) {
    // Disable interrupts to make a critical section to avoid race condition
    uint32_t prim = cpu_enter_critical();
    if (update_offset) {
        config_.phase_offset += count - count_in_cpr_;
        config_.phase_offset = mod(config_.phase_offset, config_.cpr);
    }
    // Update states
    count_in_cpr_ = mod(count, config_.cpr);
    pos_cpr_counts_ = (float)count_in_cpr_;
    cpu_exit_critical(prim);
}
bool Encoder::run_index_search() {
    config_.use_index = true;
    index_found_ = false;
    set_idx_subscribe();
    bool success = axis_->run_lockin_spin(axis_->config_.calibration_lockin, false);
    return success;
}
bool Encoder::run_direction_find() {
    int32_t init_enc_val = shadow_count_;
    Axis::LockinConfig_t lockin_config = axis_->config_.calibration_lockin;
    lockin_config.finish_distance = lockin_config.vel * 3.0f; // run for 3 seconds
    lockin_config.finish_on_distance = true;
    lockin_config.finish_on_enc_idx = false;
    lockin_config.finish_on_vel = false;
    bool success = axis_->run_lockin_spin(lockin_config, false);
    if (success) {
        // Check response and direction
        if (shadow_count_ > init_enc_val + 8) {
            // motor same dir as encoder
            config_.direction = 1;
        } else if (shadow_count_ < init_enc_val - 8) {
            // motor opposite dir as encoder
            config_.direction = -1;
        } else {
            config_.direction = 0;
        }
    }
    return success;
}
bool Encoder::run_hall_polarity_calibration() {
    Axis::LockinConfig_t lockin_config = axis_->config_.calibration_lockin;
    lockin_config.finish_distance = lockin_config.vel * 3.0f; // run for 3 seconds
    lockin_config.finish_on_distance = true;
    lockin_config.finish_on_enc_idx = false;
    lockin_config.finish_on_vel = false;
    auto loop_cb = [this](bool const_vel) {
        if (const_vel)
            sample_hall_states_ = true;
        // No need to cancel early
        return true;
    };
    config_.hall_polarity_calibrated = false;
    states_seen_count_.fill(0);
    bool success = axis_->run_lockin_spin(lockin_config, false, loop_cb);
    sample_hall_states_ = false;
    if (success) {
        std::bitset<8> state_seen;
        std::bitset<8> state_confirmed;
        for (int i = 0; i < 8; i++) {
            if (states_seen_count_[i] > 0)
                state_seen[i] = true;
            if (states_seen_count_[i] > 50)
                state_confirmed[i] = true;
        }
        if (!(state_seen == state_confirmed)) {
            set_error(ERROR_ILLEGAL_HALL_STATE);
            return false;
        }
        // Hall effect sensors can be arranged at 60 or 120 electrical degrees.
        // Out of 8 possible states, 120 and 60 deg arrangements each miss 2 states.
        // ODrive assumes 120 deg separation - if a 60 deg setup is used, it can
        // be converted to 120 deg states by flipping the polarity of one sensor.
        uint8_t states = state_seen.to_ulong();
        uint8_t hall_polarity = 0;
        auto flip_detect = [](uint8_t states, unsigned int idx)->bool {
            return (~states & 0xFF) == (1<<(0+idx) | 1<<(7-idx));
        };
        if (flip_detect(states, 0)) {
            hall_polarity = 0b000;
        } else if (flip_detect(states, 1)) {
            hall_polarity = 0b001;
        } else if (flip_detect(states, 2)) {
            hall_polarity = 0b010;
        } else if (flip_detect(states, 3)) {
            hall_polarity = 0b100;
        } else {
            set_error(ERROR_ILLEGAL_HALL_STATE);
            return false;
        }
        config_.hall_polarity = hall_polarity;
        config_.hall_polarity_calibrated = true;
    }
    return success;
}
bool Encoder::run_hall_phase_calibration() {
    Axis::LockinConfig_t lockin_config = axis_->config_.calibration_lockin;
    lockin_config.finish_distance = lockin_config.vel * 30.0f; // run for 30 seconds
    lockin_config.finish_on_distance = true;
    lockin_config.finish_on_enc_idx = false;
    lockin_config.finish_on_vel = false;
    auto loop_cb = [this](bool const_vel) {
        if (const_vel)
            sample_hall_phase_ = true;
        // No need to cancel early
        return true;
    };
    calibrate_hall_phase_ = true;
    config_.hall_edge_phcnt.fill(0.0f);
    hall_phase_calib_seen_count_.fill(0);
    bool success = axis_->run_lockin_spin(lockin_config, false, loop_cb);
    if (error_ & ERROR_ILLEGAL_HALL_STATE)
        success = false;
    if (success) {
        float delta_phase = 0.0f;
        for (int i = 0; i < 6; i++) {
            int next_i = (i == 5) ? 0 : i+1;
            delta_phase += wrap_pm_pi(config_.hall_edge_phcnt[next_i] - config_.hall_edge_phcnt[i]);
        }
        if (delta_phase < 0.0f) {
            config_.direction = -1;
            for (int i = 0; i < 6; i++)
                config_.hall_edge_phcnt[i] = wrap_pm_pi(-config_.hall_edge_phcnt[i]);
        } else {
            config_.direction = 1;
        }
        float offset = config_.hall_edge_phcnt[0];
        for (int i = 0; i < 6; i++) {
            float& phcnt = config_.hall_edge_phcnt[i];
            phcnt = fmodf_pos((6.0f / (2.0f * M_PI)) * (phcnt - offset), 6.0f);
        }
    } else {
        config_.hall_edge_phcnt = hall_edge_defaults;
    }
    calibrate_hall_phase_ = false;
    return success;
}
bool Encoder::run_offset_calibration() {
    const float start_lock_duration = 1.0f;
    if (config_.use_index && !index_found_) {
        set_error(ERROR_INDEX_NOT_FOUND_YET);
        return false;
    }
    if (config_.mode == MODE_HALL && !config_.hall_polarity_calibrated) {
        set_error(ERROR_HALL_NOT_CALIBRATED_YET);
        return false;
    }
    shadow_count_ = count_in_cpr_;
    CRITICAL_SECTION() {
        axis_->open_loop_controller_.Idq_setpoint_ = {0.0f, 0.0f};
        axis_->open_loop_controller_.Vdq_setpoint_ = {0.0f, 0.0f};
        axis_->open_loop_controller_.phase_ = 0.0f;
        axis_->open_loop_controller_.phase_vel_ = 0.0f;
        float max_current_ramp = axis_->motor_.config_.calibration_current / start_lock_duration * 2.0f;
        axis_->open_loop_controller_.max_current_ramp_ = max_current_ramp;
        axis_->open_loop_controller_.max_voltage_ramp_ = max_current_ramp;
        axis_->open_loop_controller_.max_phase_vel_ramp_ = INFINITY;
        axis_->open_loop_controller_.target_current_ = axis_->motor_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL ? axis_->motor_.config_.calibration_current : 0.0f;
        axis_->open_loop_controller_.target_voltage_ = axis_->motor_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL ? 0.0f : axis_->motor_.config_.calibration_current;
        axis_->open_loop_controller_.target_vel_ = 0.0f;
        axis_->open_loop_controller_.total_distance_ = 0.0f;
        axis_->open_loop_controller_.phase_ = axis_->open_loop_controller_.initial_phase_ = wrap_pm_pi(0 - config_.calib_scan_distance / 2.0f);
        axis_->motor_.current_control_.enable_current_control_src_ = (axis_->motor_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL);
        axis_->motor_.current_control_.Idq_setpoint_src_.connect_to(&axis_->open_loop_controller_.Idq_setpoint_);
        axis_->motor_.current_control_.Vdq_setpoint_src_.connect_to(&axis_->open_loop_controller_.Vdq_setpoint_);
        axis_->motor_.current_control_.phase_src_.connect_to(&axis_->open_loop_controller_.phase_);
        axis_->acim_estimator_.rotor_phase_src_.connect_to(&axis_->open_loop_controller_.phase_);
        axis_->motor_.phase_vel_src_.connect_to(&axis_->open_loop_controller_.phase_vel_);
        axis_->motor_.current_control_.phase_vel_src_.connect_to(&axis_->open_loop_controller_.phase_vel_);
        axis_->acim_estimator_.rotor_phase_vel_src_.connect_to(&axis_->open_loop_controller_.phase_vel_);
    }
    axis_->wait_for_control_iteration();
    axis_->motor_.arm(&axis_->motor_.current_control_);
    for (size_t i = 0; i < (size_t)(start_lock_duration * 1000.0f); ++i) {
        if (!axis_->motor_.is_armed_) {
            return false;
        }
        if (axis_->requested_state_ != Axis::AXIS_STATE_UNDEFINED) {
            axis_->motor_.disarm();
            return false;
        }
        osDelay(1);
    }
    int32_t init_enc_val = shadow_count_;
    uint32_t num_steps = 0;
    int64_t encvaluesum = 0;
    CRITICAL_SECTION() {
        axis_->open_loop_controller_.target_vel_ = config_.calib_scan_omega;
        axis_->open_loop_controller_.total_distance_ = 0.0f;
    }
    while ((axis_->requested_state_ == Axis::AXIS_STATE_UNDEFINED) && axis_->motor_.is_armed_) {
        bool reached_target_dist = axis_->open_loop_controller_.total_distance_.any().value_or(-INFINITY) >= config_.calib_scan_distance;
        if (reached_target_dist) {
            break;
        }
        encvaluesum += shadow_count_;
        num_steps++;
        osDelay(1);
    }
    if (shadow_count_ > init_enc_val + 8) {
        config_.direction = 1;
    } else if (shadow_count_ < init_enc_val - 8) {
        config_.direction = -1;
    } else {
        set_error(ERROR_NO_RESPONSE);
        axis_->motor_.disarm();
        return false;
    }
    float elec_rad_per_enc = axis_->motor_.config_.pole_pairs * 2 * M_PI * (1.0f / (float)(config_.cpr));
    float expected_encoder_delta = config_.calib_scan_distance / elec_rad_per_enc;
    calib_scan_response_ = std::abs(shadow_count_ - init_enc_val);
    if (std::abs(calib_scan_response_ - expected_encoder_delta) / expected_encoder_delta > config_.calib_range) {
        set_error(ERROR_CPR_POLEPAIRS_MISMATCH);
        axis_->motor_.disarm();
        return false;
    }
    float mean_enc_delta = (float)(encvaluesum - (int64_t)init_enc_val * (int64_t)num_steps) / (float)num_steps;
    float calib_enc_angle = mean_enc_delta * elec_rad_per_enc;
    int32_t expected_phase = config_.calib_scan_distance;
    float phase_offset = expected_phase - calib_enc_angle;
    config_.phase_offset = static_cast<int32_t>(std::round(phase_offset / elec_rad_per_enc));
    is_ready_ = true;
    axis_->motor_.disarm();
    return true;
}
bool Encoder::update() {
    if (mode_ == MODE_INCREMENTAL || mode_ == MODE_HALL) {
        int16_t tim_cnt = (int16_t)(int32_t)timer_->Instance->CNT;
        int32_t delta_cnt = tim_cnt - tim_cnt_sample_;
        tim_cnt_sample_ = tim_cnt;
        int32_t corrected_cnt;
        if (config_.direction == 1) {
            corrected_cnt = delta_cnt;
        } else if (config_.direction == -1) {
            corrected_cnt = -delta_cnt;
        } else {
            return true;
        }
        shadow_count_ += corrected_cnt;
        pos_estimate_counts_ += corrected_cnt;
        count_in_cpr_ = (int32_t)(pos_cpr_counts_ + corrected_cnt);
        count_in_cpr_ = mod(count_in_cpr_, config_.cpr);
        pos_cpr_counts_ = (float)count_in_cpr_;
    }
    float delta_pos = pos_estimate_counts_ - pos_estimate_counts_prev_;
    pos_estimate_counts_prev_ = pos_estimate_counts_;
    if (config_.enable_phase_interpolation) {
        interpolation_ += delta_pos;
        if (std::abs(interpolation_) > 0.5f * 4.0f) {
            interpolation_ = 0.0f;
        }
    } else {
        interpolation_ = 0.0f;
    }
    float pll_pos = pos_cpr_counts_;
    if (config_.enable_phase_interpolation) {
        pll_pos += interpolation_ * 0.25f;
    }
    delta_pos_cpr_counts_ = pll_pos - pll_pos_;
    pll_pos_ += current_meas_period * pll_vel_;
    pll_vel_ += current_meas_period * pll_ki_ * delta_pos_cpr_counts_;
    pll_pos_ += current_meas_period * pll_kp_ * delta_pos_cpr_counts_;
    float phase = (pll_pos_ * 2.0f * M_PI / (float)config_.cpr + config_.phase_offset_float);
    phase = wrap_pm_pi(phase);
    float phase_vel = pll_vel_ * 2.0f * M_PI / (float)config_.cpr;
    phase_ = phase;
    phase_vel_ = phase_vel;
    pos_estimate_ = (pos_estimate_counts_ / (float)config_.cpr) * config_.direction;
    vel_estimate_ = phase_vel / (2.0f * M_PI);
    pos_circular_ = pos_cpr_counts_ / (float)config_.cpr;
    pos_estimate_valid_ = true;
    vel_estimate_valid_ = true;
    return true;
}
void Encoder::sample_now() {
    if (mode_ & MODE_FLAG_ABS) {
        if (!abs_spi_pos_updated_) {
            abs_spi_start_transaction();
        }
    }
    port_samples_[0] = GPIOA->IDR;
    port_samples_[1] = GPIOB->IDR;
    port_samples_[2] = GPIOC->IDR;
}
bool Encoder::read_sampled_gpio(Stm32Gpio gpio) {
    for (size_t i = 0; i < sizeof(ports_to_sample) / sizeof(ports_to_sample[0]); i++) {
        if (gpio.port_ == ports_to_sample[i]) {
            return (port_samples_[i] & gpio.pin_mask_) != 0;
        }
    }
    return false;
}
void Encoder::decode_hall_samples() {
    hall_state_ = 0;
    hall_state_ |= read_sampled_gpio(hallA_gpio_) ? 0x1 : 0x0;
    hall_state_ |= read_sampled_gpio(hallB_gpio_) ? 0x2 : 0x0;
    hall_state_ |= read_sampled_gpio(hallC_gpio_) ? 0x4 : 0x0;
}
int32_t Encoder::hall_model(float internal_pos) {
    return 0;
}
void Encoder::abs_spi_cs_pin_init() {
    abs_spi_cs_gpio_ = get_gpio(config_.abs_spi_cs_gpio_pin);
}
bool Encoder::abs_spi_start_transaction() {
    return false;
}
void Encoder::abs_spi_cb(bool success) {
}