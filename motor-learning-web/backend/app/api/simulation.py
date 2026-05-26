import os
import re
import shutil
import subprocess
import tempfile
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

router = APIRouter(tags=["simulation"])

EMACHENY_SRC = r"E:\gitdownload\ACMSIMC_TUT-commissioning_spmsm\ACMSIMC_TUT-commissioning_spmsm"


class SimParams(BaseModel):
    R: float = 0.45
    Ld: float = 0.00415
    Lq: float = 0.00415
    KE: float = 0.504
    Js: float = 0.06
    npp: int = 2
    Vdc: float = 300.0
    cur_Kp: float = 15.0
    cur_Ti: float = 0.08
    cur_limit: float = 400.0
    spd_Kp: float = 0.5
    spd_Ti: float = 1.05
    spd_limit: float = 8.0
    speed_ref: float = 500.0
    steps: int = 50000
    Ts: float = 0.00025
    down_sample: int = 10


def generate_acmconfig(params: SimParams) -> str:
    ts_upsampling = 2
    ts_inverse = int(1.0 / params.Ts)
    return f'''#ifndef ACMCONFIG_H
#define ACMCONFIG_H

#define NUMBER_OF_STEPS    {params.steps}
#define TS                 {params.Ts}
#define TS_INVERSE         {ts_inverse}

#define TS_UPSAMPLING_FREQ_EXE         0.5
#define TS_UPSAMPLING_FREQ_EXE_INVERSE 2

#define MACHINE_TS         (TS*TS_UPSAMPLING_FREQ_EXE)
#define MACHINE_TS_INVERSE (TS_INVERSE*TS_UPSAMPLING_FREQ_EXE_INVERSE)
#define DOWN_SAMPLE        {params.down_sample}

#define PMSM_RESISTANCE                   {params.R}
#define PMSM_D_AXIS_INDUCTANCE            {params.Ld}
#define PMSM_Q_AXIS_INDUCTANCE            {params.Lq}
#define PMSM_PERMANENT_MAGNET_FLUX_LINKAGE {params.KE}
#define PMSM_SHAFT_INERTIA                {params.Js}
#define PMSM_NUMBER_OF_POLE_PAIRS         {params.npp}

#define CONTROL_STRATEGY             NULL_D_AXIS_CURRENT_CONTROL
#define SENSORLESS_CONTROL           false

#define CURRENT_LOOP_PID_PROPORTIONAL_GAIN     {params.cur_Kp}
#define CURRENT_LOOP_PID_INTEGRAL_TIME_CONSTANT {params.cur_Ti}
#define CURRENT_LOOP_PID_DIREVATIVE_TIME_CONSTANT 0
#define CURRENT_LOOP_LIMIT_VOLTS               {params.cur_limit}

#define SPEED_LOOP_PID_PROPORTIONAL_GAIN       {params.spd_Kp}
#define SPEED_LOOP_PID_INTEGRAL_TIME_CONSTANT  {params.spd_Ti}
#define SPEED_LOOP_PID_DIREVATIVE_TIME_CONSTANT 0
#define SPEED_LOOP_LIMIT_NEWTON_METER          {params.spd_limit}
#define SPEED_LOOP_CEILING                     40

#define SATURATED_MAGNETIC_CIRCUIT     False
#define VOLTAGE_CURRENT_DECOUPLING_CIRCUIT False
#define INVERTER_NONLINEARITY          true
#define PC_SIMULATION                  True

#define DATA_FILE_NAME   "pmsm_sc.dat"

#endif
'''


@router.post("/simulate")
async def run_simulation(params: SimParams):
    workdir = tempfile.mkdtemp(prefix="acmsimc_")

    try:
        src_files = [
            "main.c", "controller.c", "commissioning.c",
            "observer.c", "inverter.c",
            "inverter.h", "controller.h", "commissioning.h",
            "observer.h", "ACMSim.h", "ACMConfig.h",
        ]
        for f in src_files:
            src_path = os.path.join(EMACHENY_SRC, f)
            if os.path.exists(src_path):
                shutil.copy2(src_path, os.path.join(workdir, f))

        config_path = os.path.join(workdir, "ACMConfig.h")
        with open(config_path, "w", encoding="utf-8") as f:
            f.write(generate_acmconfig(params))

        obs_h_path = os.path.join(workdir, "observer.h")
        if os.path.exists(obs_h_path):
            with open(obs_h_path, "r", encoding="utf-8", errors="replace") as f:
                obs_content = f.read()
            obs_content = obs_content.replace(
                '#define IS_P(X) sm.is_prev[X]',
                '#define IS_P(X) sm.is_prev[X]\n#define US_P(X) sm.us_prev[X]',
            )
            with open(obs_h_path, "w", encoding="utf-8") as f:
                f.write(obs_content)

        inv_h_path = os.path.join(workdir, "inverter.h")
        if os.path.exists(inv_h_path):
            with open(inv_h_path, "r", encoding="utf-8", errors="replace") as f:
                inv_content = f.read()
            inv_content = re.sub(
                r'#define\s+_Udc\s+\d+',
                f'#define _Udc   {int(params.Vdc)}',
                inv_content,
            )
            with open(inv_h_path, "w", encoding="utf-8") as f:
                f.write(inv_content)

        compile_result = subprocess.run(
            [
                "gcc", "-O2", "-o", "acmsimc.exe",
                "main.c", "controller.c", "commissioning.c",
                "observer.c", "inverter.c", "-lm",
            ],
            cwd=workdir,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=30,
        )
        if compile_result.returncode != 0:
            raise HTTPException(
                status_code=500,
                detail=f"Compilation failed: {compile_result.stderr}",
            )

        subprocess.run(
            [os.path.join(workdir, "acmsimc.exe")],
            cwd=workdir,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=60,
        )

        csv_path = os.path.join(workdir, "pmsm_sc.dat")
        if not os.path.exists(csv_path):
            raise HTTPException(
                status_code=500,
                detail="Simulation output file not found",
            )

        time_data = []
        id_data = []
        iq_data = []
        rpm_data = []
        ud_data = []
        uq_data = []
        KE_data = []
        Js_data = []

        with open(csv_path, "r", encoding="utf-8", errors="replace") as f:
            f.readline()
            machine_ts = params.Ts * params.down_sample * 2.0
            idx = 0
            for line in f:
                line = line.strip()
                if not line:
                    continue
                parts = line.split(",")
                if len(parts) >= 13:
                    time_data.append(idx * machine_ts)
                    id_data.append(float(parts[0]))
                    iq_data.append(float(parts[1]))
                    rpm_data.append(float(parts[2]))
                    ud_data.append(float(parts[4]))
                    uq_data.append(float(parts[5]))
                    KE_data.append(float(parts[11]))
                    Js_data.append(float(parts[12]))
                    idx += 1

        total_points = len(time_data)
        if total_points > 2000:
            step = total_points // 2000
            time_data = time_data[::step]
            id_data = id_data[::step]
            iq_data = iq_data[::step]
            rpm_data = rpm_data[::step]
            ud_data = ud_data[::step]
            uq_data = uq_data[::step]
            KE_data = KE_data[::step]
            Js_data = Js_data[::step]

        return {
            "success": True,
            "time": time_data,
            "id": id_data,
            "iq": iq_data,
            "rpm": rpm_data,
            "ud": ud_data,
            "uq": uq_data,
            "KE_identified": KE_data[-1] if KE_data else 0,
            "Js_identified": Js_data[-1] if Js_data else 0,
            "total_points": total_points,
        }

    except subprocess.TimeoutExpired:
        raise HTTPException(status_code=500, detail="Simulation timeout")
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        try:
            shutil.rmtree(workdir, ignore_errors=True)
        except Exception:
            pass