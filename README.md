# Own flight controller - Step 1: lift / hover / descend

The C controller decides what the drone does and when. Python only passes data and draws it.

```
 C controller (core/)  --motor commands (4 x 0..1)-->  Python physics (replaces PX4/Gazebo)
        ^                                                        |
        '------------ "sensor readings": height, climb rate <----'
                                                                 |
                                                       VPython quadcopter display
```

## Folder layout

| Path | What it is | Goes on the MCU? |
|---|---|---|
| `core/controller.c/.h` | The portable controller (all decisions live here) | **Yes** |
| `dll/dll_wrapper.c` | PC-only adapter so Python can call the core | No |
| `build.py` | Compiles the core + wrapper into `build/controller.dll` | No |
| `python/physics.py` | Simulated drone (mass, gravity, motor lag) | No (real hardware replaces it) |
| `python/controller_lib.py` | Loads the DLL | No |
| `python/quad_view.py` | VPython quadcopter display | No |
| `python/run_sim.py` | Runs controller + physics (+ display) | No |
| `python/test_flight.py` | Automatic pass/fail checks | No |

## How to run (Windows)

You need 64-bit Python 3.9+ and a C compiler.

```
pip install vpython
pip install ziglang        # easiest C compiler (skip if you already have gcc / Visual Studio)

cd drone_ctrl
python build.py            # 1. compile the C code -> build\controller.dll
python python\test_flight.py            # 2. automatic checks (expect 15 / 15 passed)
python python\run_sim.py --headless     # 3. one flight, text output only
python python\run_sim.py                # 4. quadcopter window: opens in your browser, click "Start"
```

Options for `run_sim.py`:

```
python python\run_sim.py --hover-height 3 --climb-speed 1.5 --hover-time 8 --descend-speed 1.0
python python\run_sim.py --autostart --speed 0.5      # start at once, half speed (slow motion)
```

If `python build.py` says no compiler was found: run `pip install ziglang` and try again.
If `pip install vpython` fails, try Python 3.11.
The display runs in a browser tab; the Reset button puts the drone back on the pad.

## Which C function controls what, and how it is tested

| Layer | C function | Step 1 status | Tested by (`test_flight.py`) |
|---|---|---|---|
| 1 Sensor input | `sensors_process()` | stub (pass-through) | - |
| 2 State estimation | `estimator_update()` | stub (copy) | - |
| 3 Mode / safety | `mode_update()` | **real**: IDLE > LIFT > HOVER > DESCEND > LANDED | idle before Start; phase order; hover lasts the set time |
| 4 Setpoint generator | `setpoint_update()` | **real**: smooth ramp, speed and acceleration limits | overshoot < 10 cm; works for other heights and speeds |
| 5 Position control | `altitude_control()` | **real**: height > climb rate > thrust, hover feed-forward + integrator | hover within 3 cm; robust when true hover thrust is 0.33 to 0.62 instead of the assumed 0.50 |
| 6 Attitude control | `attitude_control()` | stub (Step 2) | - |
| 7 Rate control | `rate_control()` | stub (Step 2) | - |
| 8 Motor mixer | `mixer_update()` | **real**: equal thrust now, roll/pitch/yaw mixing ready | equal motors; pitch goes to the right motors; output clamped 0..1 |
| 9 Motor output | simulation now, MCU drivers later | adapter | - |

Top-level entry point: `ctrl_step()` calls layers 1 to 8 in order.

## Adjustable parameters

`hover_height_m`, `climb_speed_mps`, `hover_time_s`, `descend_speed_mps` (see `Params` in `controller.h`;
the others - gains, accel limit, landing speed - have defaults in `ctrl_default_params()`).

## Results from my test run (simulation, no hardware)

Nominal flight (2 m, 5 s hover): hover error about 1 cm, overshoot about 4 cm, touchdown about 0.3 m/s.

## Known limits of Step 1

- Vertical motion only: no tilt, no balance control yet (Step 2).
- The physics model is idealised (no sensor noise, no wind). The controller sees exact height and speed.
- The physics numbers (1.5 kg, thrust/weight 2.0, motor lag 50 ms) are placeholders until the real frame, motors and battery are chosen.
- Touchdown is detected from height; the real drone will need a different method (thrust and motion based).
- The VPython window could not be rendered in my environment; the logic was checked, and the C and physics tests passed. Please check the visuals on your machine.

## Porting the core to the MCU later

`core/` already follows the rules: no printf, malloc, files, clock or hardware calls; `dt` is passed in; all memory is in the `Controller` struct.
Add `mcu_main.c` that reads sensors, calls `ctrl_step()` at a fixed rate, and writes `Motors` to the ESC drivers.
Only `<math.h>` (`sqrtf`, `fabsf`) is used. On a small MCU without a float unit, floats are emulated and slower;
that is a later decision based on the chip.
