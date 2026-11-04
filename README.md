# Own flight controller - Step 2: balance control + 45-degree out-and-back

The C controller decides what the drone does and when. Python only passes data and draws it.

```
 C controller (core/)  --4 motor commands (0..1)-->  Python physics (replaces PX4/Gazebo)
        ^                                                       |
        '-- "sensor readings": position, speed, tilt, gyro  <---'
                                                                |
                                                      VPython quadcopter display
```

## The flight sequence (all decided inside the C code)

1. **LIFT** vertically to the hover point, then **HOVER** for `hover_time`.
2. **OUTBOUND**: straight line forward and upward at 45 degrees above horizontal (default 2 m forward, 2 m up).
3. **RETURN**: the same straight line, backwards, to the hover point (after a short `far_hold` pause).
4. **DESCEND** vertically (slow near the ground) and **LAND** on the starting point.

Adjustable: hover height, climb speed, hover time, descend speed, diagonal length, diagonal angle,
diagonal speed, pause at the far end. (`diag_horiz = 0` skips the diagonal = the Step 1 flight.)

## Folder layout

| Path | What it is | Goes on the MCU? |
|---|---|---|
| `core/controller.c/.h` | The portable controller (all decisions live here) | **Yes** |
| `dll/dll_wrapper.c` | PC-only adapter so Python can call the core | No |
| `build.py` | Compiles the core + wrapper into `build/controller.dll` | No |
| `python/physics.py` | Simulated drone: thrust, gravity, tilt, motor lag | No (real hardware replaces it) |
| `python/controller_lib.py` | Loads the DLL | No |
| `python/quad_view.py` | VPython quadcopter display (tilts, spinning rotors, path trail) | No |
| `python/run_sim.py` | Runs controller + physics (+ display) | No |
| `python/test_flight.py` | Automatic pass/fail checks | No |
| `python/make_video.py` | Renders the flight to an MP4 (side view) | No |

## How to run (Windows)

You need 64-bit Python 3.9+ and a C compiler.

```
pip install vpython matplotlib
pip install ziglang        # easiest C compiler (skip if you already have gcc / Visual Studio)

cd drone_ctrl
python build.py                          # 1. compile the C code -> build\controller.dll
python python\test_flight.py             # 2. automatic checks (expect 30 / 30 passed)
python python\run_sim.py --headless      # 3. one flight, text output only
python python\run_sim.py --autostart     # 4. quadcopter window in your browser (or click Start)
```

In the window: **Start**, **Reset**, and **Gust** (kicks the drone while it flies, so you can see the balance
controller catch it). Options: `--diag-horiz 3 --diag-angle 45 --diag-speed 1.5 --hover-time 8 --speed 0.5`
(`--speed 0.5` = slow motion).

**Recording a video of the display:** run step 4, then record the browser tab with Windows Game Bar
(`Win + Alt + R`) or OBS. The whole sequence takes about 20 s. `python python\make_video.py` makes a simple
2D side-view MP4 of the same flight without screen recording.

If `python build.py` finds no compiler: `pip install ziglang`, then run it again.
If `pip install vpython` fails, try Python 3.11.

## Which C function does what, and how it is tested

| Layer | C function | Status | Tested by `test_flight.py` |
|---|---|---|---|
| 1 Sensor input | `sensors_process()` | stub (pass-through) | - |
| 2 State estimation | `estimator_update()` | stub (copy) | - |
| 3 Mission / modes | `mode_update()` | **real**: IDLE > LIFT > HOVER > OUTBOUND > FAR_HOLD > RETURN > DESCEND > LANDED | idle before Start; phase order; hover time; diagonal skipped when length is 0 |
| 4 Setpoint generator | `setpoint_update()` | **real**: smooth moving target along each straight segment (speed + acceleration limits, acceleration feed-forward) | route accuracy; adjustable distance / angle / speed |
| 5a Horizontal position | `position_control_xy()` | **real**: position > speed > acceleration > wanted tilt | stays within 6 cm of the line; return follows the same line |
| 5b Altitude | `altitude_control()` | **real**: height > climb rate > thrust, tilt compensation | hover within 3 cm; hover point and descent |
| 6 Attitude | `attitude_control()` | **real**: wanted angle > wanted rotation rate | gust recovery |
| 7 Rate | `rate_control()` | **real**: PI on the gyro rate > roll / pitch torque | gust recovery |
| 8 Motor mixer | `mixer_update()` | **real**: thrust + roll/pitch > 4 motors, clamped 0..1 | motor order and signs |
| 9 Motor output | simulation now, MCU drivers later | adapter | - |

Top-level entry point: `ctrl_step()` calls layers 1 to 8 in order.

## Results from the test run (simulation)

| Check | Result |
|---|---|
| Deviation from the straight 45 deg line (out / back) | 2.8 cm / 2.6 cm |
| Flight angle outbound | 44.6 deg |
| Return point vs hover point | within 1 cm |
| Landing offset from the start point | about 0 cm |
| Maximum tilt in the sequence | 10 deg |
| Touchdown speed | 0.29 m/s |
| Gust of 3 rad/s roll + pitch in hover: level again after | under 1.5 s (5 different drone variants) |

## Known limits

- **Sensors are perfect.** Position, speed, tilt and gyro come straight from the simulation (no noise, no drift,
  no wind). Real position needs GPS / optical flow / a rangefinder and real tilt needs a filter on the IMU.
  Layers 1 and 2 are the next placeholders to fill.
- **Yaw is held at 0** (no heading control yet). Roll and pitch dynamics are simplified (no gyroscopic coupling).
- **Physics numbers are placeholders** (1.5 kg, thrust/weight 2, motor lag 50 ms, arm 0.2 m, inertia 0.03 kg m^2)
  until the real frame, motors and battery exist. The gains were chosen to work for motor lag up to about 70 ms
  and rotation inertia within about +-33 %; with a 90 ms lag the balance loop becomes too oscillatory. They
  will need re-tuning on the real drone.
- Touchdown is detected from height; the real drone needs a thrust / motion based method.
- The VPython window could not be rendered in my environment (it needs a browser). I tested the C code, the
  physics and the display logic; please check the visuals on your machine.

## Porting the core to the MCU later

`core/` follows the rules: no printf, malloc, files, clock or hardware calls; `dt` is passed in; all memory is in
the `Controller` struct; only `<math.h>` is used. Add `mcu_main.c` that reads sensors, calls `ctrl_step()` at a
fixed rate (for example 200 Hz) and writes `Motors` to the ESC drivers.