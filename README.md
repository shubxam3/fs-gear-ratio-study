# Formula Student acceleration simulation

A longitudinal vehicle model in MATLAB/Simulink that answers one design question: **which gear ratio gets a four-wheel-drive electric Formula Student car over the 75 m acceleration event fastest?**

I wrote it in 2026 as controls engineer at High-Speed Karlsruhe e.V. (Formula Student Electric, KIT), where the result fed into the choice of planetary gear ratio for the hub motors.

> The parameters in this repository are representative Formula Student values and public motor datasheet figures. They are not the team's car data, so the numbers the code prints are not the team's results.

## The question

A higher ratio gives more wheel torque off the line, but the motor reaches its speed limit sooner and the rotor inertia costs more. A lower ratio does the opposite. The tyres cap what can be used at low speed, and the rules cap electrical power at 80 kW. The optimum sits where these limits meet, and the useful output is less the single best number than the **band of ratios that lose under 10 ms**, because a planetary stage can only be built with certain tooth counts.

## Model

The car is a point mass with four driven wheels, one motor and one gearbox per wheel.

| Part | How it is modelled |
|---|---|
| Motor | Fischer TI085 peak curve: constant torque up to base speed, constant power up to maximum speed, then zero |
| Power limit | Torque is scaled down whenever total electrical power would exceed 80 kW |
| Gearbox | Fixed ratio with constant efficiency |
| Tyres | Either constant friction, or a simplified Magic Formula with load sensitivity, evaluated at the slip ratio of peak force |
| Load transfer | Axle loads from static distribution, aero downforce and longitudinal acceleration |
| Aero | Drag and downforce proportional to v², with a front/rear balance |
| Inertia | Wheels and motor rotors reflected to the car as extra mass: `m_eff = m + (4·J_wheel + 4·J_rotor·i²) / r²` |

At each time step, each axle delivers the smaller of the motor force and the tyre limit:

```
F_axle = min( 2 · T_motor(ω) · i · η / r ,  2 · Fx_max(Fz_axle / 2) )
a      = ( F_front + F_rear − F_drag − F_roll ) / m_eff
```

Load transfer depends on acceleration, and acceleration depends on the loads. The loop is broken by using the acceleration from the previous time step, which is what the Memory block does in the Simulink version.

## Running it

MATLAB script version (no toolboxes needed):

```matlab
main_sweep
```

This sweeps the ratio, prints the optimum and the 10 ms band, and saves a six-panel figure to `docs/sweep_result.png`.

Simulink version:

```matlab
cd simulink
build_model      % creates accel_baseline.slx from script
sweep_simulink   % runs the sweep through the model
```

`build_model.m` creates the block diagram programmatically, so the model is reproducible from text and can be reviewed in a diff.

## Repository layout

```
main_sweep.m             gear ratio sweep, report and plots
src/params_generic.m     all parameters in one place
src/run_accel.m          one 75 m run for one gear ratio
src/motor_torque.m       motor peak torque curve
src/mf_fx.m              simplified Magic Formula
simulink/build_model.m   builds the baseline Simulink model
simulink/model_params.m  parameters for the Simulink model
simulink/sweep_simulink.m  sweep through the Simulink model
```

## How the model developed

The model was built up in steps, each one added because the previous version gave an answer that did not hold up:

1. **Rear-wheel drive, closed-form traction limit.** Constant friction, load transfer solved analytically, lumped drag. Good enough to see the shape of the time-versus-ratio curve.
2. **Magic Formula tyre and aero.** Traction limit now changes with speed through downforce and with load through tyre load sensitivity.
3. **Four-wheel drive.** Separate front and rear axle loads and traction limits, wheel and rotor inertia, gearbox efficiency, 80 kW power limit.
4. **RK4 integration** as a check on the Euler scheme.
5. **Simulink model**, built from script, to cross-check the MATLAB loop with a fixed-step ode4 solver.
6. **Fine sweep and tolerance band**, to turn "the optimum" into a range the gearbox designers could pick tooth counts from.

## Limitations

- No wheel dynamics: the tyres are assumed to sit at the slip ratio of peak force, as if traction control were ideal. Real launches lose time here.
- No pitch dynamics or suspension; load transfer is quasi-static.
- When traction-limited, the model still divides by the effective mass including rotor inertia, which is slightly pessimistic.
- Motor and gearbox efficiency are constants, with no temperature or current dependence.
- The Simulink baseline uses constant tyre friction and has no power limit, so it runs faster than the full script model.
- The model has not been validated against measured runs in this repository.

## Author

Shubham Pathak, M.Sc. student in Mechatronics and Information Technology at KIT.
