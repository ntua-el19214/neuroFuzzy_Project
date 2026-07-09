# 7-DOF Vehicle Dynamics Simulation: Fuzzy vs. Gain-Scheduled LQR Torque Vectoring for a Formula Student EV

A MATLAB simulation comparing two torque-vectoring strategies — a fuzzy-logic controller and a gain-scheduled LQR state-space controller — against a no-torque-vectoring baseline, on a nonlinear 7-DOF vehicle model of a Formula Student electric race car.

## What this is

[Formula Student](https://www.imeche.org/events/formula-student) (FSAE) is a student engineering competition in which teams design, build, and race a small formula-style car. Cars are scored across events run on narrow, technical tracks — skid pad, autocross, endurance, acceleration — with a minimum turn diameter of 9 m and minimum track width of 3 m. Rotating the car quickly through tight, low-speed corners matters as much as raw grip.

This repo simulates a **7-DOF vehicle model** — longitudinal velocity, lateral velocity, yaw rate, plus four independent wheel speeds — driven by a **Pacejka Magic Formula 6.2** tire model (Hoosier R20 compound). The tire coefficients were fit to real TTC (Tire Testing Consortium) data by Prom Racing Formula Student NTUA's Vehicle Dynamics department. On top of that model, it compares three ways of distributing motor torque across four independently-driven wheels:

- **No torque vectoring** — an open-loop baseline torque split.
- **Gain-scheduled LQR ("state space")** — a linear-quadratic regulator designed on the linearized system, with a PI pre-filter for steady-state correction.
- **Fuzzy logic** — a rule-based fuzzy inference controller.

## Why torque vectoring matters for FSAE

A car with four independently driven wheels can redistribute torque left-to-right to actively assist rotation, rather than relying solely on tire slip angle and weight transfer. On the tight, low-speed, high-turn-density tracks FSAE events use, faster car rotation into and out of corners translates directly into lap time — which is why torque vectoring is a common area of investment for electric FSAE teams with independently driven wheels.

## Vehicle model

The equations of motion follow the standard planar bicycle-model form extended to four wheels: `vx_dot`, `vy_dot`, `psi_ddot` (yaw acceleration), and per-wheel `omega_dot`. Tire forces are computed from slip ratio and slip angle at each wheel via the Pacejka model (`Tires/R20/F_longit.m`, `Tires/R20/F_lateral.m`).

Known simplifying assumptions (see the presentation for full derivations):
- Aerodynamic drag is modeled acting only along the vehicle's x-axis — a simplification, not fully accurate for a car generating side force in a corner.
- Each tire's self-aligning torque (produced by contact-patch deformation) is neglected.

## Controllers

**State-space controller** (`gainScheduling.m`, `Vehicle/motorTorque.m`, matrices built in `helpSetupTheProblem.m`): two parts — a PI "pre-filter" that corrects for steady-state error and model discrepancy, and an LQR designed on the linearized system `x = [beta; psi_dot]`, `u = [Fx_fl Fx_fr Fx_rl Fx_rr]`, with cost `J = ∫ (xᵀQx + uᵀRu) dt`, linearized around straight-line driving.

**Fuzzy controller** (`Vehicle/fuzzyTorqueVectoring.m`): takes yaw-rate error and yaw-acceleration error as inputs, and outputs a target yaw moment `Mz` through a rule-based fuzzy inference system (Gaussian membership functions, a 7x7 rule table). `Mz` is converted to a front/rear `ΔFx` and distributed to each wheel weighted by vertical load (`Fz`).

**Tradeoffs**: the state-space approach is mathematically well-established and computationally cheap to evaluate online, but struggles with the system's heavy nonlinearity and is complex and time-consuming to tune (each new operating point requires re-linearizing and re-solving the LQR). The fuzzy approach is comparatively straightforward to implement and tune by hand and handles nonlinearity natively, but it depends on expert domain knowledge to design the rule base and has no structured multi-input-multi-output design methodology to fall back on.

## Repo structure

```
neuroFuzzy_Project/
├── README.md                                  this file
├── .gitignore
├── Fuzzy Torque Vectoring Presentation.pdf     full writeup: theory, derivations, plots
├── runControllerComparison.m                   ENTRY POINT — runs all 3 controller modes and plots the comparison
├── helpSetupTheProblem.m                       builds the linearized model + LQR gains consumed by the state-space controller
├── gainScheduling.m                            blends LQR gains across operating points (see Known limitations)
├── vehicleOutputFcn.m                          ode45 OutputFcn — logs auxiliary sim data (forces, torques, errors) each step
├── vizMF.m                                     diagnostic plot of the fuzzy controller's membership functions
├── Vehicle/
│   ├── AMK-FSAE Motors Data.xlsx               motor torque/speed curve source data
│   ├── Motors.m                                fits the motor torque-speed envelope
│   ├── deltaFunc.m                             open-loop steering input (the slalom maneuver)
│   ├── fuzzyTorqueVectoring.m                  the fuzzy logic controller
│   ├── fuzzyMFParams.m                         shared fuzzy membership-function parameters (controller + vizMF.m)
│   ├── calcMembership.m                        shared Gaussian fuzzification helper
│   ├── motorTorque.m                           state-space controller's torque allocation, grip-limited via TireMaxFx
│   └── nonLinearVehicleModel.m                 the 7-DOF equations of motion (ode45 RHS)
├── Tires/
│   ├── calculateSlipRatio.m                    per-wheel longitudinal slip ratio
│   ├── maxFxForSaFzCombination.m               grip-limit lookup: max Fx vs. slip angle x vertical load
│   └── R20/
│       ├── F_lateral.m                         Pacejka lateral force model
│       ├── F_longit.m                          Pacejka longitudinal force model
│       └── tire_plot.m                         standalone tire-curve visualization
└── Real Vehicle Data/
    ├── P22_Endurance_Data.csv                  raw telemetry, not currently consumed by any code path
    ├── Post_Season_Testing_P20_Marathonas.csv  raw telemetry that informed the LQR operating-point selection
    ├── Post_Season_Testing_P20_Marathonas_Map_2_3.csv
    └── importfile.m                            auto-generated CSV import helper
```

## How to run

**Prerequisites:** MATLAB with:
- Symbolic Math Toolbox (used in `helpSetupTheProblem.m`)
- Control System Toolbox (`icare`, used for the LQR design)
- Curve Fitting Toolbox (`fit`, used in `Vehicle/Motors.m` and `Tires/maxFxForSaFzCombination.m`)
- Optimization Toolbox (`fmincon`, used in `Tires/maxFxForSaFzCombination.m`)

**Steps:**
1. Clone the repo.
2. Open MATLAB and `cd` into the repo root.
3. Run `runControllerComparison.m`.

Note: the first run computes the tire grip-limit lookup (`vehicle.TireMaxFx`) via a grid search over slip angle x vertical load using `fmincon` — roughly 10,000 optimizer calls. This can take a few minutes; it's a one-time cost per MATLAB session, not something that reruns per simulation step.

## Results

The controllers are evaluated against an open-loop steering input modeling an extreme slalom (inspired by the narrowest legal FSAE slalom cone spacing), chosen specifically to stress-test each controller rather than represent typical driving.

| TV Type | Sum ψ̇ Error | Avg Velocity [m/s] |
|---|---|---|
| No TV | 0.299 | 7.88 |
| State Space TV | 0.125 (−58.2%) | 9.23 (+17.3%) |
| Fuzzy TV | 0.025 (−91.6%) | 9.54 (+21.1%) |

These exact numbers are from the presentation and were produced by the pre-cleanup code. They may shift slightly now that the air-density inconsistency and the `TireMaxFx` grip-limit wiring have been fixed — but the qualitative conclusion (fuzzy outperforms state-space, both outperform no torque vectoring) is expected to hold.

## Data

`Real Vehicle Data/` contains raw CAN-bus telemetry CSVs from real car testing and competition sessions: `P22_Endurance_Data.csv` (~2.3 MB), `Post_Season_Testing_P20_Marathonas.csv` (~10.8 MB), and a second session/map configuration (~7.4 MB). `P22_Endurance_Data.csv` isn't currently read by any code path — it's kept for reference/future work. The P20 data historically informed the LQR linearization operating-point selection (see the provenance comment inside `helpSetupTheProblem.m`, above the commented-out `importfile(...)` + `histogram2(...)` block).

## Known limitations

- The evaluation maneuver is a synthetic open-loop slalom, not closed-loop path tracking.
- The LQR is designed around a single linearization point rather than true gain scheduling across multiple operating points. The code supports multi-point scheduling (`gainScheduling.m` has a full triangular-membership interpolation branch for it), but it's currently dormant because `helpSetupTheProblem.m` only ever computes one operating point.
- Self-aligning torque and some combined-slip coupling effects are neglected in the tire model.

## References/credits

- Prom Racing Formula Student NTUA — Vehicle Dynamics department, TTC tire data.
- Pacejka Magic Formula 6.2 tire model.
- Jazar, N. (2018). *Vehicle Dynamics: Theory and Application*, 3rd Edition. Springer-Verlag New York.
- B. Jäger, P. Neugebauer, R. Kriesten, N. Parspour and C. Gutenkunst, "Torque-vectoring stability control of a four wheel drive electric vehicle," 2015 IEEE Intelligent Vehicles Symposium (IV). doi:10.1109/IVS.2015.7225818.

See [`Fuzzy Torque Vectoring Presentation.pdf`](./Fuzzy%20Torque%20Vectoring%20Presentation.pdf) in this repo for the fuller writeup, including derivations and plots.
