# MATLAB-Autoware

MATLAB/Simulink reimplementation of the Autoware planning and control chain for the **ATOM** four-wheel-steering (4WS) vehicle. It reproduces Autoware's lane-driving pipeline (velocity smoother, planning validator, lateral and longitudinal control) and the parking module, but changes the controller in the lane-drving to be 4WS oriented Stanley and updates the parking module to have a costumized Hybrid A\* planner followed by a nonlinear MPC solved with **acados**.

- 2WS and 4WS vehicle models (counter-phase and in-phase rear steering)
- Lane-following scenario: velocity smoother, planning validator, Stanley lateral control with 4WS, PID longitudinal control
- Parking scenario: Hybrid A\* on occupancy grids, time parameterisation, parking velocity smoother, acados NMPC on front and rear steering rates
- Validation against the Autoware C++ implementation (`validation/`)

## Requirements

| Component | Version | Needed for |
|---|---|---|
| OS | Linux x86-64 (tested on Ubuntu) | generated MEX binaries are `.mexa64` |
| MATLAB + Simulink | R2026a | all models (saved in R2026a; older releases cannot open them) |
| C compiler | gcc supported by MATLAB R2026a (`mex -setup C`) | compiling the acados solver and its S-function |
| [acados](https://github.com/acados/acados) | built from source, with the class-based MATLAB interface (`AcadosOcp`, `AcadosOcpSolver`) | generating the NMPC solver (QP solver: HPIPM, built by default) |
| CasADi for MATLAB | 3.4.0 – 3.6.7, in `<acados-root>/external/casadi-matlab` | symbolic NMPC model |
| Optimization Toolbox | optional | the `fmincon` rows of `ATOM_parking_chain.slx` and the LUT generator |

## Getting started

### 1. Install acados (once)

Follow the [acados installation guide](https://docs.acados.org/installation/index.html). Summary:

```bash
git clone https://github.com/acados/acados.git
cd acados
git submodule update --recursive --init
mkdir -p build && cd build
cmake ..
make install -j4
```

Then install CasADi for MATLAB into `<acados-root>/external/casadi-matlab`, as described in the [acados MATLAB interface guide](https://docs.acados.org/matlab_octave_interface/index.html).

### 2. Clone this repository

```bash
git clone https://github.com/atom-urop/MATLAB-Autoware.git
cd MATLAB-Autoware
```

### 3. Start MATLAB with the acados environment

MATLAB inherits environment variables only from the shell that starts it. From the repository root:

```bash
export ACADOS_INSTALL_DIR=<acados-root>
export MATLABPATH=$MATLABPATH:$ACADOS_INSTALL_DIR/external/casadi-matlab/
export MATLABPATH=$MATLABPATH:$ACADOS_INSTALL_DIR/interfaces/acados_matlab_octave/
export MATLABPATH=$MATLABPATH:$ACADOS_INSTALL_DIR/interfaces/acados_matlab_octave/acados_template_mex/
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ACADOS_INSTALL_DIR/lib:$PWD/Acados/build
matlab
```

### 4. Run the simulation

In the MATLAB Command Window:

```matlab
ATOM_Simulink_init
```

`ATOM_Simulink_init` sets the MATLAB path, loads the parameters and look-up tables, creates the Simulink buses, opens `models/ATOM_simple_planning_simulator.slx`, and generates and compiles the acados NMPC solver into `Acados/c_generated_atom/` and `Acados/build/` (these folders are not tracked by git). Then press **Run** in the model.

### 5. Expected result

The simulation runs to the end. The model's stop callback opens the result figures, docked in one Figures window:

- the parking plan on the occupancy grid (`plotPlan`)
- the controller references (`PlotControllerReferences`)
- the controller tracking (`PlotControllerTracking`)

## Repository structure

```
MATLAB-Autoware/
├── ATOM_Simulink_init.m        entry point: path, parameters, LUTs, buses, model, acados solver
├── models/
│   ├── ATOM_simple_planning_simulator.slx   main model: lane following + parking (acados NMPC)
│   ├── ATOM_parking_chain.slx               parking test bench: two fmincon NMPC rows, one acados row
│   └── legacy/                              2WS and lateral-only 4WS models
├── Acados/                     NMPC formulation (model, cost, constraints, solver options)
│   └── Analysis/               NMPC predictions vs. actual vehicle states
├── Buses/                      Simulink bus definitions
├── Parameters/                 vehicle, velocity smoother, planning validator, Stanley and PID parameters
├── LUT/                        4WS look-up tables and the script that generates them
├── Functions/
│   ├── freespace_planner/      Hybrid A* planner, collision checking, path-to-trajectory conversion
│   ├── motion_utils/           nearest-point search, longitudinal offsets, quaternion/yaw conversion
│   ├── vehicle_model/          2WS/4WS vehicle dynamics
│   ├── velocity_smoother/      curvature, 4WS steering angles, steering-rate velocity limit
│   └── analysis/               plots called after a simulation
└── validation/
    ├── vehicle_model_2ws_vs_4ws/   2WS vs. 4WS vehicle model, data from Autoware ROS 2 bags
    └── velocity_smoother_cpp/      MATLAB vs. Autoware C++ velocity smoother
```

## Models

| Model | Content |
|---|---|
| `models/ATOM_simple_planning_simulator.slx` | Main model. Lane-follower scenario (velocity smoother, planning validator, Stanley lateral control, PID longitudinal control) and parking scenario (Hybrid A\*, time parameterisation, parking velocity smoother, acados NMPC with feed-forward + P longitudinal control) on the 4WS vehicle model |
| `models/ATOM_parking_chain.slx` | Parking test bench with three parallel chains: two with the `fmincon` NMPC, one with the acados NMPC. Open it with `open_system('ATOM_parking_chain')` after `ATOM_Simulink_init` |
| `models/legacy/ATOM_simple_planning_2WS.slx` | Lateral pipeline with the 2WS vehicle on six test trajectories |
| `models/legacy/ATOM_pred_sim_variable_rear_ratio_lateral.slx` | Lateral-only pipeline with predictive Stanley control and variable rear-steering ratio |

## Analysis and validation

Run `ATOM_Simulink_init` first, so that `Functions/` is on the path.

**NMPC predictions vs. actual states** (after a simulation of `ATOM_parking_chain`):

```matlab
run(fullfile('Acados', 'Analysis', 'atom_nmpc_analysis.m'))
```

**2WS vs. 4WS vehicle model** (data recorded from the Autoware planning simulator):

```matlab
addpath(fullfile('validation', 'vehicle_model_2ws_vs_4ws'))
PlotComparisonBetween4WSand2WSVehicleModel
PlotVehicle
```

**MATLAB vs. Autoware C++ velocity smoother:**

```matlab
addpath(fullfile('validation', 'velocity_smoother_cpp'))
CompareWithCpp_front_rear
Plot2WSvs4WS
PlotProofOfLiveTopicFromPlanningSimulator
PrintLutYaml          % prints the LUT in YAML format for the Autoware parameter file
```

The validation scripts clear the workspace; run `ATOM_Simulink_init` again before the next simulation.

## Regenerating the look-up tables

`LUT/Curvature_computation_4WS.m` recomputes `LUT_gain4WS_computation.mat` and `LUT_rr_computation.mat` and saves them into `LUT/`. It takes a long time, overwrites the committed tables and requires the Optimization Toolbox.

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `Undefined function 'AcadosOcp'` | acados MATLAB interface not on the path | start MATLAB from the shell of step 3 |
| `libacados.so: cannot open shared object file` | `LD_LIBRARY_PATH` not set | start MATLAB from the shell of step 3 |
| `No supported compiler was found` | MEX compiler not configured | `mex -setup C` |
| `Run ATOM_Simulink_init as a whole script ...` | the script was started with *Run Section* | press **Run** or type `ATOM_Simulink_init` |
| `Undefined function 'vehicleParameters'` | `ATOM_Simulink_init` was not run | run it before opening a model |

## Versions

The state of the repository before the cleanup is kept as tag [`v0.1-pre-cleanup`](https://github.com/atom-urop/MATLAB-Autoware/tree/v0.1-pre-cleanup).

## Authors

Hussein Ghaddar, Andrea Ricetti. Developed within the ATOM project (UROP programme, Politecnico di Torino).

## License

No license has been chosen yet. Contact the authors before reusing the code.
