%% ATOM_Simulink_init.m

clc;
clear;
close all;
%% 


%% Repository root (folder that contains this file)
repoRoot = fileparts(mfilename('fullpath'));
if isempty(repoRoot)
    error('Run ATOM_Simulink_init as a whole script (Run button or by name), not by section.');
end

%% Add folders to the MATLAB path
addpath(fullfile(repoRoot, "Parameters"));
addpath(fullfile(repoRoot, "Buses"));
addpath(genpath(fullfile(repoRoot, "Functions")));
addpath(fullfile(repoRoot, "LUT"));
addpath(fullfile(repoRoot, "Acados"));


%% Load vehicle parameters

vehicleParameters;
velocity_smoother_params;
planning_validator_params;
stanley_params;
PID_Longitudinal_Velocity_Controller_Parameters;



% %% Load LUTs-Andrea
% 
% load('LUT_gain4WS_computation.mat');
% load('LUT_rr_computation.mat');

%% Load LUTs-Andrea

load("/home/husain5/MATLAB-Autoware/LUT/LUT_gain4WS_computation.mat")
load("/home/husain5/MATLAB-Autoware/LUT/LUT_rr_computation.mat")

%% Create Simulink buses

createBuses;


%% Open Simulink model

%open_system("ATOM_simple_planning_2WS.slx");

%open_system("ATOM_pred_sim_variable_rear_ratio_lateral.slx");

open_system("ATOM_simple_planning_simulator.slx");

%open_system("ATOM_parking_chain");


%% To plot the actual velocity and acceleration along the path, at the end of the simulation run:
%plotVehicleRun(out.veh_log)


%% Vehicle shape margin for parking scenario
margin = 0.5; %from the yaml file vehicle_shape_margin_m: 0.5    % vehicle_shape_margin_m
p  = vehicle.param;
Lv = p.front_overhang + p.wheel_base + p.rear_overhang + margin;    % 3.420
Wv = p.wheel_tread + p.left_overhang + p.right_overhang + margin;   % 2.656
shape.back  = -(p.rear_overhang + margin/2);                        % -0.710
shape.front = Lv + shape.back;                                      % +2.710
shape.left  =  Wv/2;                                                % +1.328
shape.right = -Wv/2;                                                % -1.328

%% acados NMPC solver (parking controller)
clear mex                                                % unload an old solver MEX before rebuilding
oldFolder = cd(fullfile(repoRoot, "Acados"));            % generated code -> Acados/c_generated_atom, Acados/build
[atomSolver, atomCfg] = generate_atom_solver(vehicle);
cd(oldFolder);