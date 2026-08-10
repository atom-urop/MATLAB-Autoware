%% ATOM_Simulink_init.m

clc;
clear;
close all;


%% Add folders to MATLAB path- Andrea

% addpath("/home/andrea-ricetti/Documenti/MATLAB/Parameters");
% addpath("/home/andrea-ricetti/Documenti/MATLAB/Buses");
% addpath("/home/andrea-ricetti/Documenti/MATLAB/Functions");

%% Add folders to MATLAB path - Hussein

addpath("/home/husain5/MATLAB-Autoware/Parameters");
addpath("/home/husain5/MATLAB-Autoware/Buses");
addpath("/home/husain5/MATLAB-Autoware/Functions");

%% Load vehicle parameters

vehicleParameters;
velocity_smoother_params;
planning_validator_params;
stanley_params;
PID_Longitudinal_Velocity_Controller_Parameters;


%% Create Simulink buses

createBuses;


%% Open Simulink model

open_system("ATOM_simple_planning_simulator.slx");

%open_system("ATOM_planning_sim_variable_rear_ratio.slx");

%open_system("ATOM_planning_MPC.slx");

%% To plot the actual velocity and acceleration along the path, at the end of the simulation run:
%plotVehicleRun(out.veh_log)