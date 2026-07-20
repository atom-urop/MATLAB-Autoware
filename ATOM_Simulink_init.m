%% ATOM_Simulink_init.m

clc;
clear;
close all;


%% Add folders to MATLAB path

addpath("/home/andrea-ricetti/Documenti/MATLAB/Parameters");
addpath("/home/andrea-ricetti/Documenti/MATLAB/Buses");
addpath("/home/andrea-ricetti/Documenti/MATLAB/Functions");


%% Load vehicle parameters

vehicleParameters;
velocity_smoother_params;
planning_validator_params;


%% Create Simulink buses

createBuses;


%% Open Simulink model

open_system("ATOM_simple_planning_simulator.slx");