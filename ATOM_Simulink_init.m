%% ATOM_Simulink_init.m

clc;
clear;
close all;


%% Add folders to MATLAB path

addpath("/home/andrea-ricetti/Documenti/MATLAB/Parameters");
addpath("/home/andrea-ricetti/Documenti/MATLAB/Buses");


%% Load vehicle parameters

vehicleParameters;


%% Create Simulink buses

createBuses;


%% Open Simulink model

% open_system("AutowareSimulator.slx");