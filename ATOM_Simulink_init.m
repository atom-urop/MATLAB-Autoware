%% ATOM_Simulink_init.m

clc;
clear;
close all;


% % Add folders to MATLAB path - Andrea
% -
% addpath("/home/andrea-ricetti/Documenti/MATLAB/Parameters");
% addpath("/home/andrea-ricetti/Documenti/MATLAB/Buses");
% addpath("/home/andrea-ricetti/Documenti/MATLAB/Functions");
% addpath("/home/andrea-ricetti/Documenti/MATLAB/LUT");


%% Add folders to MATLAB path - Hussein

addpath("/home/husain5/MATLAB-Autoware/Parameters");
addpath("/home/husain5/MATLAB-Autoware/Buses");
addpath("/home/husain5/MATLAB-Autoware/Functions");
addpath("/home/husain5/MATLAB-Autoware/LUT");


%% Load vehicle parameters

vehicleParameters;
velocity_smoother_params;
planning_validator_params;
stanley_params;
PID_Longitudinal_Velocity_Controller_Parameters;

scenario = 'perpendicular_lot';    % 'perpendicular_lot' | 'empty_lot' | 'narrow_corridor'




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