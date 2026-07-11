% clc
% clear all
% close all

% --------------
%% /localization/kinematic_state
%% --------------


% 1. SOTTO-ELEMENTI: Geometria Base (Punti e Vettori)
% Struttura per Point position (x, y, z) e Vector3 (linear/angular)
elem3D = Simulink.BusElement;
elem3D.DataType = 'double';
elem3D.Dimensions = 1;

PointBus = Simulink.Bus;
elemX = elem3D; elemX.Name = 'x';
elemY = elem3D; elemY.Name = 'y';
elemZ = elem3D; elemZ.Name = 'z';
PointBus.Elements = [elemX, elemY, elemZ];

% Struttura per Quaternion orientation (x, y, z, w)
QuaternionBus = Simulink.Bus;
elemW = elem3D; elemW.Name = 'w';
QuaternionBus.Elements = [elemX, elemY, elemZ, elemW];


% 2. SOTTO-ELEMENTI: Pose e Twist (Con Covarianza)
% Sotto-bus per "Pose"
PoseBus = Simulink.Bus;
p1 = Simulink.BusElement; p1.Name = 'position'; p1.DataType = 'Bus: PointBus';
p2 = Simulink.BusElement; p2.Name = 'orientation'; p2.DataType = 'Bus: QuaternionBus';
PoseBus.Elements = [p1, p2];

% Sotto-bus per "Twist"
TwistBus = Simulink.Bus;
t1 = Simulink.BusElement; t1.Name = 'linear'; t1.DataType = 'Bus: PointBus'; % Riutilizza il vettore 3D
t2 = Simulink.BusElement; t2.Name = 'angular'; t2.DataType = 'Bus: PointBus';
TwistBus.Elements = [t1, t2];


% %% 3. ELEMENTI INTERMEDI: PoseWithCovariance e TwistWithCovariance
% covElem = Simulink.BusElement;
% covElem.Name = 'covariance';
% covElem.DataType = 'double';
% covElem.Dimensions = 36; % float64[36] che vedi a schermo
% 
% PoseWithCovarianceBus = Simulink.Bus;
% pc1 = Simulink.BusElement; pc1.Name = 'pose'; pc1.DataType = 'Bus: PoseBus';
% PoseWithCovarianceBus.Elements = [pc1, covElem];
% 
% TwistWithCovarianceBus = Simulink.Bus;
% tc1 = Simulink.BusElement; tc1.Name = 'twist'; tc1.DataType = 'Bus: TwistBus';
% TwistWithCovarianceBus.Elements = [tc1, covElem];


% 4. IL BUS PRINCIPALE: KinematicStateBus (Unione finale SENZA COVARIANZA)
KinematicStateBus = Simulink.Bus;

% Elemento Pose
mainElem1 = Simulink.BusElement;
mainElem1.Name = 'pose';
mainElem1.DataType = 'Bus: PoseBus';

% Elemento Twist
mainElem2 = Simulink.BusElement;
mainElem2.Name = 'twist';
mainElem2.DataType = 'Bus: TwistBus';

KinematicStateBus.Elements = [mainElem1, mainElem2];


% --------------
%% /localization/acceleration
%% --------------

AccelerationBus = Simulink.Bus;


mainElem2 = Simulink.BusElement;
mainElem2.Name = 'Acceleration';
mainElem2.DataType = 'Bus: TwistBus';

AccelerationBus.Elements = mainElem2;




% ---------------------
%% /planning/trajectory
% ---------------------

% Variabili dinamiche in float32 (singola precisione)

elemFloat = Simulink.BusElement; elemFloat.DataType = 'single';

tp3 = elemFloat; tp3.Name = 'longitudinal_velocity_mps';
tp4 = elemFloat; tp4.Name = 'lateral_velocity_mps';
tp5 = elemFloat; tp5.Name = 'acceleration_mps2';
tp6 = elemFloat; tp6.Name = 'heading_rate_rps';
tp7 = elemFloat; tp7.Name = 'front_wheel_angle_rad';
tp8 = elemFloat; tp8.Name = 'rear_wheel_angle_rad';


TrajectoryBus = Simulink.Bus;

TrajectoryBus.Elements = [mainElem1, tp3, tp4, tp5, tp6, tp7, tp8];


% ---------------------
%% /control/command/control_cmd
% ---------------------

% 2. BUS COMANDO LATERALE (Lateral Control Command)
LateralBus = Simulink.Bus;

% Variabili dinamiche (float32 -> single)

lat3 = elemFloat; lat3.Name = 'front_steering_tire_angle';

%
%
%
% MODIFICA 4WS
lat4 = elemFloat; lat4.Name = 'rear_steering_tire_angle';  
%
%
%                 



LateralBus.Elements = [lat3, lat4];


% 3. BUS COMANDO LONGITUDINALE (Longitudinal Control Command)

LongitudinalBus = Simulink.Bus;

% Variabili dinamiche (Velocity, Acceleration, Jerk)
long3 = elemFloat; long3.Name = 'velocity';
long4 = elemFloat; long4.Name = 'acceleration';
long5 = elemFloat; long5.Name = 'jerk';


LongitudinalBus.Elements = [long3, long4, long5];

% 4. IL BUS PRINCIPALE: ControlCMDBus 
ControlCMDBus = Simulink.Bus;

% Elemento Lateral
mainElemA = Simulink.BusElement;
mainElemA.Name = 'Lateral';
mainElemA.DataType = 'Bus: LateralBus';

% Elemento Twist
mainElemB = Simulink.BusElement;
mainElemB.Name = 'Longitudinal';
mainElemB.DataType = 'Bus: LongitudinalBus';

ControlCMDBus.Elements = [mainElemA, mainElemB];