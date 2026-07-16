%% createBuses.m

%% ============================================================
% BASIC TYPES
% =============================================================
N = 100;

doubleElem = Simulink.BusElement;
doubleElem.DataType = 'double';
doubleElem.Dimensions = 1;

NElem = Simulink.BusElement;
NElem.DataType = 'double';
NElem.Dimensions = N;

singleElem = Simulink.BusElement;
singleElem.DataType = 'single';
singleElem.Dimensions = 1;


%% ============================================================
% GEOMETRY BUS
% geometry_msgs/Point
% =============================================================

PointBus = Simulink.Bus;

x = doubleElem;
x.Name = 'x';

y = doubleElem;
y.Name = 'y';

z = doubleElem;
z.Name = 'z';

PointBus.Elements = [x y z];


%% geometry_msgs/Quaternion

QuaternionBus = Simulink.Bus;

qx = doubleElem;
qx.Name = 'x';

qy = doubleElem;
qy.Name = 'y';

qz = doubleElem;
qz.Name = 'z';

qw = doubleElem;
qw.Name = 'w';

QuaternionBus.Elements = [qx qy qz qw];



%% ============================================================
% NAV_MSGS ODOMETRY
% /localization/kinematic_state
% =============================================================


%% Pose

PoseBus = Simulink.Bus;

position = Simulink.BusElement;
position.Name = 'position';
position.DataType = 'Bus: PointBus';

orientation = Simulink.BusElement;
orientation.Name = 'orientation';
orientation.DataType = 'Bus: QuaternionBus';

PoseBus.Elements = [position orientation];


%% Twist

TwistBus = Simulink.Bus;

linear = Simulink.BusElement;
linear.Name = 'linear';
linear.DataType = 'Bus: PointBus';

angular = Simulink.BusElement;
angular.Name = 'angular';
angular.DataType = 'Bus: PointBus';

TwistBus.Elements = [linear angular];


%% KinematicState

KinematicStateBus = Simulink.Bus;

pose = Simulink.BusElement;
pose.Name = 'pose';
pose.DataType = 'Bus: PoseBus';

twist = Simulink.BusElement;
twist.Name = 'twist';
twist.DataType = 'Bus: TwistBus';


KinematicStateBus.Elements = [pose twist];



%% ============================================================
% /localization/acceleration
% =============================================================

AccelerationBus = Simulink.Bus;

acceleration = Simulink.BusElement;
acceleration.Name = 'acceleration';
acceleration.DataType = 'Bus: TwistBus';

AccelerationBus.Elements = acceleration;



%% ============================================================
% /planning/trajectory
% =============================================================


PointBusN = Simulink.Bus;

x = NElem;
x.Name = 'x';

y = NElem;
y.Name = 'y';

z = NElem;
z.Name = 'z';

PointBusN.Elements = [x y z];


%% geometry_msgs/Quaternion

QuaternionBusN = Simulink.Bus;

qx = NElem;
qx.Name = 'qx';

qy = NElem;
qy.Name = 'qy';

qz = NElem;
qz.Name = 'qz';

qw = NElem;
qw.Name = 'qw';

QuaternionBusN.Elements = [qx qy qz qw];

PoseBusN = Simulink.Bus;

positionN = Simulink.BusElement;
positionN.Name = 'position';
positionN.DataType = 'Bus: PointBusN';

orientationN = Simulink.BusElement;
orientationN.Name = 'orientation';
orientationN.DataType = 'Bus: QuaternionBusN';

PoseBusN.Elements = [positionN orientationN];


poseN = Simulink.BusElement;
poseN.Name = 'pose';
poseN.DataType = 'Bus: PoseBusN';



TrajectoryDataBus = Simulink.Bus;

fields={
    'longitudinal_velocity_mps'
    'lateral_velocity_mps'
    'acceleration_mps2'
    'heading_rate_rps'
    'front_wheel_angle_rad'
    'rear_wheel_angle_rad'
    };


elements=poseN;


for i=1:length(fields)

    e=Simulink.BusElement;
    e.Name=fields{i};
    e.DataType='double';
    e.Dimensions = 100;

    elements=[elements;e];

end


TrajectoryDataBus.Elements=elements;

%% ============================================================
% CONTROL COMMAND
% /control/command/control_cmd
% =============================================================


%% Lateral

LateralBus = Simulink.Bus;


frontSteer = doubleElem;
frontSteer.Name = 'front_steering_tire_angle';

rearSteer = doubleElem;                                 % Modified for 4WS
rearSteer.Name = 'rear_steering_tire_angle';            % Modified for 4WS


LateralBus.Elements = [
    frontSteer
    rearSteer
];


%% Longitudinal

LongitudinalBus = Simulink.Bus;


velocity = doubleElem;
velocity.Name = 'velocity';

acceleration = doubleElem;
acceleration.Name = 'acceleration';

jerk = doubleElem;
jerk.Name = 'jerk';


LongitudinalBus.Elements = [
    velocity
    acceleration
    jerk
];


%% Control command

ControlCMDBus = Simulink.Bus;


lateral = Simulink.BusElement;
lateral.Name = 'lateral';
lateral.DataType = 'Bus: LateralBus';


longitudinal = Simulink.BusElement;
longitudinal.Name = 'longitudinal';
longitudinal.DataType = 'Bus: LongitudinalBus';


ControlCMDBus.Elements = [
    lateral
    longitudinal
];



%% ============================================================
% VEHICLE INTERNAL STATE
% State vector:
%
% X = [x y yaw vx steer_front steer_rear acc]
%
% It's not a topic in Autoware. It's created to simulate the update of the
% state inside the node simple_planning_simulator_core
% =============================================================


VehicleInternalStateBus = Simulink.Bus;


x = doubleElem;
x.Name = 'x';

y = doubleElem;
y.Name = 'y';

yaw = doubleElem;
yaw.Name = 'yaw';

vx = doubleElem;
vx.Name = 'vx';

steer_front = doubleElem;
steer_front.Name = 'steer_front';

steer_rear = doubleElem;                    % Modified for 4WS
steer_rear.Name = 'steer_rear';

acc = doubleElem;
acc.Name = 'acc';


VehicleInternalStateBus.Elements = [
    x
    y
    yaw
    vx
    steer_front
    steer_rear
    acc
];