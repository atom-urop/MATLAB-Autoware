function [qx,qy,qz,qw] = yawToQuaternion(yaw)


% quaternion from yaw only

qx = 0;
qy = 0;
qz = sin(yaw/2);
qw = cos(yaw/2);

end