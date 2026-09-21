function yaw = QuaternionToYaw(qx,qy,qz,qw)

yaw = zeros(length(qx),1);

for i = 1:length(qx)

    sqx = qx(i)*qx(i);
    sqy = qy(i)*qy(i);
    sqz = qz(i)*qz(i);
    sqw = qw(i)*qw(i);
    
    
    sarg = -2*(qx(i)*qz(i) - qw(i)*qy(i)) / ...
        (sqx + sqy + sqz + sqw);
    
    
    if sarg <= -0.99999
    
        yaw(i) = -2*atan2(qy(i),qx(i));
    
    
    elseif sarg >= 0.99999
    
        yaw(i) = 2*atan2(qy(i),qx(i));
    
    
    else
    
        yaw(i) = atan2( ...
            2*(qx(i)*qy(i) + qw(i)*qz(i)), ...
            sqw + sqx - sqy - sqz);
    
    end

end 
end