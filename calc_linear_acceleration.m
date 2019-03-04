function linear_acceleration = calc_linear_acceleration(thrust, disturbances, theta, linear_velocity)
%
% inputs: thrust vector (m1, m2, m3, m4)
%         disturbances vector (x,y,z)
%         theta vector (x,y,z)
%         linear velocity vector (x,y,z)
%
% output: linear acceleration vector (x,y,z)
%

m = .743; % mass of drone
g = 9.81; % acceleration due to gravity
rho = 1.225; % density of air at sea level
coef_d = 1.0; % coefficient of drag for the drone

% cross sectional areas of drone in x,y,z planes
areas = [.0197, .0197, .0512];

linear_acceleration = [0 0 0];
force = [0 0 0];
xyz_thrust = [0 0 0];
drag = [0 0 0];

% trigonometric functions for calculating pitch,roll,yaw (x,y,z) thrust 
xyz_thrust(1) = sin(theta(2))*cos(theta(1))*sum(thrust);
xyz_thrust(2) = sin(theta(1))*cos(theta(2))*sum(thrust);
xyz_thrust(3) = cos(theta(1))*cos(theta(2))*sum(thrust);

% angular position and magnitude on the pitch-roll (x,y) plane
angular_pos = atan2(xyz_thrust(1), xyz_thrust(2));
angular_mag = sqrt(xyz_thrust(1)^2 + xyz_thrust(2)^2);

% if the drone is pointed upwards, x and y thrust uses + theta(3) (z)
if(xyz_thrust(3) >= 0)
    xyz_thrust(1) = angular_mag * sin(angular_pos + theta(3));
    xyz_thrust(2) = angular_mag * cos(angular_pos + theta(3));
else
    xyz_thrust(1) = angular_mag * sin(angular_pos - theta(3));
    xyz_thrust(2) = angular_mag * cos(angular_pos - theta(3));  
end

% calculate drage and force in x,y,z 
for i=1:3
    drag(i) = .5*rho*linear_velocity(i)^2* areas(i)*coef_d;
    % if the drone is going backward (w.r.t its local coordinate system)
    if (linear_velocity(i) < 0)
        force(i) = xyz_thrust(i) - disturbance(i) + drag(i);
    else
        force(i) = xyz_thrust(i) - disturbance(i) - drag(i);
    end
    % account for gravity in the z direction
    if(i == 3)
        force(i) = force(i) - m*g;
    end
end

%f=ma to calculate final acceleration
for i=1:3
    linear_acceleration(i) = force(i)/m;
end