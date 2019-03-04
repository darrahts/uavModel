function angular_acceleration = angular_acceleration_calculation(thrust, torque)
%
% inputs: torque vector
%         thrust vector
%
% output: rotational acceleration (of drone)
%

% moments about each axis (x,y,z) i.e. (pitch, roll, yaw)
moments = [0 0 0];

% rotational acceleration about each axis
angular_acceleration = [0 0 0];

% moments of inertia empirically derived from sample data
moments_I = [.003, .003, .007];

% pitch
% motor pair A * 1/2 rigid body length - motor pair B * rigid body length
moments(1) = (thrust(3) + thrust(4))*.237/2 - (thrust(1)+thrust(2))*.237/2;
% roll
% motor pair A * 1/2 rigid body length - motor pair B * rigid body length
moments(2) = (thrust(3) + thrust(2))*.237/2 - (thrust(1)+thrust(4))*.237/2;
%yaw
% motor pair A - motor pair B
moments(3) = (torque(4) + torque(2)) - (torque(1)  + torque(3));

for i=1:3
    angular_acceleration(i) = moments(i) / moments_I(i);
end