function [thrust, torque, current] = fcn(voltage)
%
% input: voltage
%
% output: motor torque, 
%         propeller thrust 
%         battery current
%

% rpm constant
KV = 1400;
% per motor
rpm = [0 0 0 0]; 
thrust = [0 0 0 0];
torque = [0 0 0 0];
current = [0 0 0 0];

for i=1:4
    % empirically derived equation given sample data
    rpm(i) = -2.6931*voltage(i)^3 + KV*voltage(i);
    %coefficient of thrust, empirically derived
    coef_t = 2*(10^-15)*rpm(i)^3 - 4*(10^-11)*rpm(i)^2 + 3*(10^-7)*rpm(i) + .1013;
    %thrust = coef_t * density of air * velocity^2 * propeller diameter (m) ^4
    thrust(i) = coef_t*1.225*(rpm(i)/60)^2*.2^4;
    % empirically derived
    torque(i) = 4*(10^-14)*rpm(i)^3 +  8*(10^-12)*rpm(i)^2 + 3*(10^-6)*rpm(i);
    % current = rpm constant * torque
    current(i) = KV*torque(i);
end