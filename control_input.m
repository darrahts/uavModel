function motor_voltage = control_input(throttle, pitch, roll, yaw)
%
% outputs voltage per motor given control inputs
%
% inputs: throttle (double)
%         pitch (double)
%         roll (double)
%         yaw (double)
%
% output: voltage vector (m1 m2 m3 m4)
%

%battery voltage
BATT_V = 11.4;

bias = 0.0;

% pitch, roll, and yaw inputs, /2 due to drone symmetry w.r.t 4 motors
xyz_input = (pitch + roll + yaw )/2;

% difference between max throttle (100%) and current throttle
throttle_diff = 100 - throttle;

% throttle is most important, so excess is for pitch/roll/yaw
if(xyz_input > throttle_diff)
    bias = xyz_input - throttle_diff; 
end

% if the input is greater than throttle, set the bias to input-throttle
if(xyz_input > (throttle))
    if(bias < (xyz_input - throttle))
        bias = xyz_input - throttle;
    end
end

% set the bias for each motor and attitude value (pitch,roll,yaw)
bias = 2*bias/3;

%update the input commands 
if (bias ~= 0.0)
    pitch = pitch - bias;
    roll = roll - bias;
    yaw = yaw - bias;
end

motor_voltage = [(throttle - pitch/2 - roll/2 - yaw/2)*BATT_V/100
                 (throttle - pitch/2 + roll/2 + yaw/2)*BATT_V/100
                 (throttle + pitch/2 + roll/2 - yaw/2)*BATT_V/100
                 (throttle + pitch/2 - roll/2 + yaw/2)*BATT_V/100];








