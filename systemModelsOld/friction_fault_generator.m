function Tf = friction_fault_generator(motor_select, enabled, t)

Tf = [1e-8,1e-8,1e-8,1e-8];

if enabled == 1
    Tf(motor_select) = Tf(motor_select) + abs(normrnd(0, 1e-9 * t));
end
