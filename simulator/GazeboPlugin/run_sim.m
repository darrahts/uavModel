% running headless takes a few to load, so disp a start message


% sample time
s    = .01;

% stop time
t    = 4.3;

% to connect to gazebo, info only, set with dialog boxes
host = '98.193.213.40';
port = 11346;

% simulate the system
disp("opening system...");
open_system('gazebo_cosim_test');
set_param('gazebo_cosim_test', 'StopTime', num2str(t));
disp("Starting simulation...");
out = sim('gazebo_cosim_test');
disp("Finished!");
% get results
final_distance = out.z.Data(end);

% display result
res = sprintf("final distance: %.2f meters", final_distance);
disp(res);