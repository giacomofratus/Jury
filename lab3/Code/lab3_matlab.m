TIME_STEP = wb_robot_get_basic_time_step;
% Get devices
motor_R = wb_robot_get_device('motor_R');
motor_L = wb_robot_get_device('motor_L');
gyro = wb_robot_get_device('gyro');
lidar_R = wb_robot_get_device('lidar_R');
lidar_F = wb_robot_get_device('lidar_F');
compass = wb_robot_get_device('compass');

% get node for Robot and Supervisor
robot = wb_supervisor_node_get_from_def('robot')
rotation = wb_supervisor_node_get_field(robot,'rotation')

% enable devices
wb_gyro_enable(gyro, TIME_STEP);
wb_distance_sensor_enable(lidar_R, TIME_STEP);
wb_distance_sensor_enable(lidar_F, TIME_STEP);
wb_compass_enable(compass, TIME_STEP);

disp(motor_R)
disp(motor_L)
% set initial postion for motor
wb_motor_set_position(motor_R, inf);
wb_motor_set_position(motor_L, inf);
% initialize output data
lidar_R_data = []
lidar_F_data = []
gyro_data = []
compass_data = []
position_data = []
velocity_data = []
angle_data = []
%counter
k=1
% read the input data
w=readmatrix('D:\EE 183DA\lab3_data\Segway1_inputs.csv');
wr=-w(:,1);
wl=-w(:,2);


while wb_robot_step(TIME_STEP) ~= -1 
  if k<=length(wr)
      % store data from sensor reading and ground truth
      lidar_F_data(k) = wb_distance_sensor_get_value(lidar_F);
      lidar_R_data(k) = wb_distance_sensor_get_value(lidar_R);
      gyro_data(:,k) = wb_gyro_get_values(gyro);
      compass_data(:,k) = wb_compass_get_values(compass);
      position_data(:,k) = wb_supervisor_node_get_position(robot);
      angle_data(:,k) = wb_supervisor_field_get_sf_rotation(rotation);
      velocity_data(:,k) = wb_supervisor_node_get_velocity(robot);
  else
      wb_console_print(sprintf('Simulation Done, pause and hit reset to get data'), WB_STDOUT)
  end
  if k<=length(wr)
    wb_motor_set_velocity(motor_R,wr(k));
    wb_motor_set_velocity(motor_L,wl(k));
  else
    % stop the robot with 0 velocity
    wb_motor_set_velocity(motor_R,0);
    wb_motor_set_velocity(motor_L,0);
  end
  k = k + 1;


end

% cleanup code goes here: write data to files, etc.
ts = TIME_STEP*10^-3;
time = 0:ts:ts*(length(wr)-1);
data = [time', position_data([3,1],:)',angle_data(4,:)',velocity_data(5,:)', lidar_F_data', lidar_R_data',gyro_data(2,:)', compass_data([1,3],:)'];
filename ='D:\EE 183DA\lab3_data\Segway1_webots';
name = strcat(filename,'.csv');
writematrix(data, name,'Delimiter','comma')
% function to get 6 values for supervisor_get_velocity
function result = wb_supervisor_node_get_velocity(noderef)
  obj = calllib('libController', 'wb_supervisor_node_get_velocity', noderef);
  setdatatype(obj,'doublePtr',1,6);
  result = get(obj,'Value');    
end
