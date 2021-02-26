%Import python and weBots Data
%output = [x, y, theta, wc(ground_truth), df, dr, wC(gyro), Hx, Hy]

%Pytont Names:
% x = x_pos_py
% y = y_pos_py
% theta = theta_py
% wC = wC_py_truth
% df = df_py
% dr = dr_py
% wC = wC_py_gyro
% Hx = Hx_py
% Hy = Hy_py

%WeBots Names:
% x = x_pos 
% y = y_pos
% theta = theta
% wC = wC_truth
% df = lidar_F_data
% dr = lidar_R_data
% wC = wC_gyro
% Hx = Hx
% Hy = Hy

clear all; close all;
a_sim=readmatrix('paperBot3_outputs.csv'); %analytical simulation data, file generated from python
p_sim=readmatrix('paperBot3_webots.csv'); %physics simulation data, file generated from webots

% Time vector
t=p_sim(:,1)'; 

% python position data
a_trajectory=a_sim(:,[1:2])/1000;
x_pos_py = a_trajectory(:,1);
y_pos_py = a_trajectory(:,2);

% we bots position data
p_trajectory=p_sim(:,[2,3])+0.5;
x_pos = p_trajectory(:,1);
y_pos = p_trajectory(:,2);

% python angular position data (supervisor)
theta_py = a_sim(:,3);

% we bots angular position data (supervisor)
theta = p_sim(:,4)+pi;

% python angular velocity data (supervisor)
wC_py_truth = a_sim(:,4);

% we bots angular velocity data (supervisor)
wC_truth = p_sim(:,5);


% python Lidar, Gyro, Compass (with noise)
df_py = a_sim(:,5)/1000;
dr_py = a_sim(:,6)/1000;
wC_py_gyro = a_sim(:,7);
Hx_py = a_sim(:,8);
Hy_py = a_sim(:,9);
% we bots Lidar, Gyro, Compass (sensors)
lidar_F_data = p_sim(:,6);
lidar_R_data = p_sim(:,7);
wC_gyro = p_sim(:,8);
Hx = p_sim(:,9);
Hy = p_sim(:,10);

% Scale we-bots 
lidar_F_data = lidar_F_data*1/4096;
lidar_R_data = lidar_R_data*1/4096;
Hx = -1*Hx;
Hy = -1*Hy;


%Comparing Position
figure;
plot(x_pos,y_pos,'LineWidth',2); 
grid on;
hold on;
plot(x_pos_py,y_pos_py,'--','LineWidth',2);
title('Robot Position');
xlabel('x (m)') 
ylabel('y (m)') 
legend({'WeBots','Python Sim'})

%Position error plot x values
x_error = error(x_pos_py,x_pos);
figure;
grid on;
plot(t,x_error);
title('Error: X - position');
xlabel('time (s)');
ylabel('% Error');
ylim([-1 100]);
x_rms = sqrt(immse(x_pos_py,x_pos));
annotation('textbox',[.2 .5 .3 .3],'String',...
sprintf('Root Mean Square Error %f', x_rms),'FitBoxToText','on');

%Position error plot y values
y_error = error(y_pos_py,y_pos);
figure;
grid on;
plot(t,y_error);
title('Error: Y - position');
xlabel('time (s)');
ylabel('% Error');
ylim([-1 100]);
y_rms = sqrt(immse(y_pos_py,y_pos));
annotation('textbox',[.2 .5 .3 .3],'String',...
sprintf('Root Mean Square Error %f', y_rms),'FitBoxToText','on');

%Comparing Absolute Angle
figure;
plot(t,theta)
hold on;
grid on;
plot(t,theta_py);
title('Robot Angle');
xlabel('time (s)') 
ylabel('Angle (rad)') 
legend({'WeBots','Python Sim'})

%Angle error plot
theta_error = error(theta_py,theta);
figure;
grid on;
plot(t,theta_error);
title('Error: Angle');
xlabel('time (s)');
ylabel('% Error');
ylim([-1 100]);
theta_rms = sqrt(immse(theta_py,theta));
annotation('textbox',[.2 .5 .3 .3],'String',...
sprintf('Root Mean Square Error %f', theta_rms),'FitBoxToText','on');
%Comparing Angular velocity from Ground Truth
figure;
plot(t,wC_truth,'.');
hold on;
grid on;
plot(t,wC_py_truth,'.');
hold off;
title('Robot Angular Velocity True Values');
xlabel('time(s)');
ylabel('Velocity (rad/sec)');
legend({'WeBots Ground Truth','Python Sim Truth'});


%Angular Velocity error plot from Ground Truth
wC_truth_error = error(wC_py_truth,wC_truth);
figure;
grid on;
plot(t,wC_truth_error);
title('Error: Angular Velocity (True Values)');
xlabel('time (s)');
ylabel('% Error');
ylim([0 100]);
wC_truth_rms = sqrt(immse(wC_py_truth,wC_truth));
annotation('textbox',[.2 .5 .3 .3],'String',...
sprintf('Root Mean Square Error %f', wC_truth_rms),'FitBoxToText','on');

%Comparing Front Lidar
figure;
plot(t,lidar_F_data,'.'); %Convert to m if not already converted
grid on;
hold on;
plot(t,df_py,'.'); %This is converted to m, remove /1000 if already in m
title('Front Lidar Sensor');
xlabel('time (s)') 
ylabel('Distance (m)') 
legend({'WeBots','Python Sim'})

%Front Lidar error plot
lidar_F_error = error(df_py,lidar_F_data);
figure;
grid on;
plot(t,lidar_F_error);
title('Error: Front Lidar Sensor');
xlabel('time (s)');
ylabel('% Error');
ylim([0 100]);
df_rms = sqrt(immse(df_py,lidar_F_data));
annotation('textbox',[.2 .5 .3 .3],'String',...
sprintf('Root Mean Square Error %f', df_rms),'FitBoxToText','on');

%Comparing Right Lidar
figure;
plot(t,lidar_R_data,'.');
grid on;
hold on;
plot(t,dr_py,'.'); 
title('Right Lidar Sensor');
xlabel('time (s)') 
ylabel('Distance (m)') 
legend({'WeBots','Python Sim'})

%Right Lidar error plot
lidar_R_error = error(dr_py,lidar_R_data);
figure;
grid on;
plot(t,lidar_R_error);
title('Error: Right Lidar Sensor');
xlabel('time (s)');
ylabel('% Error');
ylim([0 100]);
dr_rms = sqrt(immse(dr_py,lidar_R_data));
annotation('textbox',[.2 .5 .3 .3],'String',...
sprintf('Root Mean Square Error %f', dr_rms),'FitBoxToText','on');

%Comparing Angular velocity from Gyro
figure;
plot(t,wC_gyro,'.');
hold on;
grid on;
plot(t,wC_py_gyro, '.');
hold off;
title('Robot Angular Velocity Gyro Values');
xlabel('time(s)');
ylabel('Velocity (rad/sec)');
legend({'WeBots Gyro','Python Sim Gyro'});

%Angular Velocity error plot from Gyro
figure;
wC_gyro_error = error(wC_py_gyro,wC_gyro);
grid on;
plot(t,wC_gyro_error);
title('Error: Angular Velocity (Gyro Values)');
xlabel('time (s)');
ylabel('% Error');
ylim([0 100]);
wC_gyro_rms = sqrt(immse(wC_py_gyro,wC_gyro));
annotation('textbox',[.2 .5 .3 .3],'String',...
sprintf('Root Mean Square Error %f', wC_gyro_rms),'FitBoxToText','on');

%compare x compass reading
figure;
plot(t,Hx)
hold on;
grid on;
plot(t,Hx_py);
title('Comapass x component');
xlabel('time (s)') 
ylabel('Hx') 
legend({'WeBots','Python Sim'})

%Hx compass error plot
Hx_error = error(Hx_py,Hx);
figure;
grid on;
plot(t,Hx_error);
title('Error: Magnetic x-axis');
xlabel('time (s)');
ylabel('% Error');
ylim([0 100]);
Hx_rms = sqrt(immse(Hx_py,Hx));
annotation('textbox',[.2 .5 .3 .3],'String',...
sprintf('Root Mean Square Error %f', Hx_rms),'FitBoxToText','on');

%compare y compass reading
figure;
plot(t,Hy)
hold on;
grid on;
plot(t,Hy_py);
title('Comapass y component');
xlabel('time (s)') 
ylabel('Hy') 
legend({'WeBots','Python Sim'})

%Hy compass error plot
Hy_error = error(Hy_py,Hy);
figure;
grid on;
plot(t,Hy_error);
title('Error: Magnetic y-axis');
xlabel('time (s)');
ylabel('% Error');
ylim([0 100]);
Hy_rms = sqrt(immse(Hy_py,Hy));
annotation('textbox',[.2 .5 .3 .3],'String',...
sprintf('Root Mean Square Error %f', Hy_rms),'FitBoxToText','on');
% compute error
function out = error(python, webot)
    out= abs(webot-python)./((abs(webot+python))/2)*100;
end

function o = immse(python, webots)
    o = sum((python-webots).^2)/length(python);
end