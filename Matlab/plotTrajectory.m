numberOfPoint = 20;
x_center = 0;
y_center = 0;
r_Traj=0.05;

theta = 0:pi/50:2*pi;
x_trajectory = r_Traj*cos(theta) + x_center;
y_trajectory = r_Traj*sin(theta) + y_center;
z_trajectory = 0.15*cos(2*theta)-0.44;
trajectory = [x_trajectory; y_trajectory; z_trajectory];
sizeOfTraj = size(trajectory);
q1Array=[1,sizeOfTraj(2)];
q2Array=[1,sizeOfTraj(2)];
q3Array=[1,sizeOfTraj(2)];
    for i = 1:sizeOfTraj(2)
        [q1Array(i),q2Array(i), q3Array(i)]=inverseKinematic2(trajectory(1,i),trajectory(2,i),trajectory(3,i));
        plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:));
        hold on
        pause(0.001);
    end

figure
plot(q1Array);
hold on;
plot(q2Array);
hold on;
plot(q3Array);
hold on;


r = tcpip('192.168.0.1', 2000, 'NetworkRole', 'server');
fopen(r);
disp('opened, waiting for data from s7');
pause(1);

data2S7 = [0.0, 0.0, 0.0, 0.0];
fwrite(r, data2S7,'float64');
pause(0.01)
data2S7 = [0.01, 0.01, 0.01, 1.0];
fwrite(r, data2S7,'float64');

fread(r,1);
data2S7 = [0.0, 0.0, 0.0, 0.0];
fwrite(r, data2S7,'float64');
pause(0.01)
data2S7 = angleRoot(i,:);
fwrite(r, data2S7,'float64');