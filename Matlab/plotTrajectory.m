numberOfPoint = 20;
x_center = 0;
y_center = 0;
r_Traj=0.05;

theta = 0:pi/50:2*pi;
x_trajectory = r_Traj*cos(theta) + x_center;
y_trajectory = r_Traj*sin(theta) + y_center;
z_trajectory = 0.15*cos(2*theta)-0.34;
trajectory = [x_trajectory; y_trajectory; z_trajectory];
sizeOfTraj = size(trajectory);
q1Array=[1,sizeOfTraj(2)];
q2Array=[1,sizeOfTraj(2)];
q3Array=[1,sizeOfTraj(2)];
% while 1
    for i = 1:sizeOfTraj(2)
%         inverseKinematic(trajectory(1,i),trajectory(2,i),trajectory(3,i));
        [q1Array(i),q2Array(i), q3Array(i)]=inverseKinematic2(trajectory(1,i),trajectory(2,i),trajectory(3,i));
        plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:));
        hold on
        pause(0.1);
    end
% end

figure
plot(q1Array);
hold on;
plot(q2Array);
hold on;
plot(q3Array);
hold on;

% startPoint = [0,0,-0.9];
% endPoint = [0.3,0.3,-1.4];
% x_trajectory = linspace(startPoint(1), endPoint(1), numberOfPoint);
% y_trajectory = linspace(startPoint(2), endPoint(2), numberOfPoint);
% z_trajectory = linspace(startPoint(3), endPoint(3), numberOfPoint);
% trajectory = [x_trajectory; y_trajectory; z_trajectory];
% 
% vector = [startPoint;endPoint];
% for i = 1:numberOfPoint
%     invertKinematic(trajectory(1,i),trajectory(2,i),trajectory(3,i));
%     plot3(vector(:,1),vector(:,2),vector(:,3),'b');
%     hold on
%     pause(0.1);
% end
% 
% startPoint = endPoint;
% endPoint = [-0.2, -0.2, -1.2];
% x_trajectory = linspace(startPoint(1), endPoint(1), numberOfPoint);
% y_trajectory = linspace(startPoint(2), endPoint(2), numberOfPoint);
% z_trajectory = linspace(startPoint(3), endPoint(3), numberOfPoint);
% trajectory = [x_trajectory; y_trajectory; z_trajectory];
% 
% vector = [startPoint;endPoint];
% for i = 1:numberOfPoint
%     invertKinematic(trajectory(1,i),trajectory(2,i),trajectory(3,i));
%     plot3(vector(:,1),vector(:,2),vector(:,3),'b');
%     hold on
%     pause(0.1);
% end
