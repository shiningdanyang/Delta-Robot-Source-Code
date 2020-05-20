x_trajectory = [0, 0];
y_trajectory = [-0.25, 0.25];	%trajectory along y-axis
z_trajectory = [-0.14, -0.14];	%h
trajectory = [x_trajectory;y_trajectory;z_trajectory];
sizeOfTraj = size(trajectory);
q1Array=[1,sizeOfTraj(2)];
q2Array=[1,sizeOfTraj(2)];
q3Array=[1,sizeOfTraj(2)];

for i=1:sizeOfTraj(2)
    [q1Array(i),q2Array(i), q3Array(i)] = inverseKinematic2(trajectory(1,i), trajectory(2,i), trajectory(3,i));	%solve IPK
    plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:));
    hold on;
    pause(1.5);	%time elapsed from start point to end point of trajectory
end
figure
plot(q1Array);	%servo1 angle value
hold on;
plot(q2Array);  %servo2 angle value
hold on;
plot(q3Array);  %servo3 angle value
hold on;