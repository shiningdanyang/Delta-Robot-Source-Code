numberOfPoint = 20;

figure
startPoint = [0,0,-0.9];
endPoint = [0.3,0.3,-1.4];
x_trajectory = linspace(startPoint(1), endPoint(1), numberOfPoint);
y_trajectory = linspace(startPoint(2), endPoint(2), numberOfPoint);
z_trajectory = linspace(startPoint(3), endPoint(3), numberOfPoint);
trajectory = [x_trajectory; y_trajectory; z_trajectory];

vector = [startPoint;endPoint];
plot3(vector(:,1),vector(:,2),vector(:,3),'b');
hold on
for i = 1:numberOfPoint
    invertKinematic(trajectory(1,i),trajectory(2,i),trajectory(3,i));
    pause(0.1);
end

startPoint = endPoint;
endPoint = [-0.2, -0.2, -1.2];
x_trajectory = linspace(startPoint(1), endPoint(1), numberOfPoint);
y_trajectory = linspace(startPoint(2), endPoint(2), numberOfPoint);
z_trajectory = linspace(startPoint(3), endPoint(3), numberOfPoint);
trajectory = [x_trajectory; y_trajectory; z_trajectory];
for i = 1:numberOfPoint
    invertKinematic(trajectory(1,i),trajectory(2,i),trajectory(3,i));
    pause(0.1);
end
