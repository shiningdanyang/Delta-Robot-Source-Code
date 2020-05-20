function GrafSpace(A, n)
global R
hold on
% Îêðóæíîñòü âåðõíåãî îñíîâàíèÿ
t=0:pi/180:2*pi;
x=R*cos(t); 
y=R*sin(t); 
plot(x,y,'Linewidth',2);
%Ðàáî÷àÿ çîíà â âèäå îáëàêà òî÷åê
plot3(A(:,1), A(:,2), A(:,3),'.','color', [0.5 0 0.5], 'MarkerSize',3)
grid on
rotate3d on
axis equal 
hold off           