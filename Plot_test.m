close all
u=csvread('u');
pos=csvread('pos');

limx = 3000;
limy = 2500;
limz = 3500;

figure(3)
plot3(pos(1,:), pos(2,:), pos(3,:), 'linewidth',2)
hold on
quiver3(pos(1,:), pos(2,:), pos(3,:), u(1,:), u(2,:), u(3,:))

plot(pos(1,:), pos(2,:),'color','black')
plot3(limx*ones(1,length(pos)),pos(2,:),pos(3,:),'color','black')
plot3(pos(1,:),limy*ones(1,length(pos)),pos(3,:),'color','black')

xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')

title('Out of Plane Divert Maneuver')

%xlim([0 limx])
%ylim([0 limy])
%zlim([0 limz])
grid on