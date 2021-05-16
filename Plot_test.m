clc
u=csvread('u');
pos=csvread('pos');
z=csvread('z');

m_actual=exp(z);

limx = 3000;
limy = 2500;
limz = 3500;

figure(20)
plot3(pos(1,:), pos(2,:), pos(3,:), 'linewidth',2)
hold on
quiver3(pos(1,:), pos(2,:), pos(3,:), u(1,:), u(2,:), u(3,:))
hold on
plot3(r(:,1),r(:,2),r(:,3),'Linewidth',2)
grid on

plot(pos(1,:), pos(2,:),'color','black')
plot3(limx*ones(1,length(pos)),pos(2,:),pos(3,:),'color','black')
plot3(pos(1,:),limy*ones(1,length(pos)),pos(3,:),'color','black')

xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')

xlim([-2500 0])
ylim([-100 700])
zlim([0 2000])

title('Out of Plane Divert Maneuver')

figure(21)
plot(m_actual)

grid on