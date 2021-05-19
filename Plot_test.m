clc
u=csvread('u');
pos=csvread('pos');
z=csvread('z');
mc = csvread('MonteCarlo.csv');

m_actual=exp(z);

limx = 500;
limy = 1000;
limz = 3500;

for i=1:length(m_actual)
    T(:,i) = m_actual(i)*u(:,i);
    T_norm(i) = norm(T(:,1));
end


figure(20)
plot3(pos(1,:), pos(2,:), pos(3,:), 'linewidth',2)
hold on
quiver3(pos(1,:), pos(2,:), pos(3,:), u(1,:), u(2,:), u(3,:))
hold on
%plot3(r(:,1),r(:,2),r(:,3),'Linewidth',2)
grid on

plot(pos(1,:), pos(2,:),'color','black')
plot3(limx*ones(1,length(pos)),pos(2,:),pos(3,:),'color','black')
plot3(pos(1,:),limy*ones(1,length(pos)),pos(3,:),'color','black')

xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')

xlim([-600 500])
ylim([-500 1000])
zlim([0 3000])
axis equal

title('Out of Plane Divert Maneuver')
legend('Trajectory')

figure(21)
plot(m_actual)

figure(22)
scatter(mc(:,1),mc(:,2),'*')
axis equal
%xlim([-10 10])
%ylim([-10 10])
xlabel('x (m)')
ylabel('y (m)')

title('Monte-Carlo Landing Locations')
grid on