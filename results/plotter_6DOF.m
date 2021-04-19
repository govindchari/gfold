data=csvread('data.csv');
t=data(:,1);
r=data(:,2:4);
q=data(:,5:8);
v=data(:,9:11);
w=data(:,12:14);

%Position
figure(1)
plot3(r(:,1),r(:,2),r(:,3),'Linewidth',2)
xlabel('X Distance (m)','interpreter','latex')
ylabel('Y Distance (m)','interpreter','latex')
zlabel('Z Distance (m)','interpreter','latex')
title('3D Vehicle Trajectory','interpreter','latex')
axis equal
grid on

%Orientation
figure(2)
sgtitle('Orientation Quaternion Over Time','interpreter','latex')
subplot(4,1,1)
plot(t,q(:,1),'Linewidth',2)
title('$q_1$','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
grid on

subplot(4,1,2)
plot(t,q(:,2),'Linewidth',2)
title('$q_2$','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
grid on

subplot(4,1,3)
plot(t,q(:,3),'Linewidth',2)
title('$q_3$','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
grid on

subplot(4,1,4)
plot(t,q(:,4),'Linewidth',2)
title('$q_4$','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
grid on