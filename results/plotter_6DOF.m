close all;clc;
data=csvread('data.csv');
t=data(:,1);
r=data(:,2:4);
q=data(:,5:8);
v=data(:,9:11);
w=data(:,12:14);
m=data(:,15);

%3D Trajectory
figure(1)
plot3(r(:,1),r(:,2),r(:,3),'Linewidth',2)
xlabel('X Distance (m)','interpreter','latex')
ylabel('Y Distance (m)','interpreter','latex')
zlabel('Z Distance (m)','interpreter','latex')
title('3D Vehicle Trajectory','interpreter','latex')
axis equal
grid on

%Position vs Time
figure(2)
sgtitle('Position vs Time','interpreter','latex')
subplot(3,1,1)
plot(t,r(:,1),'Linewidth',2)
title('$x$','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
grid on

subplot(3,1,2)
plot(t,r(:,2),'Linewidth',2)
title('$y$','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
grid on

subplot(3,1,3)
plot(t,r(:,3),'Linewidth',2)
title('$z$','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
grid on

%Velocity vs Time
figure(3)
sgtitle('Velocity vs Time','interpreter','latex')
subplot(3,1,1)
plot(t,v(:,1),'Linewidth',2)
title('$v_x$','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
grid on

subplot(3,1,2)
plot(t,v(:,2),'Linewidth',2)
title('$v_y$','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
grid on

subplot(3,1,3)
plot(t,v(:,3),'Linewidth',2)
title('$v_z$','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
grid on


%Orientation over Time
figure(4)
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

%Angular rate vs Time
figure(5)
sgtitle('Angular rate vs Time','interpreter','latex')
subplot(3,1,1)
plot(t,w(:,1),'Linewidth',2)
title('$w_x$','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
grid on

subplot(3,1,2)
plot(t,w(:,2),'Linewidth',2)
title('$w_y$','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
grid on

subplot(3,1,3)
plot(t,w(:,3),'Linewidth',2)
title('$w_z$','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
grid on

%Mass vs Time
figure(6)
plot(t,m,'Linewidth',2)
title('Total Vehicle Mass Over Time','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
ylabel('Mass (kg)', 'interpreter', 'latex')
grid on