close all;clc;clear
data=csvread('data.csv');
t=data(:,1);
r=data(:,2:4);
q=data(:,5:8);
v=data(:,9:11);
w=data(:,12:14);
m=data(:,15);
T_des=data(:,16:18);
M_des=data(:,19:21);
throttle=data(:,22);
axis_TVC=data(:,23:25);
angle=data(:,26);
F_net=data(:,27:29);
M_net=data(:,30:32);
q_des=data(:,33:36);
q_norm=zeros(length(data),1);
for i=1:length(q)
    q_norm(i)=norm(q(i));
end


%3D Trajectory
figure(1)
plot3(r(:,1),r(:,2),r(:,3),'Linewidth',2)
xlabel('X Distance (m)','interpreter','latex')
ylabel('Y Distance (m)','interpreter','latex')
zlabel('Z Distance (m)','interpreter','latex')
title('3D Vehicle Trajectory','interpreter','latex')
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
sgtitle('Quaternion and Desired Quaternion Over Time','interpreter','latex')
subplot(4,1,1)
plot(t,q(:,1),'Linewidth',2)
hold on
plot(t,q_des(:,1),'Linewidth',2)
title('$q_1$','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
legend('Orientation Quaternion', 'Desired Orientation Quaternion')
grid on

subplot(4,1,2)
plot(t,q(:,2),'Linewidth',2)
hold on
plot(t,q_des(:,2),'Linewidth',2)
title('$q_2$','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
legend('Orientation Quaternion', 'Desired Orientation Quaternion')
grid on

subplot(4,1,3)
plot(t,q(:,3),'Linewidth',2)
hold on
plot(t,q_des(:,3),'Linewidth',2)
title('$q_3$','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
legend('Orientation Quaternion', 'Desired Orientation Quaternion')
grid on

subplot(4,1,4)
plot(t,q(:,4),'Linewidth',2)
hold on
plot(t,q_des(:,4),'Linewidth',2)
title('$q_4$','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
legend('Orientation Quaternion', 'Desired Orientation Quaternion')
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

%Moment and Desired Moment vs Time
figure(8)
sgtitle('Desired Moment vs Time','interpreter','latex')
subplot(3,1,1)
plot(t,M_des(:,1),'Linewidth',2)
hold on
plot(t,M_net(:,1),'Linewidth',2)
title('$M_x$','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
ylabel('Moment (Nm)','interpreter','latex')
legend('Net Moment','Desired Moment')
grid on

subplot(3,1,2)
plot(t,M_des(:,2),'Linewidth',2)
hold on
plot(t,M_net(:,2),'Linewidth',2)
title('$M_y$','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
ylabel('Moment (Nm)','interpreter','latex')
legend('Net Moment','Desired Moment')
grid on

subplot(3,1,3)
plot(t,M_des(:,3),'Linewidth',2)
hold on
plot(t,M_net(:,3),'Linewidth',2)
title('$M_z$','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
ylabel('Moment (Nm)','interpreter','latex')
legend('Net Moment','Desired Moment')
grid on

%Throttle vs Time
figure(9)
plot(t,throttle,'Linewidth',2)
title('Thrust Magnitude Over Time','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
ylabel('Thrust (N)', 'interpreter', 'latex')
grid on

%TVC Angle vs Time
figure(10)
plot(t,angle * (180/pi),'Linewidth',2)
title('TVC Angle Over Time','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
ylabel('Angle (degrees)', 'interpreter', 'latex')
grid on

%Net Force vs Time
figure(11)
sgtitle('Net Force vs Time','interpreter','latex')
subplot(3,1,1)
plot(t,F_net(:,1),'Linewidth',2)
hold on
plot(t,T_des(:,1),'Linewidth',2)
title('$F_x$','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
ylabel('Force (N)','interpreter','latex')
grid on

subplot(3,1,2)
plot(t,F_net(:,2),'Linewidth',2)
hold on
plot(t,T_des(:,2),'Linewidth',2)
title('$F_y$','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
ylabel('Force (N)','interpreter','latex')
grid on

subplot(3,1,3)
plot(t,F_net(:,3),'Linewidth',2)
title('$F_z$','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
ylabel('Force (N)','interpreter','latex')
grid on

figure(12)
plot(t,q_norm,'Linewidth',2)

