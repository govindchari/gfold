clear;close all;clc;
sim=csvread('sim_data.csv');
fc=csvread('fc_data.csv');
g = 9.807;
t=sim(:,1);
ts=fc(:,1);
r=sim(:,2:4);
q=sim(:,5:8);
v=sim(:,9:11);
w=sim(:,12:14);
m=sim(:,15);
T_des=fc(:,2:4);
M_des=fc(:,5:7);
throttle=fc(:,8);
axis_TVC=fc(:,9:11);
angle=fc(:,12);
F_net=sim(:,16:18);
M_net=sim(:,19:21);
q_des=fc(:,13:16);
q_norm=zeros(length(sim),1);
h=sim(1,1);



for i=1:length(q)
    q_norm(i)=norm(q(i,:));
    tilt(i)=2*acosd(q(i,1));
end

for i=1:length(F_net)
    if (mod(i,1/h)==0)
        j=i*h;
        r_q(:,j) = r(i,:);
        T(1,j) = F_net(i,1);
        T(2,j) = F_net(i,2);
        T(3,j) = F_net(i,3)+m(i)*g;
    end
end
%3D Trajectory
figure(1)
plot3(r(:,1),r(:,2),r(:,3),'Linewidth',2)
hold on
quiver3(r_q(1,:), r_q(2,:), r_q(3,:), T(1,:), T(2,:), T(3,:), 0.7)
plot3(r(1,1),r(1,2),r(1,3),'g*')
plot3(r(length(r),1),r(length(r),2),r(length(r),3),'r*')
xmin = -2500;
xmax = 0;
ymin = 0;
ymax = 700;
zmin = 0;
zmax = 2500;
%xlim([xmin xmax])
%ylim([ymin ymax])
%zlim([zmin zmax])
hold on
plot(r(:,1), r(:,2),'color','black')
%plot3(xmax*ones(1,length(r)),r(:,2),r(:,3),'color','black')
%plot3(r(:,1),ymax*ones(1,length(r)),r(:,3),'color','black')
xlabel('X Distance (m)','interpreter','latex')
ylabel('Y Distance (m)','interpreter','latex')
zlabel('Z Distance (m)','interpreter','latex')
title('3D Vehicle Trajectory','interpreter','latex')
grid on
axis equal

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
title('$q_1$','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
grid on

subplot(4,1,2)
plot(t,q(:,2),'Linewidth',2)
hold on
plot(ts,q_des(:,2),'Linewidth',2)
title('$q_2$','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
legend('Orientation Quaternion', 'Desired Orientation Quaternion','location','northwest')
grid on

subplot(4,1,3)
plot(t,q(:,3),'Linewidth',2)
hold on
plot(ts,q_des(:,3),'Linewidth',2)
title('$q_3$','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
legend('Orientation Quaternion', 'Desired Orientation Quaternion','location','northwest')
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

%Moment and Desired Moment vs Time
figure(8)
sgtitle('Desired Moment vs Time','interpreter','latex')
subplot(2,1,1)
plot(t,M_net(:,1),'Linewidth',2)
hold on
plot(ts,M_des(:,1),'Linewidth',2)
title('$M_x$','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
ylabel('Moment (Nm)','interpreter','latex')
legend('Net Moment','Desired Moment')
grid on

subplot(2,1,2)
plot(t,M_net(:,2),'Linewidth',2)
hold on
plot(ts,M_des(:,2),'Linewidth',2)
title('$M_y$','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
ylabel('Moment (Nm)','interpreter','latex')
legend('Net Moment','Desired Moment')
grid on

%Throttle vs Time
figure(9)
plot(ts,throttle,'Linewidth',2)
title('Thrust Magnitude Over Time','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
ylabel('Thrust (N)', 'interpreter', 'latex')
grid on

%TVC Angle vs Time
figure(10)
plot(ts,angle * (180/pi),'Linewidth',2)
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
plot(ts,T_des(:,1),'Linewidth',2)
title('$F_x$','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
ylabel('Force (N)','interpreter','latex')
legend('Net Force','Desired Force')

grid on

subplot(3,1,2)
plot(t,F_net(:,2),'Linewidth',2)
hold on
plot(ts,T_des(:,2),'Linewidth',2)
title('$F_y$','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
ylabel('Force (N)','interpreter','latex')
legend('Net Force','Desired Force')

grid on

subplot(3,1,3)
plot(t,F_net(:,3),'Linewidth',2)
hold on
plot(ts,T_des(:,3),'Linewidth',2)
title('$F_z$','interpreter','latex')
xlabel('Time (s)','interpreter','latex')
ylabel('Force (N)','interpreter','latex')
legend('Net Force','Desired Force')
grid on

figure(12)
plot(t,q_norm,'Linewidth',2)
title('Quaternion Norm')
xlabel('Time (s)','interpreter','latex')
ylabel('Norm','interpreter','latex')
grid on


