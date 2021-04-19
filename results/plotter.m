data=csvread('data.csv');
t=data(:,1);
x=data(:,2);
y=data(:,3);
z=data(:,4);
q1=data(:,5);
q2=data(:,6);
q3=data(:,7);
q4=data(:,8);
xd=data(:,9);
yd=data(:,10);
zd=data(:,11);
wx=data(:,12);
wy=data(:,13);
wz=data(:,14);

plot3(x,y,z)