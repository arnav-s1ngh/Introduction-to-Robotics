% Pure Pursuit Controller for Unicycle Robot
% References:- https://arxiv.org/html/2409.09848v1#:~:text=The%20pure%20pursuit%20method%20stands,and%20the%20vehicle's%20reference%20point.
% References:- https://in.mathworks.com/matlabcentral/answers/1696610-pure-pursuit-controller-should-follow-y-x-5
close all;
clear;
clc;
% Initial Pose
x=0;             
y=0;            
theta=0;     
dt=0.0005;       
T=15;            
v=2;
% Parameters
ld=0.008;
radius=1;      
dur=0:dt:T;
s=size(dur);
xpath=zeros(s);
ypath=zeros(s);
thetapath=zeros(s);
% Radial Distance
errorpath=zeros(s);

for i=1:length(dur)
    xpath(i)=x;
    ypath(i)=y;
    thetapath(i)=rem(theta,2*pi);
    errorpath(i)=abs(sqrt(x^2+y^2)-radius);
    current_angle=atan2(y,x);
    ld_angle=current_angle+v*dt/radius;
    x_ahead=radius*cos(ld_angle);
    y_ahead=radius*sin(ld_angle);
    dx=x_ahead-x;
    dy=y_ahead-y;
    x=x+v*cos(theta)*dt;
    y=y+v*sin(theta)*dt;
    alpha=atan2(dy,dx)-theta;
    %formula for a unicycle with wheel radius=0.001
    phi=atan(2*(0.001/ld)*sin(alpha));
    theta=theta+phi; 
end


figure;
plot(dur,xpath,"b-");
hold on;
plot(dur,ypath,"g-")
title("X&Y-vs-t");
xlabel("T");
ylabel("X");

figure;
plot(dur,errorpath,"b-");
title("e-vs-t");
xlabel("T");
ylabel("Error");

figure;
plot(dur,xpath,"b-");
title("X-vs-t");
xlabel("T");
ylabel("X");

figure;
plot(dur,ypath,"b-");
title("Y-vs-t");
xlabel("T");
ylabel("Y");

figure;
plot(dur,thetapath,"b-");
title("θ-vs-t");
xlabel("T");
ylabel("θ");

figure;
plot(xpath,ypath,"b-","LineWidth",2);
hold on;
axis equal;
grid on;
plot(1,0,"ro");
plot(0,1,"ro");
plot(-1,0,"ro");
plot(0,-1,"ro");
title("Y-vs-X");
xlabel("X");
ylabel("Y");
