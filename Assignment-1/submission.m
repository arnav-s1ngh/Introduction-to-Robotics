% robotics: assignment-1
close all;
clear;
clc;
% params
m1=1;  
m2=1;  
a1=1;  
a2=1;  
time_span=[0,10];
g=10;

% initial conditions
theta_i=[0;0];
vel_i=[0;0];
state_i=[theta_i;vel_i];

% case a: set point regulation
a_theta_f=@(t)[pi/3;pi/4];
a_vel_f=@(t)[0;0];

% case b: sinusoidal tracking
b_theta_f=@(t)[(pi/4)*sin(t);(pi/5)*cos(t)];
b_vel_f=@(t)[(pi/4)*cos(t);(-1)*(pi/5)*sin(t)];

% desired trajectory
disp("Select trajectory");
disp("1. Set Point Regulation");
disp("2. Sinusoidal Tracking");
response=input("Trajectory:- ");
vel_f=0;
theta_f=0;
if (response==1)
    theta_f=a_theta_f;
    vel_f=a_vel_f;
    clear;
elseif (response==2)
    theta_f=b_theta_f;
    vel_f=b_vel_f;
    clear;
else
    quit(0,"force");
end

params=@(t,state) magk(t,state,a1,a2,m1,m2,k1,k2,g);
[t,state]=ode45(params,vel_f,theta_i);

