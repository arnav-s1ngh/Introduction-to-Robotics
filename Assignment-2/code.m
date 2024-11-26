% Robotics Assignment-2

% Initial Parameters
x_i=0;
y_i=0;
theta_i=0;
v=2;
r=1;
t=15;
dt=0.01;
d=0.01; %look ahead distance
kv=1;
ki=1;

% Desired Trajectory
theta_f=(v/r)*tim;
x_f=r*cos(theta_f);
y_f=r*sin(theta_f);


for i= 0:dt:t
    x_error=x_f-x_i;
    y_error=y_f-y_i;
    net_error=sqrt(x_error^2+y_error^2)-d;
    v_i=kv*net_error

end
    


