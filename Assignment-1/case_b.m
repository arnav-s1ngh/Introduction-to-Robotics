% Robotics Assignment-1

% System Parameters
m1=1; 
m2=1;  
a1=1; 
a2=1; 
g=10;          
k1=1; 
k2=1;  
learning_rate=200000; % a value above 90k gives a decent overall result

%Duration
start=0;
stop=10;
step=0.01;
t=start:step:stop;

% Trajectory
x0=[0,0,0,0];
function [qd,vd]=desired_trajectory(t)
    qd=[pi/3;pi/4];
    vd=[0;0];
    %qd=[pi/4*(sin(t));pi/5*cost(t)];
    %vd=[pi/4*(cos(t));pi/5*(-1)*(sin(t))]
end

% Adaptive PD Controller
function [tau,k1, k2]=adaptive_pd_controller(q,v,qd,vd,k1,k2,learning_rate)
    e=q-qd;
    e_v=v-vd;
    k1=k1+learning_rate*(e'*e);
    k2=k2+learning_rate*(e_v'*e_v);
    tau=-k1*e-k2*e_v;
end

% System motion
function dyn=motion(t,x,m1,m2,a1,a2,g,k1,k2,learning_rate)
    q=x(1:2);
    v=x(3:4);
    [qd,vd]=desired_trajectory(t);
    [tau,~,~]=adaptive_pd_controller(q,v,qd,vd,k1,k2,learning_rate);
    M=[(m1+m2)*a1^2+m2*a2^2+2*m2*a1*a2*cos(q(2)),m2*a2^2+m2*a1*a2*cos(q(2));m2*a2^2+m2*a1*a2*cos(q(2)),m2*a2^2];
    V=[-m2*a1*a2*sin(q(2))*(2*v(1)*v(2)+v(2)^2);m2*a1*a2*v(1)^2*sin(q(2))];
    G=[(m1+m2)*g*a1*cos(q(1))+m2*g*a2*cos(q(1)+q(2));m2*g*a2*cos(q(1)+q(2))];
    a=M\(tau-V-G);
    dyn=[v;a];
end

% DE Solver
[t,X]=ode45(@(t,x) motion(t,x,m1,m2,a1,a2,g,k1,k2,learning_rate),t,x0);
theta1=X(:,1);
theta2=X(:,2);

% Calculate desired trajectory for plotting
qd_traj=zeros(length(t),2);
vd_traj=zeros(length(t),2);
for i = 1:length(t)
    [qd_temp, vd_temp] = desired_trajectory(t(i));
    qd_traj(i,:) = qd_temp';
    vd_traj(i,:) = vd_temp';
end

% Create animation
figure('Position', [100 100 1200 600]);

% Store end-effector trajectory
ee_x = zeros(length(t), 1);
ee_y = zeros(length(t), 1);
ee_x_desired = zeros(length(t), 1);
ee_y_desired = zeros(length(t), 1);

% Forward Kinematics
function [x,y]=fk(theta1, theta2, a1, a2)
    x1=a1*cos(theta1);
    y1=a1*sin(theta1);
    x2=x1+a2*cos(theta1+theta2);
    y2=y1+a2*sin(theta1+theta2);
    x=[0,x1,x2];
    y=[0,y1,y2];
end

% Animation
for i = 1:10:length(t)
    % Calculate current positions
    [x, y] = fk(theta1(i), theta2(i), a1, a2);
    [xd, yd] = fk(qd_traj(i,1), qd_traj(i,2), a1, a2);
    
    % Store end-effector positions
    ee_x(i) = x(3);
    ee_y(i) = y(3);
    ee_x_desired(i) = xd(3);
    ee_y_desired(i) = yd(3);
    
    % Left subplot - Robot arm visualization
    subplot(1,3,1);
    plot(x, y, 'b-', 'LineWidth', 2); hold on;
    plot(x, y, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
    plot(xd, yd, 'r--', 'LineWidth', 1);
    plot(xd, yd, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
    % Plot trajectory traces
    plot(ee_x(1:i), ee_y(1:i), 'b:', 'LineWidth', 1);
    plot(ee_x_desired(1:i), ee_y_desired(1:i), 'r:', 'LineWidth', 1);
    
    grid on;
    axis equal;
    axis([-2.5 2.5 -2.5 2.5]);
    title('2-DOF Robot Arm Motion');
    xlabel('X Position');
    ylabel('Y Position');
    legend('Current Position', 'Joints', 'Desired Position', 'Desired Joints', ...
           'Actual Trajectory', 'Desired Trajectory', 'Location', 'best');
    hold off;
    
    % Middle subplot - Joint 1 angle
    subplot(1,3,2);
    plot(t(1:i), theta1(1:i), 'b-', t(1:i), qd_traj(1:i,1), 'r--');
    grid on;
    title('Joint 1 Angle');
    xlabel('Time (s)');
    ylabel('Angle (rad)');
    legend('Actual \theta_1', 'Desired \theta_1');
    
    % Right subplot - Joint 2 angle
    subplot(1,3,3);
    plot(t(1:i), theta2(1:i), 'b-', t(1:i), qd_traj(1:i,2), 'r--');
    grid on;
    title('Joint 2 Angle');
    xlabel('Time (s)');
    ylabel('Angle (rad)');
    legend('Actual \theta_2', 'Desired \theta_2');
    
    drawnow;
end

% Calculate and display tracking errors
mean_error_1 = mean(abs(theta1 - qd_traj(:,1)));
mean_error_2 = mean(abs(theta2 - qd_traj(:,2)));
disp('Mean tracking error for joint 1: %.4f radians\t');
disp(mean_error_1);
disp('Mean tracking error for joint 2: %.4f radians\t')
disp(mean_error_2);

figure('Position', [100 100 1200 800]);

% Calculate errors
theta_error = [theta1 - qd_traj(:,1), theta2 - qd_traj(:,2)];

% Calculate velocity errors
theta_dot_actual = X(:,3:4);  % Extract actual velocities from state vector
theta_dot_error = [theta_dot_actual(:,1) - vd_traj(:,1), ...
                  theta_dot_actual(:,2) - vd_traj(:,2)];

% Calculate torques and torque errors
torque_actual = zeros(length(t), 2);
torque_desired = zeros(length(t), 2);
for i = 1:length(t)
    q = X(i,1:2)';
    v = X(i,3:4)';
    [qd_temp, vd_temp] = desired_trajectory(t(i));
    
    % Calculate actual torque
    [tau, k1_adapted, k2_adapted] = adaptive_pd_controller(q, v, qd_temp, vd_temp, k1, k2, learning_rate);
    torque_actual(i,:) = tau';
    
    % Calculate desired torque (using desired trajectory)
    [tau_d, ~, ~] = adaptive_pd_controller(qd_temp, vd_temp, qd_temp, vd_temp, k1, k2, learning_rate);
    torque_desired(i,:) = tau_d';
end
torque_error = torque_actual - torque_desired;

% Position error plots
subplot(3,2,1);
plot(t, theta_error(:,1), 'b-', 'LineWidth', 1);
grid on;
title('Joint 1 Position Error');
xlabel('Time (s)');
ylabel('\theta_1 Error (rad)');

subplot(3,2,2);
plot(t, theta_error(:,2), 'b-', 'LineWidth', 1);
grid on;
title('Joint 2 Position Error');
xlabel('Time (s)');
ylabel('\theta_2 Error (rad)');

% Velocity error plots
subplot(3,2,3);
plot(t, theta_dot_error(:,1), 'r-', 'LineWidth', 1);
grid on;
title('Joint 1 Velocity Error');
xlabel('Time (s)');
ylabel('\theta_1_{dot} Error (rad/s)');

subplot(3,2,4);
plot(t, theta_dot_error(:,2), 'r-', 'LineWidth', 1);
grid on;
title('Joint 2 Velocity Error');
xlabel('Time (s)');
ylabel('\theta_2_{dot} Error (rad/s)');

% Torque error plots
subplot(3,2,5);
plot(t, torque_error(:,1), 'g-', 'LineWidth', 1);
grid on;
title('Joint 1 Torque Error');
xlabel('Time (s)');
ylabel('\tau_1 Error (N⋅m)');

subplot(3,2,6);
plot(t, torque_error(:,2), 'g-', 'LineWidth', 1);
grid on;
title('Joint 2 Torque Error');
xlabel('Time (s)');
ylabel('\tau_2 Error (N⋅m)');
