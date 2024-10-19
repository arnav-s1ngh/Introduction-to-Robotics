% 2-DOF Robot Arm Simulation with 2D Visualization - Sinusoidal Tracking (Case B)
clear all; close all; clc;

% System Parameters
m1 = 1; m2 = 1;  % masses
a1 = 1; a2 = 1;  % link lengths
g = 10;          % gravity

% Control gains (increased for better tracking)
k1 = 1; k2 = 1;  % Increased from 1 to improve tracking performance
adaptation_rate = 200000;

% Time settings
t_span = [0 10];
dt = 0.01;
t = t_span(1):dt:t_span(2);

% Initial conditions [θ1 θ2 θ1_dot θ2_dot]
x0 = [0 0 0 0];

% Mass Matrix M(q)
function M = mass_matrix(q, params)
    m1 = params.m1; m2 = params.m2;
    a1 = params.a1; a2 = params.a2;
    
    M = [(m1+m2)*a1^2 + m2*a2^2 + 2*m2*a1*a2*cos(q(2)), m2*a2^2 + m2*a1*a2*cos(q(2));
         m2*a2^2 + m2*a1*a2*cos(q(2)), m2*a2^2];
end

% Velocity terms V(q,q_dot)
function V = velocity_terms(q, q_dot, params)
    m2 = params.m2;
    a1 = params.a1; a2 = params.a2;
    
    V = [-m2*a1*a2*sin(q(2))*(2*q_dot(1)*q_dot(2) + q_dot(2)^2);
         m2*a1*a2*q_dot(1)^2*sin(q(2))];
end

% Gravity terms G(q)
function G = gravity_terms(q, params)
    m1 = params.m1; m2 = params.m2;
    a1 = params.a1; a2 = params.a2;
    g = params.g;
    
    G = [(m1+m2)*g*a1*cos(q(1)) + m2*g*a2*cos(q(1)+q(2));
         m2*g*a2*cos(q(1)+q(2))];
end

% Desired trajectory generator
function [qd, qd_dot] = desired_trajectory(t)
    qd = [pi/4*sin(t); pi/5*cos(t)];
    qd_dot = [pi/4*cos(t); -pi/5*sin(t)];
end

% Adaptive PD Controller
function [tau, k1_adapted, k2_adapted] = adaptive_pd_controller(q, q_dot, qd, qd_dot, k1, k2, adaptation_rate)
    e = q - qd;
    e_dot = q_dot - qd_dot;
    
    % Adapt gains based on tracking error
    k1_adapted = k1 + adaptation_rate * (e' * e);
    k2_adapted = k2 + adaptation_rate * (e_dot' * e_dot);
    
    tau = -k1_adapted*e - k2_adapted*e_dot;
end

% System dynamics
function dx = dynamics(t, x, params, k1, k2, adaptation_rate)
    q = x(1:2);
    q_dot = x(3:4);
    
    % Get desired trajectory
    [qd, qd_dot] = desired_trajectory(t);
    
    % Calculate control input
    [tau, ~, ~] = adaptive_pd_controller(q, q_dot, qd, qd_dot, k1, k2, adaptation_rate);
    
    % Calculate dynamic terms
    M = mass_matrix(q, params);
    V = velocity_terms(q, q_dot, params);
    G = gravity_terms(q, params);
    
    % Solve for accelerations
    q_ddot = M \ (tau - V - G);
    
    dx = [q_dot; q_ddot];
end

% Forward Kinematics
function [x, y] = forward_kinematics(theta1, theta2, a1, a2)
    % End of first link
    x1 = a1 * cos(theta1);
    y1 = a1 * sin(theta1);
    
    % End of second link
    x2 = x1 + a2 * cos(theta1 + theta2);
    y2 = y1 + a2 * sin(theta1 + theta2);
    
    x = [0, x1, x2];
    y = [0, y1, y2];
end

% Simulation parameters
params.m1 = m1; params.m2 = m2;
params.a1 = a1; params.a2 = a2;
params.g = g;

% Simulate system
[t, X] = ode45(@(t,x) dynamics(t, x, params, k1, k2, adaptation_rate), t, x0);

% Extract results
theta1 = X(:,1);
theta2 = X(:,2);

% Calculate desired trajectory for plotting
qd_traj = zeros(length(t), 2);
qd_dot_traj = zeros(length(t), 2);
for i = 1:length(t)
    [qd_temp, qd_dot_temp] = desired_trajectory(t(i));
    qd_traj(i,:) = qd_temp';
    qd_dot_traj(i,:) = qd_dot_temp';
end

% Create animation
figure('Position', [100 100 1200 600]);

% Store end-effector trajectory
ee_x = zeros(length(t), 1);
ee_y = zeros(length(t), 1);
ee_x_desired = zeros(length(t), 1);
ee_y_desired = zeros(length(t), 1);

% Animation
for i = 1:10:length(t)
    % Calculate current positions
    [x, y] = forward_kinematics(theta1(i), theta2(i), a1, a2);
    [xd, yd] = forward_kinematics(qd_traj(i,1), qd_traj(i,2), a1, a2);
    
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
fprintf('Mean tracking error for joint 1: %.4f radians\n', mean_error_1);
fprintf('Mean tracking error for joint 2: %.4f radians\n', mean_error_2);
