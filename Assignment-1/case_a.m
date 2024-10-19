% 2-DOF Robot Arm Simulation with 2D Visualization - Set-point Regulation (Case A)
clear all; close all; clc;

% System Parameters
m1 = 1; m2 = 1;  % masses
a1 = 1; a2 = 1;  % link lengths
g = 10;          % gravity

% Control gains
k1 = 1; k2 = 1;
adaptation_rate = 200000;  % Learning rate for adaptation

% Time settings
t_span = [0 10];
dt = 0.01;
t = t_span(1):dt:t_span(2);

% Initial conditions [θ1 θ2 θ1_dot θ2_dot]
x0 = [0 0 0 0];

% Desired setpoint [pi/3, pi/4]
qd = [pi/3; pi/4];
qd_dot = [0; 0];

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
function dx = dynamics(t, x, params, qd, qd_dot, k1, k2, adaptation_rate)
    q = x(1:2);
    q_dot = x(3:4);
    
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
[t, X] = ode45(@(t,x) dynamics(t, x, params, qd, qd_dot, k1, k2, adaptation_rate), t, x0);

% Extract results
theta1 = X(:,1);
theta2 = X(:,2);

% Create animation
figure('Position', [100 100 800 600]);
subplot(1,2,1); % Left subplot for arm animation

% Calculate desired end position
[xd, yd] = forward_kinematics(qd(1), qd(2), a1, a2);

% Animation
for i = 1:10:length(t)
    % Calculate current end positions
    [x, y] = forward_kinematics(theta1(i), theta2(i), a1, a2);
    
    subplot(1,2,1);
    % Plot robot arm
    plot(x, y, 'b-', 'LineWidth', 2); hold on;
    plot(x, y, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
    % Plot desired position
    plot(xd, yd, 'r--', 'LineWidth', 1);
    plot(xd, yd, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
    
    grid on;
    axis equal;
    axis([-2.5 2.5 -2.5 2.5]);
    title('2-DOF Robot Arm Motion');
    xlabel('X Position');
    ylabel('Y Position');
    legend('Current Position', 'Joints', 'Desired Position', 'Desired Joints');
    hold off;
    
    % Right subplot for joint angles
    subplot(1,2,2);
    plot(t(1:i), theta1(1:i), 'b-', t(1:i), theta2(1:i), 'r-');
    hold on;
    plot([t(1) t(end)], [qd(1) qd(1)], 'b--');
    plot([t(1) t(end)], [qd(2) qd(2)], 'r--');
    hold off;
    grid on;
    title('Joint Angles');
    xlabel('Time (s)');
    ylabel('Angle (rad)');
    legend('\theta_1', '\theta_2', '\theta_1 desired', '\theta_2 desired');
    
    drawnow;
    
    % Optional: Uncomment to save frames for video
    % frame = getframe(gcf);
    % writeVideo(v, frame);
end

% Calculate and display final error
final_error = norm([theta1(end) theta2(end)] - qd');
fprintf('Final position error: %.4f radians\n', final_error);
