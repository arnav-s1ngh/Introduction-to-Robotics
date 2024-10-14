% 2-DOF Robot Arm Simulation with PD Control
% Robotics Assignment 1

clear all;
close all;
clc;

% Parameters
m1 = 1; m2 = 1;  % masses
a1 = 1; a2 = 1;  % link lengths
g = 10;  % gravity

% Time span
time_span = [0 10];

% Initial conditions
q0 = [0; 0];
dq0 = [0; 0];
initial_state = [q0; dq0];

% Controller gains
k1 = 1; k2 = 1;

% Case a: Set-point regulation
qd_a = @(t) [pi/3; pi/4];
dqd_a = @(t) [0; 0];

% Case b: Sinusoidal tracking
qd_b = @(t) [pi/4 * sin(t); pi/5 * cos(t)];
dqd_b = @(t) [pi/4 * cos(t); -pi/5 * sin(t)];

% Choose which case to simulate (comment/uncomment as needed)
qd_func = qd_a;  
dqd_func = dqd_a;

% Define the dynamics function
dynamics = @(t, state) robot_dynamics(t, state, m1, m2, a1, a2, g, k1, k2, qd_func, dqd_func);

% Simulate the system
[t, state] = ode45(dynamics, time_span, initial_state);

% Extract results
q1 = state(:, 1);
q2 = state(:, 2);
dq1 = state(:, 3);
dq2 = state(:, 4);

% Calculate errors and control inputs
errors = zeros(length(t), 2);
control_inputs = zeros(length(t), 2);

for i = 1:length(t)
    qd = qd_func(t(i));
    dqd = dqd_func(t(i));
    
    e = state(i, 1:2)' - qd;
    de = state(i, 3:4)' - dqd;
    
    errors(i, :) = e';
    control_inputs(i, :) = (-k1 * e - k2 * de)';
end

% Plotting
figure;
subplot(3, 1, 1);
plot(t, errors);
title('Tracking Error');
xlabel('Time (s)');
ylabel('Error (rad)');
legend('θ1 error', 'θ2 error');

subplot(3, 1, 2);
plot(t, [dq1, dq2]);
title('Angular Velocities');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
legend('dθ1', 'dθ2');

subplot(3, 1, 3);
plot(t, control_inputs);
title('Control Inputs');
xlabel('Time (s)');
ylabel('Torque (N·m)');
legend('τ1', 'τ2');

% Visualization using Peter Corke's Robotics Toolbox
figure;
robot = SerialLink([Revolute('d', 0, 'a', a1, 'alpha', 0), ...
                    Revolute('d', 0, 'a', a2, 'alpha', 0)], ...
                    'name', '2-DOF Planar Robot');

% Set up the plot
robot.plot([0, 0], 'workspace', [-3 3 -3 3 -3 3]);
hold on;
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
view(2); % Set to 2D view

% Animation loop
for i = 1:10:length(t)
    robot.plot([q1(i), q2(i)], 'workspace', [-3 3 -3 3 -3 3]);
    title(sprintf('Time: %.2f s', t(i)));
    drawnow;
    pause(0.0000000000000001);
end

% Helper function to define the robot dynamics
function dstate = robot_dynamics(t, state, m1, m2, a1, a2, g, k1, k2, qd_func, dqd_func)
    q1 = state(1);
    q2 = state(2);
    dq1 = state(3);
    dq2 = state(4);
    
    % Mass matrix M(q)
    M = [(m1 + m2)*a1^2 + m2*a2^2 + 2*m2*a1*a2*cos(q2), m2*a2^2 + m2*a1*a2*cos(q2);
         m2*a2^2 + m2*a1*a2*cos(q2), m2*a2^2];
    
    % Coriolis and centrifugal terms V(q, dq)
    V = [-m2*a1*a2*(2*dq1*dq2 + dq2^2)*sin(q2);
         m2*a1*a2*dq1^2*sin(q2)];
    
    % Gravity terms G(q)
    G = [(m1 + m2)*g*a1*cos(q1) + m2*g*a2*cos(q1 + q2);
         m2*g*a2*cos(q1 + q2)];
    
    % Desired trajectory and its derivative
    qd = qd_func(t);
    dqd = dqd_func(t);
    
    % Compute errors
    e = [q1; q2] - qd;
    de = [dq1; dq2] - dqd;
    
    % Control input (PD control)
    tau = -k1 * e - k2 * de;
    
    % Compute accelerations
    ddq = M \ (tau - V - G);
    
    % Return the state derivatives
    dstate = [dq1; dq2; ddq];
end
