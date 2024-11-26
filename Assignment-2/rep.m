% Pure Pursuit Controller for Unicycle Robot

% Simulation Parameters
dt = 0.01;          % Time step
T = 10;             % Total simulation time
v = 2;              % Linear velocity (constant)
L = 0.5;            % Look-ahead distance
radius = 1;         % Path radius

% Initial Conditions
x = 0;              % Initial x-position
y = 0;              % Initial y-position
theta = 0;          % Initial orientation

% Storage for history
t_hist = 0:dt:T;
x_hist = zeros(size(t_hist));
y_hist = zeros(size(t_hist));
theta_hist = zeros(size(t_hist));

% Simulation Loop
for i = 1:length(t_hist)
    % Store current state
    x_hist(i) = x;
    y_hist(i) = y;
    theta_hist(i) = theta;
    
    % Calculate look-ahead point on circular path
    r = sqrt(x^2 + y^2);
    current_angle = atan2(y, x);
    lookahead_angle = current_angle + v * dt / radius;
    
    x_lookahead = radius * cos(lookahead_angle);
    y_lookahead = radius * sin(lookahead_angle);
    
    % Calculate heading to look-ahead point
    dx = x_lookahead - x;
    dy = y_lookahead - y;
    desired_heading = atan2(dy, dx);
    
    % Compute heading error
    heading_error = desired_heading - theta;
    heading_error = atan2(sin(heading_error), cos(heading_error));
    
    % Compute angular velocity
    omega = (2 * v / L) * sin(heading_error);
    
    % Update robot state
    x = x + v * cos(theta) * dt;
    y = y + v * sin(theta) * dt;
    theta = theta + omega * dt;
end

% Plotting
figure;

% Trajectory Plot
subplot(2,2,[1,3]);
plot(x_hist, y_hist, 'b-', 'LineWidth', 2);
hold on;

% Reference Circle
theta_circle = linspace(0, 2*pi, 100);
x_circle = cos(theta_circle);
y_circle = sin(theta_circle);
plot(x_circle, y_circle, 'r--', 'LineWidth', 1.5);

title('Robot Trajectory');
xlabel('X Position');
ylabel('Y Position');
axis equal;
grid on;

% Orientation Plot
subplot(2,2,2);
plot(t_hist, theta_hist, 'g-');
title('Orientation vs Time');
xlabel('Time');
ylabel('Orientation (rad)');

% Position Plot
subplot(2,2,4);
plot(t_hist, x_hist, 'b-');
hold on;
plot(t_hist, y_hist, 'r-');
title('Position vs Time');
xlabel('Time');
ylabel('Position');
legend('X', 'Y');
