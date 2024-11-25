function [delta] = steeringInput(a, b, N, v0, vehicle)
% This script will be used to calculate the steering input, to simulate
% scenarios challenging for the torque vectoring controller

% First we need to define the scenarios

% Scenario 1 - Slalom with a certain entrance velocity to the maneuver 
%              for slaloms, cone spacing in considered to be 7.5m
% Scenario 2 - Corner entry with specific entry speed
% Scenario 3 - Fomrula Student skid pad

% Controllers should be optimized arround these regions

% Parameters
vehicle.tf = 1.5; % track width in meters (assumed value)

%% Slalom trajectory
% Define slalom waypoints
slalomPointsX = [-3, 0, 3.75, 7.5, 11.25, 15, 18];
slalomPointsY = [-vehicle.tf/2, -vehicle.tf/2, 0, vehicle.tf/2, 0, -vehicle.tf/2, -vehicle.tf/2];

% Interpolate trajectory for smooth curve
num_points = round((slalomPointsX(end) - slalomPointsX(1))/v0/(b-a)*N); % Number of points to interpolate
x_interp = linspace(min(slalomPointsX), max(slalomPointsX), num_points);
y_interp = interp1(slalomPointsX, slalomPointsY, x_interp, 'spline'); % Smooth trajectory

% Compute heading direction (theta_d) and curvature (kappa)
dx = gradient(x_interp); % Derivative of x
dy = gradient(y_interp); % Derivative of y
ddx = gradient(dx);       % Second derivative of x
ddy = gradient(dy);       % Second derivative of y

theta_d = atan2(dy, dx); % Heading direction at each point
kappa = (dx .* ddy - dy .* ddx) ./ (dx.^2 + dy.^2).^(3/2); % Curvature

% Calculate steering angle (delta)
delta = atan(vehicle.wb .* kappa);

% % Plot the trajectory
% figure;
% plot(x_interp, y_interp, 'b-', 'LineWidth', 2); hold on;
% % plot(slalomPointsX, slalomPointsY, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
% quiver(x_interp, y_interp, cos(theta_d), sin(theta_d), 0.5, 'r'); % Visualize heading
% grid on;
% xlabel('X (m)');
% ylabel('Y (m)');
% title('Interpolated Slalom Trajectory with Heading Directions');
% legend('Interpolated Trajectory', 'Waypoints', 'Heading Direction');
% axis equal;
% 
% % Plot steering input
% figure;
% plot(x_interp, rad2deg(delta), 'g-', 'LineWidth', 2);
% grid on;
% xlabel('X (m)');
% ylabel('Steering Angle (degrees)');
% title('Steering Angle Along the Trajectory');
