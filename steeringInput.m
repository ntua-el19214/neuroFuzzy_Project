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
totalSlalomX = 15;

% Interpolate trajectory for smooth curve
timeStep = (b-a)/N;
steeringTimeVector = 0:timeStep:totalSlalomX/v0;
delta = -deg2rad(20)*sin(2*pi/totalSlalomX*v0*steeringTimeVector);

% Plot steering input
figure;
plot(steeringTimeVector, rad2deg(delta), 'g-', 'LineWidth', 2);
grid on;
xlabel('X (m)');
ylabel('Steering Angle (degrees)');
title('Steering Angle Along the Trajectory');
