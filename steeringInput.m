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
slalomEndTime = 3.5;

% Interpolate trajectory for smooth curve
timeStep = (b-a)/N;
steeringTimeVector = 0:timeStep:slalomEndTime;
delta1 = -deg2rad(10)*sin(2*pi/(slalomEndTime*2/5)*steeringTimeVector); 
delta2 = deg2rad(20)*sin(2*pi/(slalomEndTime*3/5)*steeringTimeVector);

delta = zeros(1,length(steeringTimeVector));
delta1IdxFirst  = round((pi/(2*pi/(slalomEndTime*2/5))/timeStep));
delta2Idx       = delta1IdxFirst +1 + round(((slalomEndTime*3/5))/timeStep);

delta(1:delta1IdxFirst) = delta1(1:delta1IdxFirst);
delta(delta1IdxFirst+1:delta2Idx) = delta2(1:delta2Idx-delta1IdxFirst);
delta(delta2Idx+1:end) = 0.5*delta1(delta1IdxFirst+1:2*delta1IdxFirst);

% Plot steering input
figure;
plot(steeringTimeVector, rad2deg(delta), 'g-', 'LineWidth', 2);
grid on;
xlabel('X (m)');
ylabel('Steering Angle (degrees)');
title('Steering Angle Along the Trajectory');
