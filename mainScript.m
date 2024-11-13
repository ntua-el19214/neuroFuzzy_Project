% Main Script to Run nonLinearVehicleModel
clc;
% clear;

% Define vehicle parameters (example values)
vehicle.m  = 270;            % Mass of the vehicle (kg)
vehicle.cd = 2;              % Coefficient of drag
vehicle.cl = 7;              % Coefficient of lift
vehicle.wb = 1.57;           % Wheelbase (m)
vehicle.wd = 0.5;            % Weight distribution (% front)
vehicle.tf = 1.22;           % Track width front (m)
vehicle.tr = 1.22;           % Track width rear (m)
vehicle.R = 0.25;            % Wheel radius (m)
vehicle.CoGz = 0.3;          % Center of Gravity height (m)
vehicle.Jz = 100;            % Yaw moment of inertia (kg*m^2)
vehicle.Jw = 0.6;            % Wheel inertia (kg*m^2)
vehicle.GR = 15;             % Vehicle gear ratio
%vehicle.TireMaxFx = maxFxForSaFzCombination();
vehicle.Motors = Motors('AMK-FSAE Motors Data.xlsx');


% Define control parameters and input variables (example values)
delta            = 0.05;      % Steering angle (rad)
load("ssSteerAVector.mat");           % Steady State linearization points Steering angle (example empty)
load("ExpandedMatrices.mat");         % Matrices struct for control (example empty)

% Define initial state vector Y0 = [psi_dot, v, beta, accel, beta_dot, omega_FL, omega_FR, omega_RL, omega_RR]
Y0 = [0;    % Initial yaw rate (psi_dot)
      10;   % Initial speed (v, m/s)
      0;    % Initial beta_dot
      0;    % Initial wheel speed FL (omega_FL)
      0;    % Initial wheel speed FR (omega_FR)
      0;    % Initial wheel speed RL (omega_RL)
      0];   % Initial wheel speed RR (omega_RR)

% Simulation time
tspan = [0 10];   % Time interval (seconds)

% Define anonymous function for ODE solver
dynamicsFunc = @(t, Y) nonLinearVehicleModel(t, Y, vehicle, delta, steerAngleVector, ExpandedMatrices);

% Run the ODE solver
[t, Y] = ode45(dynamicsFunc, tspan, Y0);

% Plot Results
figure;
subplot(3, 1, 1);
plot(t, Y(:, 1));
title('Yaw Rate (psi\_dot)');
xlabel('Time (s)');
ylabel('Yaw Rate (rad/s)');

subplot(3, 1, 2);
plot(t, Y(:, 2));
title('Speed (v)');
xlabel('Time (s)');
ylabel('Speed (m/s)');

subplot(3, 1, 3);
plot(t, Y(:, 3));
title('Side Slip Angle (beta)');
xlabel('Time (s)');
ylabel('Beta (rad)');

% Add any additional plots if needed
