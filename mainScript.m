% Main Script to Run nonLinearVehicleModel
clc;
clear nonLinearVehicleModel
clear outputFunction
clear

% Define Runge-Kuta parameters
a = 0; b = 4; % a (simulation start time), b (simulation end time) in seconds
N = 500000;   % Number of steps (nodes)

A = [0 0 0 0 0; 1/3 0 0 0 0; 1/6 1/6 0 0 0; 1/8 0 3/8 0 0; 1/2 0 -3/2 2 0];
tau =  [0; 1/3; 1/3; 1/2; 1];
bhta = [1/6; 0; 0; 2/3; 1/6];

addpath(genpath('./'))
close all

% Define vehicle parameters (example values)
vehicle.m  = 270;            % Mass of the vehicle (kg)
vehicle.cd = 2;              % Coefficient of drag
vehicle.cl = 7;              % Coefficient of lift
vehicle.wb = 1.57;           % Wheelbase (m)
vehicle.wd = 0.5;            % Weight distribution (% front)
vehicle.tf = 1.22;           % Track width front (m)
vehicle.tr = 1.22;           % Track width rear (m)
vehicle.R = 0.21;            % Wheel radius (m)
vehicle.CoGz = 0.3;          % Center of Gravity height (m)
vehicle.Jz = 100;            % Yaw moment of inertia (kg*m^2)
vehicle.Jw = 0.2;           % Wheel inertia (kg*m^2)
vehicle.GR = 15;             % Vehicle gear ratio
% vehicle.TireMaxFx = maxFxForSaFzCombination();
vehicle.Motors = Motors('AMK-FSAE Motors Data.xlsx');

v0 = 10;
% Define control parameters and input variables (example values)
delta = steeringInput(a,b,N,v0,vehicle);    % Steering angle (rad)
delta = [zeros(1,100000) ,delta , zeros(1,(N - length(delta)-100000))];

figure 
plot(delta*57.2957795, 'LineWidth',1.5)
grid on
xlabel("Time Steps")
ylabel("Steering Input [deg]")

load("ssSteerAVector.mat");                 % Steady State linearization points Steering angle (example empty)
load("ExpandedMatrices.mat");               % Matrices struct for control (example empty)

fxSS = [30.6;       % Steady state fx FL
        30.6;       % Steady state fx FR
        30.6;       % Steady state fx RL
        30.6;];     % Steady state fx FR

% Define initial state vector Y0 = [psi_dot, v, beta, accel, beta_dot, omega_FL, omega_FR, omega_RL, omega_RR]
Y0 = [0;               % Initial yaw rate (psi_dot)
      v0;              % Initial speed (vx, m/s)
      0;               % Initial speed (vy, m/s)
      v0/vehicle.R;    % Initial wheel speed FL (omega_FL)
      v0/vehicle.R;    % Initial wheel speed FR (omega_FR)
      v0/vehicle.R;    % Initial wheel speed RL (omega_RL)
      v0/vehicle.R;    % Initial wheel speed RR (omega_RR)
      0;               % intergal error
      0;               % displacement x
      0;
      0];              % displacement y

% Simulation time
tspan = [0 10];   % Time interval (seconds)
% Set ODE options with OutputFcn
ode = @(t, Y, delta,ax, ay) nonLinearVehicleModel(t, Y, delta, ax, ay, vehicle, steerAngleVector, fxSS, ExpandedMatrices);
% Run the ODE solver with options
[t, Y, ax, ay] = RKESys(a,b,N,ode,delta, Y0,A,bhta,tau);

% Plot Results
% The variables Tmotor_values, yaw_error_values, and time_values should now be available in the base workspace.
%% Plot
% figure;
% for i = 1:4
%     subplot(2, 2, i);
%     plot(time_values, Tmotor_values(:, i));
%     title(['Motor Torque ', num2str(i), ' Over Time']);
%     xlabel('Time (s)');
%     ylabel('Torque (Nm)');
% end

% figure;
% plot(t(2:end), yaw_error_values);
% title('Yaw Error Over Time');
% xlabel('Time (s)');
% ylabel('Yaw Error (rad/s)');

figure;
plot(t, Y(2,:));
title('Velocity Over Time');
xlabel('Time (s)');
ylabel('Velocity (m/s)');

figure
for i = 1:4
    subplot(2, 2, i);
    plot(t, Y(i+3,:));
    title(['Wheel Speed ', num2str(i), ' Over Time']);
    xlabel('Time (s)');
    ylabel('Motor Speed (rad/s)');
end

figure;
plot(t, ax);
title('Acceleration x Over Time');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');

figure;
plot(t, ay);
title('Acceleration y Over Time');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');

figure;
plot(t, Y(9,:));
title('Displacement X');
xlabel('Time (s)');
ylabel('X (m)');

figure;
plot(t, Y(10,:));
title('Displacement Y');
xlabel('Time (s)');
ylabel('Y (m)');

figure;
plot(Y(9,:), Y(10,:));
title('Vehicle Trajectory (on vehicle frame)');
xlabel('Displacement X (m)');
ylabel('Displacement Y (m)');

figure;
plot(t(2:end), sqrt(Y(2,2:end).^2 + Y(3,2:end).^2) .* delta / vehicle.wb);
hold on;
plot(t(2:end),Y(1,2:end))
title('Target Yaw Rate vs Actual Yaw Rate');
xlabel('Time (s)');
ylabel('Yaw Rate (rad/s)');