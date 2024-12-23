% Main Script to Run nonLinearVehicleModel (Multiple Simulations)
clc;
clear nonLinearVehicleModel
clear outputFunction
clear

% Define Runge-Kutta parameters
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
vehicle.Jw = 0.2;            % Wheel inertia (kg*m^2)
vehicle.GR = 15;             % Vehicle gear ratio
% vehicle.TireMaxFx = maxFxForSaFzCombination();
vehicle.Motors = Motors('AMK-FSAE Motors Data.xlsx');

v0 = 10;

% Define control parameters and input variables (example values)
delta = steeringInput(a, b, N, v0, vehicle);    % Steering angle (rad)
delta = [zeros(1, 30000), delta, zeros(1, (N - length(delta) - 30000))];

% Define multiple simulation variations 
% Q = [2 2 4;
%     2 2 4;
%     2 2 4;
%     2 2 4;
%     2 2 4];
Q = [2 2 4];

R = [1];

stateVectorResults = zeros(11, N+1, length(Q(:,1))); % To store yaw rate results
allMotorTorques = zeros(4, N+1, length(Q(:,1)));

fxSS = [200;       % Steady state fx FL
        200;       % Steady state fx FR
        200;       % Steady state fx RL
        200];     % Steady state fx FR

for i = 1:length(Q(:,1))
    [ExpandedMatrices, steerAngleVector] = helpSetupTheProblem(Q(i, :), R(1));

    % Define initial state vector Y0 = [psi_dot, v, beta, accel, beta_dot, omega_FL, omega_FR, omega_RL, omega_RR]
    Y0 = [0;               % Initial yaw rate (psi_dot)
          v0;              % Initial speed (vx, m/s)
          0;               % Initial speed (vy, m/s)
          v0 / vehicle.R;  % Initial wheel speed FL (omega_FL)
          v0 / vehicle.R;  % Initial wheel speed FR (omega_FR)
          v0 / vehicle.R;  % Initial wheel speed RL (omega_RL)
          v0 / vehicle.R;  % Initial wheel speed RR (omega_RR)
          0;               % Integral error
          0;               % Displacement x
          0;
          0];              % Displacement y

    % Set ODE options with OutputFcn
    ode = @(t, Y, delta, ax, ay) nonLinearVehicleModel(t, Y, delta, ax, ay, vehicle, steerAngleVector, fxSS, ExpandedMatrices);
    
    % Run the ODE solver with options
    [t, Y, ax, ay, motorTorques] = RKESys(a, b, N, ode, delta, Y0, A, bhta, tau);
    
    % Store the state vector for each iteration
    for iState =1: length(Y0)
        stateVectorResults(iState,:, i) = Y(iState, :);
    end

    % Store motor torque commands for all simulations completed
    for iMotor = 1:4
        allMotorTorques(iMotor, :, i) = motorTorques(iMotor, :);
    end

    % Dimensions of state vector are States x steps x variations of
    % controller
end
%%
% Display results 
legendObject = [];
% Optional: Plot yaw rate results for each mass
figure;
plot(t(2:end), sqrt(Y(2,2:end).^2 + Y(3,2:end).^2) .* delta / vehicle.wb)
hold on
for iPlot = 1:length(Q(:,1))
    plot(t, stateVectorResults(1,:,iPlot), 'LineWidth', 1.5);
    legendObject = [legendObject, "Yaw Rate " + mat2str(Q(iPlot,:))];
end
legendObject = ["Desired Yaw Rate", legendObject];
legend(legendObject)
title('Yaw Rate vs Time');
xlabel('Time (s)');
ylabel('Yaw Rate (rad/s)');
grid on;
hold off

figure('Position',[100 100 1400 400]);
legendObject = [];
numPlots = length(Q(:,1)); % Number of trajectories

% Create a tiled layout for better management of subplots and colorbars
tiledlayout(1, numPlots, 'Padding', 'compact', 'TileSpacing', 'compact');

for iPlot = 1:numPlots
    nexttile;
    cmap = 'winter'; 
    colormap(cmap);  

    scatter(stateVectorResults(9,:,iPlot), stateVectorResults(10,:,iPlot), 24, t, ...
            'Marker', '.');
    title(['Trajectory ', mat2str(Q(iPlot,:))]);
    
    % Add labels for axes
    xlabel('X (m)');
    ylabel('Y (m)');
    grid on;
    legendObject = [legendObject, "Trajectory  " + mat2str(Q(iPlot,:))];
    title("Trajectory  " + mat2str(Q(iPlot,:)))
end
c = colorbar;
c.Label.String = 'Time (s)'; 
grid on;


%% plot velocity
figure 
plot(t(2:end), sqrt(Y(2,2:end).^2 + Y(3,2:end).^2))


%% Plot each wheel
% Motor torque plotting
figure
hold on
for i = 1:4
    subplot(2, 2, i);
    hold on
    for iFigure = 1:length(Q(:,1))
        plot(t, allMotorTorques(i, :, iFigure), 'DisplayName', "Trajectory  " + mat2str(Q(iFigure,:)));
        title(['Wheel Motor Torque ', num2str(i), ' Over Time']);
        xlabel('Time (s)');
        ylabel('Motor Torque (Nm)');
        grid on
    end
    hold off
end
% add a bit space to the figure
fig = gcf;
fig.Position(3) = fig.Position(3) + 400;
% add legend
Lgnd = legend('show');
Lgnd.Position(1) = 0.01;
Lgnd.Position(2) = 0.4;
hold off

% Wheel angular velocity plots
figure 
hold on
for i = 1:4
    subplot(2, 2, i);
    hold on
    for iFigure = 1:length(Q(:,1))
        plot(t, stateVectorResults(i+3, :, iFigure), 'DisplayName', "Trajectory  " + mat2str(Q(iFigure,:)));
        title(['Wheel Speed ', num2str(i), ' Over Time']);
        xlabel('Time (s)');
        ylabel('Motor Speed (rad/s)');
        grid on
    end
    hold off
end
% add a bit space to the figure
fig = gcf;
fig.Position(3) = fig.Position(3) + 400;
% add legend
Lgnd = legend('show');
Lgnd.Position(1) = 0.01;
Lgnd.Position(2) = 0.4;
hold off



%% plot vy
figure 
plot(t, Y(3,:))