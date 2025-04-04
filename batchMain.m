% Main Script to Run nonLinearVehicleModel (Multiple Simulations)
clc;
clear nonLinearVehicleModel
clear vehicleOutputFcn
close all

% Simulation paramters
a = 0; b = 6; % a (simulation start time), b (simulation end time) in seconds

addpath(genpath('./'))

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
vehicle.Jz = 100;             % Yaw moment of inertia (kg*m^2)
vehicle.Jw = 0.2;            % Wheel inertia (kg*m^2)
vehicle.GR = 15;             % Vehicle gear ratio
% vehicle.TireMaxFx = maxFxForSaFzCombination();
vehicle.Motors = Motors('AMK-FSAE Motors Data.xlsx');

v0 = 12;

% Define multiple simulation variations 
% Q = [25 50 7.25;
%     50 100 15;
%     50 100 2.5;
%     75 150 7.5];
Q = [5 90 7.5];

R = [1];

stateVectorResults = []; % To store yaw rate results
allMotorTorques = [];
allSlipAngles = [];
allSlipRatios = [];
allFxForces   = [];
allFyForces   = [];
allFzForces   = [];
times = [];

fxSS = [110;  % Steady state fx FL
        110;  % Steady state fx FR
        110;  % Steady state fx RL
        110]; % Steady state fx FR

% Mode describes the controller type used. Can be:
%   "stateSpace"
%   "fuzzy"
%   "openLoop"
mode = ["stateSpace", "fuzzy", "openLoop"]; 
ratio = 1;
rampRate = 0.01;

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


    hWaitBar = waitbar(0, 'Solving ODE...');
    delta = @(t) deltaFunc(t);
    tspan = [a  b];
    % options = odeset('OutputFcn', @(t, Y, flag) vehicleOutputFcn(t, Y, flag, delta, vehicle, steerAngleVector, fxSS, ExpandedMatrices, tspan, hWaitBar, mode), ...
    %              'Refine', 1, ...              % Increase the refinement level (default is 1)
    %              'OutputSel', [], ...
    %              'Stats', 'on');               % Optional: Display solver statistics

    options = odeset('OutputFcn', @(t, Y, flag) vehicleOutputFcn(t, Y, flag, delta, vehicle, steerAngleVector, fxSS, ExpandedMatrices, tspan, hWaitBar, mode,ratio, rampRate), ...
                 'Refine', 1, ...              % Increase the refinement level (default is 1)
                 'RelTol', 1e-6, ...           % Set a smaller relative tolerance for higher accuracy
                 'AbsTol', 1e-8, ...           % Set a smaller absolute tolerance for higher accuracy
                 'OutputSel', [], ...
                 'Stats', 'on');               % Optional: Display solver statistics


    % Run the solver

    [t, Y] = ode45(@(t, Y) nonLinearVehicleModel(t, Y, delta, vehicle, steerAngleVector, fxSS, ExpandedMatrices,mode,ratio, rampRate), tspan, Y0, options);

    times = [times struct('times', t)];

    % Store the state vector for each iteration
    theseStates = struct ("state", zeros(size(Y'))); 
    for iStep = 1:length(Y(:,1))
        theseStates.state(:, iStep) = Y(iStep, :);
    end
    stateVectorResults = [stateVectorResults theseStates];
%%
    % Store motor torque commands for all simulations completed
    theseMotorTorques = struct ('FL', zeros(1, length(Y(:,1))),...
                              'FR', zeros(1, length(Y(:,1))),...
                              'RL', zeros(1, length(Y(:,1))),...
                              'RR', zeros(1, length(Y(:,1)))); 
    for iStep = 1:length(Y(:,1))-1
        theseMotorTorques.FL(iStep) = logData.aux(iStep).MotorTorques.FL;
        theseMotorTorques.FR(iStep) = logData.aux(iStep).MotorTorques.FR;
        theseMotorTorques.RL(iStep) = logData.aux(iStep).MotorTorques.RL;
        theseMotorTorques.RR(iStep) = logData.aux(iStep).MotorTorques.RR;
    end
%%
    % Store slip angles for all simulations completed
    theseSlipAngles = struct ('FL', zeros(1, length(Y(:,1))),...
                              'FR', zeros(1, length(Y(:,1))),...
                              'RL', zeros(1, length(Y(:,1))),...
                              'RR', zeros(1, length(Y(:,1)))); 
    for iStep = 1:length(Y(:,1))-1
        theseSlipAngles.FL(iStep) = logData.aux(iStep).slipAngles.FL;
        theseSlipAngles.FR(iStep) = logData.aux(iStep).slipAngles.FR;
        theseSlipAngles.RL(iStep) = logData.aux(iStep).slipAngles.RL;
        theseSlipAngles.RR(iStep) = logData.aux(iStep).slipAngles.RR;
    end



    % Store slip ratios for all simulations completed
    theseSlipRatios = struct ('FL', zeros(1, length(Y(:,1))),...
                              'FR', zeros(1, length(Y(:,1))),...
                              'RL', zeros(1, length(Y(:,1))),...
                              'RR', zeros(1, length(Y(:,1)))); 
    for iStep = 1:length(Y(:,1))-1
        theseSlipRatios.FL(iStep) = logData.aux(iStep).slipRatios.FL;
        theseSlipRatios.FR(iStep) = logData.aux(iStep).slipRatios.FR;
        theseSlipRatios.RL(iStep) = logData.aux(iStep).slipRatios.RL;
        theseSlipRatios.RR(iStep) = logData.aux(iStep).slipRatios.RR;
    end

    % Fx forces
    theseFx = struct ('FL', zeros(1, length(Y(:,1))),...
                              'FR', zeros(1, length(Y(:,1))),...
                              'RL', zeros(1, length(Y(:,1))),...
                              'RR', zeros(1, length(Y(:,1)))); 
    for iStep = 1:length(Y(:,1))-1
        theseFx.FL(iStep) = logData.aux(iStep).Forces.Fx.FL;
        theseFx.FR(iStep) = logData.aux(iStep).Forces.Fx.FR;
        theseFx.RL(iStep) = logData.aux(iStep).Forces.Fx.RL;
        theseFx.RR(iStep) = logData.aux(iStep).Forces.Fx.RR;
    end

    % Fy forces
    theseFy = struct ('FL', zeros(1, length(Y(:,1))),...
                              'FR', zeros(1, length(Y(:,1))),...
                              'RL', zeros(1, length(Y(:,1))),...
                              'RR', zeros(1, length(Y(:,1)))); 
    for iStep = 1:length(Y(:,1))-1
        theseFy.FL(iStep) = logData.aux(iStep).Forces.Fy.FL;
        theseFy.FR(iStep) = logData.aux(iStep).Forces.Fy.FR;
        theseFy.RL(iStep) = logData.aux(iStep).Forces.Fy.RL;
        theseFy.RR(iStep) = logData.aux(iStep).Forces.Fy.RR;
    end

    % Fz forces
    theseFz = struct ('FL', zeros(1, length(Y(:,1))),...
                              'FR', zeros(1, length(Y(:,1))),...
                              'RL', zeros(1, length(Y(:,1))),...
                              'RR', zeros(1, length(Y(:,1)))); 
    for iStep = 1:length(Y(:,1))-1
        theseFz.FL(iStep) = logData.aux(iStep).Forces.Fz.FL;
        theseFz.FR(iStep) = logData.aux(iStep).Forces.Fz.FR;
        theseFz.RL(iStep) = logData.aux(iStep).Forces.Fz.RL;
        theseFz.RR(iStep) = logData.aux(iStep).Forces.Fz.RR;
    end

    allMotorTorques = [allMotorTorques theseMotorTorques];
    allSlipAngles = [allSlipAngles theseSlipAngles];
    allSlipRatios = [allSlipRatios theseSlipRatios];
    allFxForces   = [allFxForces theseFx];
    allFyForces   = [allFyForces theseFy];
    allFzForces   = [allFzForces theseFz];

    % Dimensions of state vector are States x steps x variations of
    % controller
end
%%
% Display yaw rate results
legendObject = [];
figure;

steerVector = @(t) arrayfun(@(t_scalar) deltaFunc(t_scalar), t);
desiredYawRate = [logData.aux.psiDotTarg];
% Desired yaw rate
plot(t(2:end), desiredYawRate, 'LineWidth', 2, 'DisplayName', "Desired Yaw Rate");
hold on;

% Yaw rate for each Q configuration
for iPlot = 1:length(Q(:,1))
    plot(times(iPlot).times, stateVectorResults(iPlot).state(1,:), 'LineWidth', 1.5, 'DisplayName', "Yaw Rate " + mat2str(Q(iPlot,:)));
end
legend show;
title('Yaw Rate vs Time');
xlabel('Time (s)');
ylabel('Yaw Rate (rad/s)');
grid on;
hold off;

% Trajectories
figure('Position',[100 100 1400 400]);
tiledlayout(1, length(Q(:,1)), 'Padding', 'compact', 'TileSpacing', 'compact');

for iPlot = 1:length(Q(:,1))
    nexttile;
    scatter(stateVectorResults(iPlot).state(9,:), stateVectorResults(iPlot).state(10,:), 24, times(iPlot).times, 'Marker', '.')
    xlabel('X (m)');
    ylabel('Y (m)');
    title(['Trajectory ', mat2str(Q(iPlot,:))]);
    grid on;
end
c = colorbar;
c.Label.String = 'Time (s)';
sgtitle('Vehicle Trajectories');

% Velocity over time
figure;
plot(t, sqrt(Y(:,2).^2 + Y(:,3).^2), 'LineWidth', 2);
title('Vehicle Velocity Over Time');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
grid on;
%% Debug plots
% %% Motor torque
% figure;
% fieldNames = string(fieldnames(allMotorTorques));
% for iField = 1:length(fieldNames)
%     subplot(2, 2, iField);
%     hold on;
%     for iFigure = 1:length(Q(:,1))
%         plot(times(iFigure).times, allMotorTorques(iFigure).(fieldNames(iField))', 'DisplayName', "Trajectory " + mat2str(Q(iFigure,:)));
%     end
%     title(['Motor Torque - Wheel ', num2str(i)]);
%     xlabel('Time (s)');
%     ylabel('Torque (Nm)');
%     grid on;
%     legend show;
%     hold off;
% end
% %%
% % Wheel angular velocities
% figure;
% for i = 1:4
%     subplot(2, 2, i);
%     hold on;
%     for iFigure = 1:length(Q(:,1))
%         plot(times(iFigure).times, squeeze(stateVectorResults(iFigure).state(i+3,:)), 'DisplayName', "Trajectory " + mat2str(Q(iFigure,:)));
%     end
%     title(['Wheel Angular Velocity - ', num2str(i)]);
%     xlabel('Time (s)');
%     ylabel('Angular Velocity (rad/s)');
%     grid on;
%     legend show;
%     hold off;
% end
% 
% % Wheel slip angles
% figure;
% fieldNames = string(fieldnames(allSlipAngles));
% for iField = 1:length(fieldNames)
%     subplot(2, 2, iField);
%     hold on;
%     for iFigure = 1:length(Q(:,1))
%         plot(times(iFigure).times, rad2deg(allSlipAngles(iFigure).(fieldNames(iField))'), 'DisplayName', "Trajectory " + mat2str(Q(iFigure,:)));
%     end
%     title(['Slip Angle - Wheel ', num2str(i)]);
%     xlabel('Time (s)');
%     ylabel('Slip Angle (deg)');
%     ylim([-25 25])
%     grid on;
%     legend show;
%     hold off;
% end
% 
% % Wheel slip ratios
% figure;
% fieldNames = string(fieldnames(allSlipRatios));
% for iField = 1:length(fieldNames)
%     subplot(2, 2, iField);
%     hold on;
%     for iFigure = 1:length(Q(:,1))
%         plot(times(iFigure).times, allSlipRatios(iFigure).(fieldNames(iField))', 'DisplayName', "Trajectory " + mat2str(Q(iFigure,:)));
%     end
%     title(['Slip Ratio - Wheel ', num2str(i)]);
%     xlabel('Time (s)');
%     ylabel('Slip Ratio (Nm)');
%     ylim([-1 1])
%     grid on;
%     legend show;
%     hold off;
% end
% 
% %% Wheel Forces
% % Wheel Fx forces
% figure;
% fieldNames = string(fieldnames(allFxForces));
% for iField = 1:length(fieldNames)
%     subplot(2, 2, iField);
%     hold on;
%     for iFigure = 1:length(Q(:,1))
%         plot(times(iFigure).times, allFxForces(iFigure).(fieldNames(iField))', 'DisplayName', "Trajectory " + mat2str(Q(iFigure,:)));
%     end
%     title(['Fx - Wheel ', num2str(i)]);
%     xlabel('Time (s)');
%     ylabel('Fx (N)');
%     grid on;
%     legend show;
%     hold off;
% end
% 
% % Wheel Fy forces
% figure;
% fieldNames = string(fieldnames(allFyForces));
% for iField = 1:length(fieldNames)
%     subplot(2, 2, iField);
%     hold on;
%     for iFigure = 1:length(Q(:,1))
%         plot(times(iFigure).times, allFyForces(iFigure).(fieldNames(iField))', 'DisplayName', "Trajectory " + mat2str(Q(iFigure,:)));
%     end
%     title(['Fy - Wheel ', num2str(i)]);
%     xlabel('Time (s)');
%     ylabel('Fy (N)');
%     grid on;
%     legend show;
%     hold off;
% end
% 
% 
% % Wheel Fz forces
% figure;
% fieldNames = string(fieldnames(allFzForces));
% for iField = 1:length(fieldNames)
%     subplot(2, 2, iField);
%     hold on;
%     for iFigure = 1:length(Q(:,1))
%         plot(times(iFigure).times, allFzForces(iFigure).(fieldNames(iField))', 'DisplayName', "Trajectory " + mat2str(Q(iFigure,:)));
%     end
%     title(['Fz - Wheel ', num2str(i)]);
%     xlabel('Time (s)');
%     ylabel('Fz (N)');
%     grid on;
%     legend show;
%     hold off;
% end
% 
% % Plot side slip angle
% beta = [logData.aux(:).b];
% figure;
% plot(t(2:end), beta, 'LineWidth', 2);
% title('Side Slip angle Over Time');
% xlabel('Time (s)');
% ylabel('Velocity (m/s)');
% grid on;
% 
% % Plot side slip angle
% yawError = [logData.aux.yawError];
% figure;
% plot(t(2:end), yawError, 'LineWidth', 2, 'DisplayName',"Yaw Error");
% title('Yaw Error Over Time');
% xlabel('Time (s)');
% grid on;
% hold on
% plot(t(2:end), yawError * ExpandedMatrices.Kp(1), 'LineWidth', 2, 'DisplayName',"Proportional Input");
% plot(t(2:end),stateVectorResults.state(8, 2:end) * ExpandedMatrices.Ki(1), 'LineWidth', 2, 'DisplayName',"Integral Input");
% plot(t(2:end),-ExpandedMatrices.Kr(1, :)*[beta;stateVectorResults.state(1, 2:end)], 'LineWidth', 2, 'DisplayName',"Regulate Input");
% title('Yaw Error Over Time');
% xlabel('Time (s)');
% ylabel('Magnitude');
% legend('show')
% 
% % plot yaw acceleration error 
% yawAccError = [logData.aux.psiDDotError];
% figure;
% plot(t(2:end), yawAccError, 'LineWidth', 2, 'DisplayName',"Yaw Accel Error");
% hold on
% plot(t(2:end), yawError, 'LineWidth', 2, 'DisplayName',"Yaw Error");
% title('Yaw Accel Over Time');
% xlabel('Time (s)');
% ylabel('Yaw acc (rad/s^2)');
% legend('show')
% grid on;