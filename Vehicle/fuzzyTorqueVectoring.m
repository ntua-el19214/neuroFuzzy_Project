function [Mz, smoothedTorqueCommand] = fuzzyTorqueVectoring(psi_dot_err, psi_ddot_err, Fz, vehicle, slipAngleMatrix, wheel_omega_array, tSS, ratio, rampRate)
% Parameters
maxMz = 14000; % Maximum yaw moment (Nm)

% Define persistent variable for previous torque command
persistent prevTorqueCommand;
if isempty(prevTorqueCommand)
    prevTorqueCommand = zeros(4, 1); % Initialize to zero for all wheels
end

% Define fuzzy membership functions for psi_dot_err
psiDotErrMFs = [-0.5, -0.10, -0.04, 0, 0.04, 0.10, 0.5]; % Centers for NB to PB
psiDotErrWidth  = 0.028; % Width of each MF

% Define fuzzy membership functions for psi_ddot_err
psiDDotErrMFs = [-8.5, -5.8, -3.95, 0, 3.95, 5.8, 8.5]; % Centers for NB to PB
psiDDotErrWidth  = 0.73; % Width of each MF

% Rule table (Mz output levels): From paper Table I
ruleBase = [
    -4, -4, -4, -3, -2,  -1,  0;  % Row for NB (Error Yaw Acc.)
    -4, -4, -3, -2, -1,  0,  1;  % Row for NM
    -4, -3, -2, -1,  0,  1,  2;  % Row for NS
    -3, -2, -1,  0,  1,  2,  3;  % Row for Z
    -2, -1,  0,  1,  2,  3,  4;  % Row for PS
    -1,  0,  1,  2,  3,  4,  4;  % Row for PM
    0,  1,  2,  3,  4,  4,  4;  % Row for PB
    ];
% Fuzzification: Calculate membership values for each input
psiDotMembership = calcMembership(psi_dot_err, psiDotErrMFs, psiDotErrWidth);
psiDDotMembership = calcMembership(psi_ddot_err, psiDDotErrMFs, psiDDotErrWidth);

ruleStrength = psiDotMembership' * psiDDotMembership;
Mz = sum(sum(ruleStrength.*ruleBase)/4*maxMz);
weightSum = sum(sum(ruleStrength));

if weightSum > 0
    Mz = Mz / weightSum; % Defuzzification
else
    Mz = 0; % No output if inputs are zero
end

% Torque Vectoring Algorithm: Distribute Mz to four wheels
targetTorqueCommand = distributeTorque(Mz, Fz, vehicle, ratio);
for iMotor=1:length(wheel_omega_array)
    maxAvailableMotorTorque = vehicle.Motors(wheel_omega_array(iMotor)*vehicle.GR);
    targetTorqueCommand(iMotor) = max(min(targetTorqueCommand(iMotor)+ tSS(iMotor), maxAvailableMotorTorque),0);
end

% Apply ramp rate for smooth torque transition
smoothedTorqueCommand = prevTorqueCommand; % Initialize with previous torque values
for i = 1:length(targetTorqueCommand)
    delta = targetTorqueCommand(i) - prevTorqueCommand(i);
    if abs(delta) > rampRate
        smoothedTorqueCommand(i) = prevTorqueCommand(i) + sign(delta) * rampRate;
    else
        smoothedTorqueCommand(i) = targetTorqueCommand(i);
    end
end

% Update the persistent variable
prevTorqueCommand = smoothedTorqueCommand;
end

function membership = calcMembership(input, centers, width)
% Gaussian membership function
membership = exp(-((input - centers) / width).^2);
membership = membership / sum(membership); % Normalize
end

function wheelTorques = distributeTorque(Mz, Fz, vehicle, ratio)
% Torque Vectoring Algorithm
bf = vehicle.tf; % Front track width (m)
br = vehicle.tr; % Rear track width (m)

delta_fx = Mz / (bf + br);

fz_front = Fz(1) + Fz(2);
fz_rear  = Fz(3) + Fz(4);

fx_front = delta_fx * fz_front / sum(Fz) * ratio;
fx_rear  = delta_fx * fz_rear / sum(Fz) / ratio;

wheelTorques = zeros(4, 1); % [FL, FR, RL, RR]
wheelTorques(1) = -fx_front*Fz(1) /fz_front / vehicle.GR * vehicle.R;  % FL
wheelTorques(2) = fx_front *Fz(2) /fz_front / vehicle.GR * vehicle.R;  % FR
wheelTorques(3) = -fx_rear *Fz(3) /fz_rear / vehicle.GR * vehicle.R;  % RL
wheelTorques(4) = fx_rear  *Fz(4) /fz_rear / vehicle.GR * vehicle.R;  % RR
end
