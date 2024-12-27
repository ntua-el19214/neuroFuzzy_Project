function [Mz, torqueCommand] = fuzzyTorqueVectoring(psi_dot_err, psi_ddot_err, Fz, vehicle, slipAngleMatrix, wheel_omega_array, tSS, ratio)
    % Parameters
    maxMz = 14; % Maximum yaw moment (Nm)
    
    % Define fuzzy membership functions for psi_dot_err
    psiDotErrMFs = [-1.5, -1.0, -0.5, 0, 0.5, 1.0, 1.5]; % Centers for NB to PB
    psiDotErrWidth = 0.3; % Width of each MF
    
    % Define fuzzy membership functions for psi_ddot_err
    psiDDotErrMFs = [-1.5, -1.0, -0.5, 0, 0.5, 1.0, 1.5]; % Centers for NB to PB
    psiDDotErrWidth = 0.3; % Width of each MF
    
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
    torqueCommand = distributeTorque(Mz, Fz, vehicle, ratio);
    for iMotor=1:length(wheel_omega_array)
        maxAvailableMotorTorque = vehicle.Motors(wheel_omega_array(iMotor)*vehicle.GR);
        % maxGripLimitedTorque  = vehicle.TireMaxFx(slipAngleMatrix(iMotor), Fz(iMotor))/vehicle.GR*vehicle.R*0.3;
        % torqueCommand(iMotor) = min(max(min(torqueCommand(iMotor)+ tSS(iMotor), maxAvailableMotorTorque),0), maxGripLimitedTorque);

        torqueCommand(iMotor) = max(min(torqueCommand(iMotor)+ tSS(iMotor), maxAvailableMotorTorque),0);
    end
    % disp("psi_dot_err is: "+  string(psi_dot_err))
    % disp("psi_ddot_err is: "+ string(psi_ddot_err))
    % disp("Calculate Mz is: "+ string(Mz))
    % disp("Resulting Wheel Torques are: "+ string(wheelTorques))
end
    

function membership = calcMembership(input, centers, width)
    % Gaussian membership function
    membership = exp(-((input - centers) / width).^2);
    membership = membership / sum(membership); % Normalize
end

function wheelTorques = distributeTorque(Mz, Fz, vehicle,ratio)
    % Torque Vectoring Algorithm
    % Mz = lf * (F_fr - F_fl) + lr * (F_rr - F_rl)
    bf = vehicle.tf; % Front track width (m)
    br = vehicle.tr; % Rear track width (m)
    
    delta_fx = Mz/(bf+br);

    fz_front = Fz(1) + Fz(2);
    fz_rear  = Fz(3) + Fz(4);

    fx_front = delta_fx*fz_front/sum(Fz)*ratio;
    fx_rear  = delta_fx*fz_rear/sum(Fz)/ratio;

    wheelTorques = zeros(4, 1); % [FL, FR, RL, RR]
    wheelTorques(1) = -fx_front/2*vehicle.GR*vehicle.R;  % FL
    wheelTorques(2) =  fx_front/2*vehicle.GR*vehicle.R;  % FR
    wheelTorques(3) = -fx_rear/2*vehicle.GR*vehicle.R;  % RL
    wheelTorques(4) =  fx_rear/2*vehicle.GR*vehicle.R;  % RR
end
