function [x_dot, aux]= nonLinearVehicleModel(t, Y, deltaValues, vehicle, ssVectorSA, fxSS ,ExpandedMatrices, mode,ratio)
% NONLINEARVEHICLEMODEL describes the non-linear equations that dictate vehicle dynamics

% Initialize integral term for yaw control
persistent ax ay
if isempty(ax)
    ax = 0;
end
if isempty(ay)
    ay = 0;
end

g = 9.81; % Gravitational acceleration

% Extract state variables
psi_dot  = Y(1);  % Yaw rate
vx       = Y(2);  % Longitudinal velocity
vy       = Y(3);  % Lateral velocity
omega_FL = Y(4);  % Wheel angular velocity (front left)
omega_FR = Y(5);  % Wheel angular velocity (front right)
omega_RL = Y(6);  % Wheel angular velocity (rear left)
omega_RR = Y(7);  % Wheel angular velocity (rear right)
i_error  = Y(8);  % integral error
disp_x   = Y(9);
disp_y   = Y(10);
psi      = Y(11);

b = atan(vy/vx);

% Extract steering derivatives and 
[delta, delta_dot] = deltaValues(t);

% Vehicle geometry and load distribution
lf = vehicle.wb * (1 - vehicle.wd);
lr = vehicle.wb * vehicle.wd;
bf = vehicle.tf;
br = vehicle.tr;

% Calculate normal forces (Fz)
Fz_fl = vehicle.m * g * vehicle.wd / 2 + (vehicle.CoGz * vehicle.m * ay / (2 * vehicle.tf)) - ...
        (vehicle.CoGz * vehicle.m * ax / (2 * vehicle.wb)) + 1/2 * 1/4 * 1.22 * vehicle.cl * vx^2;
Fz_fr = vehicle.m * g * vehicle.wd / 2 - (vehicle.CoGz * vehicle.m * ay / (2 * vehicle.tf)) - ...
        (vehicle.CoGz * vehicle.m * ax / (2 * vehicle.wb)) + 1/2 * 1/4 * 1.22 * vehicle.cl * vx^2;
Fz_rl = vehicle.m * g * (1 - vehicle.wd) / 2 + (vehicle.CoGz * vehicle.m * ay / (2 * vehicle.tr)) + ...
        (vehicle.CoGz * vehicle.m * ax / (2 * vehicle.wb)) + 1/2 * 1/4 * 1.22 * vehicle.cl * vx^2;
Fz_rr = vehicle.m * g * (1 - vehicle.wd) / 2 - (vehicle.CoGz * vehicle.m * ay / (2 * vehicle.tr)) + ...
        (vehicle.CoGz * vehicle.m * ax / (2 * vehicle.wb)) + 1/2 * 1/4 * 1.22 * vehicle.cl * vx^2;

% Calculate slip ratios and slip angles
[slip_FL,slip_FR,slip_RL,slip_RR] = calculateSlipRatio(omega_FL, omega_FR, omega_RL, omega_RR, psi_dot,vx,vehicle);

slipAngle_FR = atan((vy + psi_dot*lf)/(vx + 0.5*psi_dot*bf)) - delta;
slipAngle_FL = atan((vy + psi_dot*lf)/(vx - 0.5*psi_dot*bf)) - delta;
slipAngle_RR = atan((vy - psi_dot*lr)/(vx + 0.5*psi_dot*br));
slipAngle_RL = atan((vy - psi_dot*lr)/(vx - 0.5*psi_dot*br));

% Calculate tire forces
Fx_fl = F_longit(slipAngle_FL, slip_FL, Fz_fl, deg2rad(0));
Fx_fr = F_longit(slipAngle_FR, slip_FR, Fz_fr, deg2rad(0));
Fx_rl = F_longit(slipAngle_RL, slip_RL, Fz_rl, deg2rad(0));
Fx_rr = F_longit(slipAngle_RR, slip_RR, Fz_rr, deg2rad(0));

Fy_fl = F_lateral(slipAngle_FL, slip_FL, Fz_fl, deg2rad(0));
Fy_fr = F_lateral(slipAngle_FR, slip_FR, Fz_fr, deg2rad(0));
Fy_rl = F_lateral(slipAngle_RL, slip_RL, Fz_rl, deg2rad(0));
Fy_rr = F_lateral(slipAngle_RR, slip_RR, Fz_rr, deg2rad(0));

% Update the acceleration and beta_dot in the state
vx_dot = 1 / vehicle.m * ((Fx_fl + Fx_fr)*cos(delta) + Fx_rl + Fx_rr...
                           -(Fy_fl + Fy_fr)*sin(delta)  ...
                           - 1/2 * 1.224 * vehicle.cd * vx^2) + psi_dot*vy;
                            
ax = vx_dot;
vy_dot = 1 / vehicle.m * ((Fx_fl + Fx_fr)*sin(delta) ...
                           +(Fy_fl + Fy_fr)*cos(delta) + Fy_rl + Fy_rr)...
                           - psi_dot*vx;
ay = vy_dot;

% ψ_ddot (yaw acceleration)
psi_ddot = (1 / vehicle.Jz) * (lf * ((Fx_fl + Fx_fr) * sin(delta) + (Fy_fl + Fy_fr) * cos(delta)) - ...
                               lr * (Fy_rl + Fy_rr) + bf / 2 * (-Fx_fl + Fx_fr) * cos(delta) - ...
                               (-Fy_fl + Fy_fr) * sin(delta) + br / 2 * (Fx_rr - Fx_rl));

accel = sqrt(vx_dot^2 +vy_dot^2);

% Calculate motor torque and wheel accelerations
psi_ddot_desired = 1/vehicle.wb*(accel*delta + sqrt(vx^2 + vy^2)*delta_dot);
psi_dot_desired  = sqrt(vx^2 + vy^2) * delta / (lf + lr);
yaw_error        = psi_dot_desired - psi_dot;
psi_ddot_err     = psi_ddot_desired - psi_ddot;

% Important controller parameters
slipAngleMatrix = [slipAngle_FL, slipAngle_FR, slipAngle_RL, slipAngle_RR];
fzMatrix = [Fz_fl, Fz_fr, Fz_rl, Fz_rr];
omegaMatrix = [omega_FL, omega_FR, omega_RL, omega_RR];
switch mode 
    case "stateSpace"
        gainStruct = gainScheduling(delta, ssVectorSA, ExpandedMatrices);
        input = yaw_error * gainStruct.Kp + i_error * gainStruct.Ki - gainStruct.Kr * [b; psi_dot] +fxSS;
        Tmotor = motorTorque(vehicle, input, slipAngleMatrix, fzMatrix, omegaMatrix);
    case "fuzzy"
        [Mz, Tmotor]= fuzzyTorqueVectoring(yaw_error, psi_ddot_err, fzMatrix, vehicle, slipAngleMatrix, omegaMatrix, fxSS/vehicle.GR*vehicle.R, ratio);

    case "openLoop"
        Tmotor = 200/vehicle.GR*vehicle.R*ones(1,4);
        Tmotor = Tmotor.*[0.4 0.4 1 1];

end

omega_FL_dot = (Tmotor(1)*vehicle.GR - Fx_fl * vehicle.R - 0.03*Fz_fl) / vehicle.Jw;
omega_FR_dot = (Tmotor(2)*vehicle.GR - Fx_fr * vehicle.R - 0.03*Fz_fr) / vehicle.Jw;
omega_RL_dot = (Tmotor(3)*vehicle.GR - Fx_rl * vehicle.R - 0.03*Fz_rl) / vehicle.Jw;
omega_RR_dot = (Tmotor(4)*vehicle.GR - Fx_rr * vehicle.R - 0.03*Fz_rr) / vehicle.Jw;

% Return the state derivatives
x_dot = [psi_ddot;   % Change in yaw rate
         vx_dot;     % Change in velocity
         vy_dot;
         omega_FL_dot;
         omega_FR_dot;
         omega_RL_dot;
         omega_RR_dot;
         yaw_error;
         vx;
         vy;
         psi_dot];


aux = struct();
aux.slipAngles = struct('FL', slipAngle_FL, 'FR', slipAngle_FR, 'RL', slipAngle_RL, 'RR', slipAngle_RR);
aux.slipRatios = struct('FL', slip_FL, 'FR', slip_FR, 'RL', slip_RL, 'RR', slip_RR);
aux.Forces = struct('Fx', struct('FL', Fx_fl, 'FR', Fx_fr, 'RL', Fx_rl, 'RR', Fx_rr), ...
                    'Fy', struct('FL', Fy_fl, 'FR', Fy_fr, 'RL', Fy_rl, 'RR', Fy_rr), ...
                    'Fz', struct('FL', Fz_fl, 'FR', Fz_fr, 'RL', Fz_rl, 'RR', Fz_rr));
aux.MotorTorques = struct('FL', Tmotor(1), 'FR', Tmotor(2), 'RL', Tmotor(3), 'RR', Tmotor(4));
aux.yawError 	 = yaw_error;
aux.b            = b;
aux.psiDotTarg   = psi_dot_desired;

end
