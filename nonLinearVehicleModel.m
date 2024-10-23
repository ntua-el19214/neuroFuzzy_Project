function x_dot = nonLinearVehicleModel(t, Y, vehicle, delta)
%NONLINEARVEHICLEMODEL describes the non linear equations that dictate
%vehicle dynamics
v        = Y(1);    
b        = Y(2);   
psi_dot  = Y(3);
accel    = Y(4);
b_dot    = Y(5);
omega_FL = Y(6);   
omega_FR = Y(7);   
omega_RL = Y(8);   
omega_RR = Y(9);   

vx = v*cos(b);
vy = v*sin(b);

ay = v*b_dot + psi_dot*v;
ax = sqrt(accel^2 - ay^2);
lf = vehicle.wb*(1-vehicle.wd);
lr = vehicle.wb*(vehicle.wd);
bf = vehicle.tf;
br = vehicle.tr;
%% Calculate Fz
Fz_fl = vehicle.m*g*vehicle.wd/2 + vehicle.CoGz*vehicle.m*ay/2/vehicle.trackFront - vehicle.CoGz*vehicle.m*ax/2/vehicle.wb     + 1/2*1/4*1.22*5.7*vx^2;
Fz_fr = vehicle.m*g*vehicle.wd/2 - vehicle.CoGz*vehicle.m*ay/2/vehicle.trackFront - vehicle.CoGz*vehicle.m*ax/2/vehicle.wb     + 1/2*1/4*1.22*5.7*vx^2;
Fz_rl = vehicle.m*g*(1-vehicle.wd)/2 + vehicle.CoGz*vehicle.m*ay/2/vehicle.trackFront + vehicle.CoGz*vehicle.m*ax/2/vehicle.wb + 1/2*1/4*1.22*5.7*vx^2;
Fz_rr = vehicle.m*g*(1-vehicle.wd)/2 - vehicle.CoGz*vehicle.m*ay/2/vehicle.trackFront + vehicle.CoGz*vehicle.m*ax/2/vehicle.wb + 1/2*1/4*1.22*5.7*vx^2;

%% Calculate Slip Angles & Slip Ratios
% Wheel speeds
speedFL = (vx - psi_dot*br/2*cos(delta) + (vy + psi_dot*lf)*sin(delta));
speedFR = (vx + psi_dot*br/2*cos(delta) + (vy + psi_dot*lf)*sin(delta));
speedRL = (vx - psi_dot*br/2);
speedRR = (vx + psi_dot*bf/2);

% Calculate slip ratios
slip_FL = calculateSlipRatio(omega_FL, speedFL, vehicle.R);
slip_FR = calculateSlipRatio(omega_FR, speedFR, vehicle.R);
slip_RL = calculateSlipRatio(omega_RL, speedRL, vehicle.R);
slip_RR = calculateSlipRatio(omega_RR, speedRR, vehicle.R);

% Calculate slip angles
slipAngle_FR = atan((vy + lf * yawRate)/(vx + bf/2*yawRate)) - delta;
slipAngle_FL = atan((vy + lf * yawRate)/(vx - bf/2*yawRate)) - delta;
slipAngle_RR = atan((vy - lr * yawRate)/(vx + br/2*yawRate));
slipAngle_RL = atan((vy - lr * yawRate)/(vx - br/2*yawRate));

%% Calculate tire forces
Fx_fl = F_longit(slipAngle_FR, slip_FR, Fz_fl, deg2rad(-0.5));
Fx_fr = F_longit(slipAngle_FL, slip_FL, Fz_fr, deg2rad(-0.5));
Fx_rl = F_longit(slipAngle_RR, slip_RR, Fz_rl, deg2rad(-0.5));
Fx_rr = F_longit(slipAngle_RL, slip_RL, Fz_rr, deg2rad(-0.5));


Fy_fl = F_lateral(slipAngle_FR, slip_FR, Fz_fl, deg2rad(-0.5));
Fy_fr = F_lateral(slipAngle_FL, slip_FL, Fz_fr, deg2rad(-0.5));
Fy_rl = F_lateral(slipAngle_RR, slip_RR, Fz_rl, deg2rad(-0.5));
Fy_rr = F_lateral(slipAngle_RL, slip_RL, Fz_rr, deg2rad(-0.5));
%% Equations
% 
v_dot = 1/m * ((Fx_fl + Fx_fr) * cos(delta - b) ...
                 - (Fy_fl + Fy_fr) * sin(delta - b) ...
                 + (Fx_rl + Fx_rr) * cos(b) ...
                 + (Fy_rl + Fy_rr) * sin(b) ...
                 + F_Drag * cos(b));

% β_dot
beta_dot = (1/m/v) * ((Fx_fl + Fx_fr) * sin(delta - b) ...
                 - (Fy_fl + Fy_fr) * cos(delta - b) ...
                 + (Fx_rl + Fx_rr) * sin(b) ...
                 + (Fy_rl + Fy_rr) * cos(b) ...
                 + F_Drag * sin(b) ) - psi_dot;

% ψ_ddot
psi_ddot = (1/Jz) * (lf * ((Fx_fl + Fx_fr) * sin(delta) + (Fy_fl + Fy_fr) * cos(delta)) ...
                    - lr * (Fy_rl + Fy_rr) ...
                    + bf/2 * (-Fx_fl + Fx_fr) * cos(delta) ...
                    - (-Fy_fl + Fy_fr) * sin(delta) ...
                    + br/2 * (Fx_rr - Fx_rl));
% Wheel accelerations
omega_FL_dot = (Tmotor(1) - Fx_fl*vehicle.Rt)/vehicle.Jw;
omega_FR_dot = (Tmotor(2) - Fx_fr*vehicle.Rt)/vehicle.Jw;
omega_RL_dot = (Tmotor(3) - Fx_rl*vehicle.Rt)/vehicle.Jw;
omega_RR_dot = (Tmotor(4) - Fx_rr*vehicle.Rt)/vehicle.Jw;

x_dot = [accel       ;
         b_dot       ;
         psi_ddot    ;
         v_dot       ;
         beta_dot    ;
         omega_FL_dot;
         omega_FR_dot;
         omega_RL_dot;
         omega_RR_dot;
         ];
end

