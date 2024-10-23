% Define symbolic variables
syms Fx_fl Fx_fr Fx_rl Fx_rr Fy_fl Fy_fr Fy_rl Fy_rr F_Drag delta b v psi_dot
syms m Jz lf lr bf br

% Define the approximations
cos_b_approx = 1;  % cos(b) ≈ 1
sin_b_approx = b;  % sin(b) ≈ b

% Substitute the approximations in the original equations
b_dot = (1/m/v) * ((Fx_fl + Fx_fr) * sin(delta - b) ...
                 - (Fy_fl + Fy_fr) * cos(delta - b) ...
                 + (Fx_rl + Fx_rr) * sin_b_approx ...
                 + (Fy_rl + Fy_rr) * cos_b_approx ...
                 + F_Drag * cos_b_approx - psi_dot);

psi_ddot = (1/Jz) * (lf * ((Fx_fl + Fx_fr) * sin(delta) + (Fy_fl + Fy_fr) * cos(delta)) ...
                    - lr * (Fy_rl + Fy_rr) ...
                    + bf/2 * (-Fx_fl + Fx_fr) * cos(delta) ...
                    - (-Fy_fl + Fy_fr) * sin(delta) ...
                    + br/2 * (Fx_rr - Fx_rl));

% Compute partial derivatives with respect to the specified variables
partial_b_dot_b = diff(b_dot, b);
partial_b_dot_psi_dot = diff(b_dot, psi_dot);
partial_b_dot_Fx_fl = diff(b_dot, Fx_fl);
partial_b_dot_Fx_fr = diff(b_dot, Fx_fr);
partial_b_dot_Fx_rl = diff(b_dot, Fx_rl);
partial_b_dot_Fx_rr = diff(b_dot, Fx_rr);

partial_psi_ddot_b = diff(psi_ddot, b);
partial_psi_ddot_psi_dot = diff(psi_ddot, psi_dot);
partial_psi_ddot_Fx_fl = diff(psi_ddot, Fx_fl);
partial_psi_ddot_Fx_fr = diff(psi_ddot, Fx_fr);
partial_psi_ddot_Fx_rl = diff(psi_ddot, Fx_rl);
partial_psi_ddot_Fx_rr = diff(psi_ddot, Fx_rr);

% Display the results
disp('Partial derivatives of b_dot:');
disp(['d(b_dot)/d(b) = ', char(partial_b_dot_b)]);
disp(['d(b_dot)/d(psi_dot) = ', char(partial_b_dot_psi_dot)]);
disp(['d(b_dot)/d(Fx_fl) = ', char(partial_b_dot_Fx_fl)]);
disp(['d(b_dot)/d(Fx_fr) = ', char(partial_b_dot_Fx_fr)]);
disp(['d(b_dot)/d(Fx_rl) = ', char(partial_b_dot_Fx_rl)]);
disp(['d(b_dot)/d(Fx_rr) = ', char(partial_b_dot_Fx_rr)]);

disp('Partial derivatives of psi_ddot:');
disp(['d(psi_ddot)/d(b) = ', char(partial_psi_ddot_b)]);
disp(['d(psi_ddot)/d(psi_dot) = ', char(partial_psi_ddot_psi_dot)]);
disp(['d(psi_ddot)/d(Fx_fl) = ', char(partial_psi_ddot_Fx_fl)]);
disp(['d(psi_ddot)/d(Fx_fr) = ', char(partial_psi_ddot_Fx_fr)]);
disp(['d(psi_ddot)/d(Fx_rl) = ', char(partial_psi_ddot_Fx_rl)]);
disp(['d(psi_ddot)/d(Fx_rr) = ', char(partial_psi_ddot_Fx_rr)]);


%% Linear System used to design controller.
% After we obtain the linear form of the system arround,
%  b = 0 we can write the system in the dx/dt = Ax + Bu form
m = 270;
v = 15;
bf = 1.22;
br = 1.22;
lf = 0.5*1.57;
lr = 0.5*1.57;
Jz = 75;

delta = 0.1;
Fx_fl = 500;
Fx_fr = 500;
Fx_rl = 500;
Fx_rr = 500;

Fy_fl = 500;
Fy_fr = 500;
Fy_rl = 500;
Fy_rr = 500;
a11 = (Fx_rl + Fx_rr - cos(- delta)*(Fx_fl + Fx_fr) + sin(- delta)*(Fy_fl + Fy_fr))/(m*v);
a12 = -1/(m*v);

% A matrix
A = [a11 a12;
      0   0 ];

% B matrix
B = [-sin(-delta) -sin(-delta) 0 0;
    -((bf*cos(delta))/2 - lf*sin(delta))/Jz ((bf*cos(delta))/2 + lf*sin(delta))/Jz -br/(2*Jz) br/(2*Jz)];

% C matrix
C = [0 1];

%% Expand state space linear model & Design controller
A_ex = [A [0; 0] ; C 0];
B_ex = [B;[0 0 0 0]];

Q = diag([1/(deg2rad(30))^2 1 100]);
R = 1/2000*eye(4);
K = icare(A_ex, B_ex, Q, R);

%% Next step: Construct a delta steering input to parse into the system

%% See the response of the linearized system with controllers designed for that system and tune controllers
% Define velocity and probably calculate ay and ax (from yaw rate and v) to calculate loads on
% wheels. 

%% Use controller on non linear model (i will perhaps have to make a linear model) 

%% Then do fuzzy lol

%% Maybe try some optimal control method?
