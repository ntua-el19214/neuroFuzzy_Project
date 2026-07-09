function [ExpandedMatrices, steerAngleVector] = helpSetupTheProblem(Q, ratio)
%% Linear System used to design controller.
% After we obtain the linear form of the system arround,
% b = 0 we can write the system in the dx/dt = Ax + Bu form

% Vehicle parameters
m = 270;
v = 10;
bf = 1.22;
br = 1.22;
lf = 0.5*1.57;
lr = 0.5*1.57;
Jz = 100;
F_Drag = 0.5*1.225*1.7*v^2; % 1.225 = standard air density, kg/m^3

%% Choose Linearization Points
% Kept for historical provenance: this documents where the 5 operating points'
% magic numbers (below) came from. Not meant to be re-run every execution.
% P20_data = importfile('Post_Season_Testing_P20_Marathonas.csv');
%
% AccelY    = smoothdata(P20_data.ACCEL_Y./9.81, 'gaussian', 5);
% V         = smoothdata(P20_data.GPSSpeedkph/3.6, 'gaussian', 5);
% steering  = smoothdata(P20_data.Steering/3.74, 'gaussian', 5);
% steerSign = smoothdata(P20_data.SteerSign);
% 
% steeringAngleRad = deg2rad(steering.*steerSign);
% 
% vehicleYawRate = V.*steeringAngleRad/(lr + lf);
% 
% h = histogram2(steeringAngleRad, V, [30 30],'FaceColor','flat');

% From the V - Steering histogram we obtain 3 main operating points arround
% which we linearize. Since one is near SA = 0deg, we will linearize around
% 5 total points: zero steering input, high and low steering input and
% their symmetrics.

%%
% Define the data for each operating point
% Operating points were chosen from the previous histogram
% First operating point
vSS1 = (6.66 + 7.4) / 2;
bSS1 = 0;
deltaSS1 = -(0.48 + 0.448) / 2;

% Second operating point
vSS2 = (10.36 + 9.62) / 2;
bSS2 = 0;
deltaSS2 = -(0.192 + 0.16) / 2;

% Third operating point
vSS3 = (11.1 + 11.84) / 2;
bSS3 = 0;
deltaSS3 = 0;

% Fourth operating point
vSS4 = (10.36 + 9.62) / 2;
bSS4 = 0;
deltaSS4 = (0.192 + 0.16) / 2;

% Fifth operating point
vSS5 = (6.66 + 7.4) / 2;
bSS5 = 0;
deltaSS5 = (0.48 + 0.448) / 2;

% Create a table with the operating points
OperatingPointsTable = table(...
    [vSS1; vSS2; vSS3; vSS4; vSS5], ...
    [bSS1; bSS2; bSS3; bSS4; bSS5], ...
    [deltaSS1; deltaSS2; deltaSS3; deltaSS4; deltaSS5], ...
    'VariableNames', {'vSS', 'bSS', 'deltaSS'});

%% Write linear expression to calculate forces

% Initialize a variable to store solutions
solutions = cell(1, 5);
latAcc = 1;
% Loop through each operating point.
% Only i = 3 (the zero-steering / straight-line operating point) is actually
% consumed downstream by the Matrices(i)/ExpandedMatrices(i) construction loops
% below, so this only solves the one design point that matters (avoids 4 wasted
% symbolic solve() calls). The full OperatingPointsTable above is kept intact
% since it's cheap and documents the whole design space.
for i = 3:3
    syms Fx_fl Fx_fr Fx_rl Fx_rr real
    % Extract operating point values
    v = OperatingPointsTable.vSS(i);
    D = OperatingPointsTable.deltaSS(i);
    psi_dot = v * D / (lf + lr);
    if D == 0
        latAcc = 0;
        % Fx_rl = 200;
        % Fx_rr = 200;
    end
    b = D;
    F_Drag = 0.5 * 1.225 * 1.7 * v^2; % 1.225 = standard air density, kg/m^3
    F_Df   = 0.5 * 1.225 * 7.2 * v^2; % 1.225 = standard air density, kg/m^3
    wty    = -sign(D)*latAcc*9.81*m*0.3/((bf+br)/2); % lateral weight transfer

    totalFz = F_Df+m*9.81;
    totalFy = latAcc*9.81*m;

    Fz_fl = totalFz/4 + wty/2;
    Fz_fr = totalFz/4 - wty/2;
    Fz_rl = totalFz/4 + wty/2;
    Fz_rr = totalFz/4 - wty/2;

    Fy_fl  = Fz_fl/totalFz*totalFy;
    Fy_fr  = Fz_fr/totalFz*totalFy;
    Fy_rl  = Fz_rl/totalFz*totalFy;
    Fy_rr  = Fz_rr/totalFz*totalFy;

    % Define the equations
    E(1) = (1/m/v) * ((Fx_fl + Fx_fr) * sin(D - b) ...
                 - (Fy_fl + Fy_fr) * cos(D - b) ...
                 + (Fx_rl + Fx_rr) * sin(b) ...
                 + (Fy_rl + Fy_rr) * cos(b) ...
                 + F_Drag * sin(b) ) - psi_dot == 0;
    
    E(2) = (1 / Jz) * (lf * ((Fx_fl + Fx_fr) * sin(D) + (Fy_fl + Fy_fr) * cos(D)) ...
           - lr * (Fy_rl + Fy_rr) ...
           + bf / 2 * (-Fx_fl + Fx_fr) * cos(D) ...
           - (-Fy_fl + Fy_fr) * sin(D) ...
           + br / 2 * (Fx_rr - Fx_rl)) == 0;

    E(3) = Fx_fl == Fx_rl;
    E(4) = Fx_fr == Fx_rr;

    solution = solve(E, [Fx_fl Fx_fr Fx_rl Fx_rr], 'ReturnConditions', true);
    if D~=0
        solutions{i}.Fx_fl =double(solution.Fx_fl);
        solutions{i}.Fx_fr =double(solution.Fx_fr);
        solutions{i}.Fx_rl =double(solution.Fx_rl);
        solutions{i}.Fx_rr =double(solution.Fx_rr);
    else
        % D == 0 (straight-line) case: the symbolic system is underdetermined here
        % (steady-state yaw equilibrium is trivially satisfied for any equal Fx split),
        % so instead of solving we fall back to a tuned constant approximating the
        % per-wheel longitudinal force needed to overcome straight-line resistance.
        solutions{i}.Fx_fl = 53.6782;
        solutions{i}.Fx_fr = 53.6782;
        solutions{i}.Fx_rl = 53.6782;
        solutions{i}.Fx_rr = 53.6782;
    end

    % Evaluate and display the solution
    % disp(['Operating Point ', num2str(i), ':']);
    evaluatedSolution = structfun(@(x) subs(x), solution, 'UniformOutput', false);
    % disp(evaluatedSolution);
end

% Display the table
% disp(OperatingPointsTable);


%% Set up matrices
for i =3:3
    % Extract operating point values
    v = OperatingPointsTable.vSS(i);
    D = OperatingPointsTable.deltaSS(i);
    b = D; % Assuming b = delta as per previous calculations
    psi_dot = v * D / (lf + lr);
    F_Drag = 0.5 * 1.225 * 2 * v^2; % 1.225 = standard air density, kg/m^3

    % Solve the equations to find the forces
    solution = solutions{i}; % Obtain the solution for this operating point

    % Substitute the solutions into the force expressions
    Fx_fl_val = double(solution.Fx_fl);
    Fx_fr_val = double(solution.Fx_fr);
    Fx_rl_val = double(solution.Fx_rl);
    Fx_rr_val = double(solution.Fx_rr);
   
    Fy_fl_val = double(Fy_fl);
    Fy_fr_val = double(Fy_fr);
    Fy_rl_val = double(Fy_rl);
    Fy_rr_val = double(Fy_rr);
    
    % Calculate a11 using the provided formula
    a11 = -(sin(b)*(Fy_rl_val + Fy_rr_val) - cos(b)*(Fx_rl_val + Fx_rr_val) + cos(b - D)*(Fx_fl_val + Fx_fr_val) - sin(b - D)*(Fy_fl_val + Fy_fr_val) + F_Drag*cos(b))/(m*v);
    a12 = -1;
    
    % Construct the A matrix
    A = [a11 a12;
         0   0 ];
    
    % Construct the B matrix
    B = [-sin(b - D)/(m*v) -sin(b - D)/(m*v) sin(b)/(m*v) sin(b)/(m*v);
         -((bf*cos(D))/2 - lf*sin(D))/Jz ((bf*cos(D))/2 + lf*sin(D))/Jz -br/(2*Jz) br/(2*Jz)];
    
    % Construct the C matrix
    C = [0 1];
    
    % Store the matrices in the structure
    Matrices(i).A = A;
    Matrices(i).B = B;
    Matrices(i).C = C;
end
%% Expand state space linear model & Design controller
% Given symbolic variables for state-space expansion
syms x1 x2 real % Assuming x1 and x2 represent the states

% Initialize a variable to store expanded matrices for each operating point
ExpandedMatrices = struct('A_ex', {}, 'B_ex', {}, 'Kr', {}, 'Ki', {}, 'Kp', {});

% Loop through each operating point to compute expanded matrices and controller gain
for i = 3:3
    % Retrieve the matrices from the previously calculated results
    A = Matrices(i).A;
    B = Matrices(i).B;
    C = Matrices(i).C;

    % Expand A matrix to A_ex
    A_ex = [A [0; 0]; C 0];

    % Expand B matrix to B_ex
    B_ex = [B; [0 0 0 0]];

    % Define the weights for the cost function
    Q_mat = diag(Q); % Adjust weights as necessary
    % R matrix in J = integral( x'Qx + u'Ru ) dt -- control-effort weighting for the LQR design
    R = 0.001 * eye(4); % Adjust R matrix size if needed

    % Calculate the controller gain using icare
    [~, K, ~] = icare(A_ex, B_ex, Q_mat, R);
    % actK = -K;
    % K = actK;
    frontToRearRatio = ones(4,3);
    frontToRearRatio(1:2, :) = frontToRearRatio(1:2, :).*ratio;
    frontToRearRatio(3:4, :) = frontToRearRatio(3:4, :)./(ratio);
    K = K.*frontToRearRatio;
    % Fraction of the proportional (Kp) gain shaved off before it's stored in Kr's
    % second column below -- a tuned damping factor for the yaw-rate feedback term.
    pScaleFactor = 0.70;
    % Store the expanded matrices and controller gain in the structure
    ExpandedMatrices(i).A_ex = A_ex;
    ExpandedMatrices(i).B_ex = B_ex;
    ExpandedMatrices(i).Kr = [K(:,1) K(:,2)*(1-pScaleFactor)];
    ExpandedMatrices(i).Ki = K(:,3);
    ExpandedMatrices(i).Kp = K(:,2);


    % Display the results for this operating point
    % disp(['Operating Point ', num2str(i), ':']);
    % disp('A_ex matrix:');
    % disp(A_ex);
    % disp('B_ex matrix:');
    % disp(B_ex);
    % disp('Controller Gain K:');
    % disp(K);
end

ExpandedMatrices = ExpandedMatrices(3);
% steerAngleVector = [deltaSS1 deltaSS2 deltaSS3 deltaSS4 deltaSS5];
steerAngleVector = deltaSS3;
%% Construct gain scheduling algorith
% This was a test for the algorithm
% gainStruct = gainScheduling(delta, [OperatingPointsTable.deltaSS(:)], ExpandedMatrices);

%% Next step: Construct a delta steering input to parse into the system

%% Use controller on non linear model (i will perhaps have to make a linear model)


