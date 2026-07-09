% Visualizes the fuzzy torque-vectoring controller's membership functions.
% Pulls the parameters from Vehicle/fuzzyMFParams.m and evaluates them
% through Vehicle/calcMembership.m exactly as fuzzyTorqueVectoring.m does
% at runtime (scalar error value in, vector of 7 MF centers out, degrees
% normalized to sum to 1), so this plot always matches what the live
% controller actually uses.
addpath(genpath('./'))

mf = fuzzyMFParams();

x_psi_dot_err = linspace(-2, 2, 500);
x_psi_ddot_err = linspace(-14, 14, 500);

% Evaluate membership degrees at each sample point, one call per point
% (mirrors the scalar-error call signature used inside the controller).
psiDotMembership = zeros(length(x_psi_dot_err), length(mf.psiDotErrMFs));
for k = 1:length(x_psi_dot_err)
    psiDotMembership(k, :) = calcMembership(x_psi_dot_err(k), mf.psiDotErrMFs, mf.psiDotErrWidth);
end

psiDDotMembership = zeros(length(x_psi_ddot_err), length(mf.psiDDotErrMFs));
for k = 1:length(x_psi_ddot_err)
    psiDDotMembership(k, :) = calcMembership(x_psi_ddot_err(k), mf.psiDDotErrMFs, mf.psiDDotErrWidth);
end

% Plot psi_dot_err membership functions
figure;
hold on;
for i = 1:length(mf.psiDotErrMFs)
    plot(x_psi_dot_err, psiDotMembership(:, i), 'DisplayName', sprintf('Center %g', mf.psiDotErrMFs(i)));
end
title('Membership Functions for \psi\_dot\_err');
xlabel('\psi\_dot\_err');
ylabel('Membership Value');
legend show;
grid on;
hold off;

% Plot psi_ddot_err membership functions
figure;
hold on;
for i = 1:length(mf.psiDDotErrMFs)
    plot(x_psi_ddot_err, psiDDotMembership(:, i), 'DisplayName', sprintf('Center %g', mf.psiDDotErrMFs(i)));
end
title('Membership Functions for \psi\_ddot\_err');
xlabel('\psi\_ddot\_err');
ylabel('Membership Value');
legend show;
grid on;
hold off;
