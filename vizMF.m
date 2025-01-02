    % Define fuzzy membership functions for psi_dot_err
    psiDotErrMFs = [-0.65, -0.4, -0.2, 0, 0.2, 0.4, 0.65]; % Centers for NB to PB
    psiDotErrWidth  = 0.09; % Width of each MF
    
    % Define fuzzy membership functions for psi_ddot_err
    psiDDotErrMFs = [-8, -5.5, -3, 0, 3, 5.5, 8]; % Centers for NB to PB
    psiDDotErrWidth  = 0.8; % Width of each MF
x_psi_dot_err = linspace(-2, 2, 500);
x_psi_ddot_err = linspace(-14, 14, 500);

% Gaussian Membership Function
gaussianMembership = @(x, c, w) exp(-((x - c) / w).^2);

% Plot psi_dot_err membership functions
figure;
hold on;
for i = 1:length(psiDotErrMFs)
    y = gaussianMembership(x_psi_dot_err, psiDotErrMFs(i), psiDotErrWidth);
    plot(x_psi_dot_err, y, 'DisplayName', sprintf('Center %d', psiDotErrMFs(i)));
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
for i = 1:length(psiDDotErrMFs)
    y = gaussianMembership(x_psi_ddot_err, psiDDotErrMFs(i), psiDDotErrWidth);
    plot(x_psi_ddot_err, y, 'DisplayName', sprintf('Center %d', psiDDotErrMFs(i)));
end
title('Membership Functions for \psi\_ddot\_err');
xlabel('\psi\_ddot\_err');
ylabel('Membership Value');
legend show;
grid on;
hold off;
