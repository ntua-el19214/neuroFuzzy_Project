function [t, Y, ax, ay, torques] = RKESys(a, b, N, F, delta, Y0, A, bhta, tau)
rng('default');
rng(2024);

t = linspace(a, b, N+1);
h = (b-a)/N;
M = length(Y0);
q = length(tau);

Y = zeros(M, N+1);
Y(:,1) = Y0;
ax = zeros(1, N+1);
ay = zeros(1, N+1);

% Initialize torque matrix
torques = zeros(4, N+1); % Assuming 4 motor torques (front-left, front-right, rear-left, rear-right)

% Initialize the progress bar
h_wait = waitbar(0, 'Solving ODE...');

for n = 1:N
    % Update the progress bar
    waitbar(n / N, h_wait, sprintf('Progress: %.2f%%', 100 * n / N));

    tn = zeros(q,1);
    Kn = zeros(M, q);
    Tmotor_n = zeros(4, q); % Temporary storage for torques

    for i = 1:q
        tn(i) = t(n) + tau(i) * h;
        sm1 = zeros(M, 1);

        for j = 1:q
            sm1 = sm1 + h * A(i,j) * Kn(:,j);
        end

        [Kn(:,i), Tmotor] = F(tn(i), Y(:,n) + sm1, delta(n), ax(n), ay(n));
        Tmotor_n(:,i) = Tmotor; % Store torques at each sub-step
    end

    sm2 = zeros(M, 1);
    for i = 1:q
        sm2 = sm2 + h * bhta(i) * Kn(:,i);
    end

    Y(:,n+1) = Y(:,n) + sm2;

    % Store the average torque for this step
    torques(:,n+1) = mean(Tmotor_n, 2);

    % Calculate accelerations
    ax(n+1) = (Y(2,n+1) - Y(2,n)) / h;
    ay(n+1) = (Y(3,n+1) - Y(3,n)) / h;
end

% Close the progress bar
close(h_wait);
end
