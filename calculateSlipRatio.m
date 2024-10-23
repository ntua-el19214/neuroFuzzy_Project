function slipRR = calculateSlipRatio(omega, wheelV, Reff)
% This function calculates wheel slip ratio based on
% the angular velocity (omega), linear speed (wheelV), and tire
% effective radius (Reff).

% Calculate RR slip ratio
if abs(omega * Reff) < 0.3
    slipRR = 2 * (omega * Reff - wheelV) / (0.3 + wheelV^2 / 0.3);
else
    slipRR = (omega * Reff - wheelV) / abs(wheelV);
end
end
