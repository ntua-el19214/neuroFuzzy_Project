function f = maxFxForSaFzCombination()
%MAXFXPERFZ This script calulates a fitting matching tire vertical loads & slipAngles,
%to potential tire maximum longitudinal forces fx

%slipRatios = deg2rad(-20:0.05:20);
slipAngles = deg2rad(-20:0.05:20);
vertForce  = 150:25:450*4.4482216152605;

forceMatrix = zeros([length(slipAngles) length(vertForce)]);
for iSlipAngle= 1:length(slipAngles)
    for iVerForce= 1:length(vertForce)
        objective = @(x) -F_longit(slipAngles(iSlipAngle), x, vertForce(iVerForce), deg2rad(-0.5));
        x0 = 0;
        ub = deg2rad(20);
        lb = -ub;
        [~, MaxFroce] = fmincon(objective, x0, [], [], [], [], lb, ub, []);
        forceMatrix(iSlipAngle, iVerForce) = -MaxFroce;
    end
end
% Flatten data for lsqcurvefit
%%
[YData, XData] = meshgrid(vertForce', slipAngles);
ZData = forceMatrix(:);       % Flatten the MaxForce (forceMatrix) data to a vector
f = fit([XData(:), YData(:)],ZData,'linearinterp');

