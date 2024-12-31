function status = vehicleOutputFcn(t, Y, flag, delta, vehicle, ssVectorSA, fxSS, ExpandedMatrices, tspan, hWaitBar, mode,ratio, rampRate)
persistent logData progress;

switch flag
    case 'init'
        progress = 0; % Initialize progress
        logData.time = [];
        logData.aux = [];
    case ''
        % Update the wait bar progress
        progress = (t - tspan(1)) / (tspan(end) - tspan(1));
        waitbar(progress, hWaitBar, sprintf('Progress: %.2f%%', progress * 100));
        % Evaluate the vehicle model to get the auxiliary data
        [~, aux] = nonLinearVehicleModel(t, Y, delta, vehicle, ssVectorSA, fxSS, ExpandedMatrices, mode,ratio, rampRate);

        % Append time and auxiliary data
        logData.time = [logData.time; t];
        logData.aux = [logData.aux; aux];

    case 'done'
        % Close the wait bar
        close(hWaitBar);
        % Save logData to the base workspace at the end of the simulation
        assignin('base', 'logData', logData);
        logData = [];
end

status = 0;
end
