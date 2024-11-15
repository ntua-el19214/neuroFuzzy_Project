function status = outputFunction(t, Y, flag, vehicle, delta, ssVectorSA, fxSS, ExpandedMatrices)
    persistent Tmotor_values yaw_error_values time_values
    status = 0; % Required for OutputFcn; 0 means continue integration.

    if isempty(flag) % During integration step
        [~, Tmotor, yaw_error] = nonLinearVehicleModel(t, Y, vehicle, delta, ssVectorSA, fxSS, ExpandedMatrices);
        Tmotor_values = [Tmotor_values; Tmotor(:)']; 
        yaw_error_values = [yaw_error_values; yaw_error]; 
        time_values = [time_values; t]; 
    elseif strcmp(flag, 'init') % Initialization
        Tmotor_values = [];
        yaw_error_values = [];
        time_values = [];
    elseif strcmp(flag, 'done') % Integration complete
        % Save the data to the base workspace for analysis
        assignin('base', 'Tmotor_values', Tmotor_values);
        assignin('base', 'yaw_error_values', yaw_error_values);
        assignin('base', 'time_values', time_values);
    end
end
