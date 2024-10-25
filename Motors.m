function TorqueCurve = Motors(fileName)
% Define sheet name to import
torqueSheet = 'Shaft_Torque'; 
speedSheet = 'Speed';

% Create the range string for the specified column
range = 'U1:U201';

% Import the data from the specified column and range
torque = readtable(fileName, 'Sheet', torqueSheet, 'Range', range, 'ReadVariableNames', false);
speed  = readtable(fileName, 'Sheet', speedSheet, 'Range', range, 'ReadVariableNames', false);

% Convert the imported table data to arrays
torque = table2array(torque);
speed = table2array(speed)/0.10472;

% Fit a curve to the torque and speed data
fitType = 'poly2'; 
[TorqueCurve, ~] = fit(speed, torque, fitType);
end

