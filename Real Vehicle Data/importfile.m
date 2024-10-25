function PostSeasonTestingP20Marathonas = importfile(filename, dataLines)
%IMPORTFILE Import data from a text file
%  POSTSEASONTESTINGP20MARATHONAS = IMPORTFILE(FILENAME) reads data from
%  text file FILENAME for the default selection.  Returns the data as a
%  table.
%
%  POSTSEASONTESTINGP20MARATHONAS = IMPORTFILE(FILE, DATALINES) reads
%  data for the specified row interval(s) of text file FILENAME. Specify
%  DATALINES as a positive scalar integer or a N-by-2 array of positive
%  scalar integers for dis-contiguous row intervals.
%
%  Example:
%  PostSeasonTestingP20Marathonas = importfile("C:\Uni\Software Engineering\neuroFuzzy_Project\Real Vehicle Data\Post_Season_Testing_P20_Marathonas.csv", [2, Inf]);
%
%  See also READTABLE.
%
% Auto-generated by MATLAB on 23-Oct-2024 20:04:44

%% Input handling

% If dataLines is not specified, define defaults
if nargin < 2
    dataLines = [2, Inf];
end

%% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 142);

% Specify range and delimiter
opts.DataLines = dataLines;
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["Times", "Distancem", "ACCEL_X", "ACCEL_Y", "ACCEL_Z", "GYRO_X", "GYRO_Y", "GYRO_Z", "ROLL", "PITCH", "YAW", "GPS_IMU_LAT", "GPS_IMU_LONG", "VELOCITY_X", "VELOCITY_Y", "VELOCITY_Z", "VEL_N", "VEL_E", "VEL_D", "GNSS_LAT", "GNSS_LONG", "ANGLE_TRACK", "ANGLE_SLIP", "CURVA_RADIUS", "STR_RR_DifV", "STRAIN_RR", "STR_RL_DifV", "STRAIN_RL", "STR_FR_DifV", "STRAIN_FR", "STRAIN_FL", "FL_TEMP_I", "FL_TEMP_CI", "FL_TEMP_CO", "FL_TEMP_0", "FR_TEMP_0", "FR_TEMP_CO", "FR_TEMP_CI", "FR_TEMP_I", "RR_TEMP_0", "RR_TEMP_CO", "RR_TEMP_CI", "RR_TEMP_I", "RL_TEMP_I", "RL_TEMP_CI", "RL_TEMP_CO", "RL_TEMP_O", "FL_LIN_PO_I", "FL_LIN_PO_D", "FLLINEARSP", "FL_RPM", "FR_LIN_PO_I", "FR_LIN_PO_D", "FR_LINEAR_SP", "FR_RPM", "RL_LIN_PO_I", "RL_LIN_PO_D", "RLLINEARSP", "RL_RPM", "RR_LIN_PO_I", "RR_LIN_PO_D", "RR_LINEAR_SP", "RR_RPM", "CUR_IVT", "Voltage_sens", "MIN_CELL_V", "MAX_CELL_V", "Vlsb", "Vmsb", "I", "BMS_status1", "BMS_status2", "MAX_CELL_TC", "APP1", "APP2", "Brake_pr21", "Brake_pr22", "Brake_pr11", "Brake_pr12", "Steering", "SteerSign", "MAP", "Ecu_errors1", "Request_tor", "Iq_actual", "State_of_ch", "cool_flags", "IGBT_tempC", "Motor_tempC", "Dashboard_1", "page_control", "RPM", "Energy", "Regen", "Voltage", "LOG_BUTTON", "ODOkm", "LAP", "LAPTs", "LAPDistm", "bLAPTs", "ACCLongg", "ACCLatg", "ACCCombg", "LOGTimems", "POWERhp", "TORQUENm", "GPSSpeedkph", "GPSLongdeg", "GPSLatdeg", "Bearingdeg", "uFLAGS1", "prevLAPTs", "POWER", "volt", "BrRear", "BrFront", "cell_under_V", "NO_cell_com", "cell_overh", "inv_enable", "Apps_implaus", "Brake_impl", "Apps_dev", "Brake_dev", "soft_BSPD", "dev_disable", "s_bspd_dis", "motor_t_high", "dev_t_high", "motor_t_lim", "dev_t_lim", "AMS_Status", "IMD_Status", "BSPD_Status", "Air_closed", "Air_closed1", "AA", "R2D", "Regen_Br", "DRS_ON", "TS_ON"];
opts.VariableTypes = ["datetime", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Specify variable properties
opts = setvaropts(opts, "Times", "InputFormat", "HH:mm:ss", "DatetimeFormat", "preserveinput");

% Import the data
PostSeasonTestingP20Marathonas = readtable(filename, opts);

end