function Tmotor = motorTorque(vehicle, input, slipAngleMatrix, wheel_fz_array, wheel_omega_array)
%MOTORTORQUE returns the torque split between the vehicles 4 motors.
Tmotor= zeros(4,1);
for iMotor=1:length(wheel_omega_array)
    maxGripLimitedFx = vehicle.TireMaxFx(slipAngleMatrix(iMotor), wheel_fz_array(iMotor))*0.95;
    maxGripLimitedTorque = maxGripLimitedFx/vehicle.GR*vehicle.R;

    Tmotor(iMotor) = max(min(min(max(vehicle.Motors(wheel_omega_array(iMotor)*vehicle.GR)), (input(iMotor))/vehicle.GR*vehicle.R), maxGripLimitedTorque),0);
end

