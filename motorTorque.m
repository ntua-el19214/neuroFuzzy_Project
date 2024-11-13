function Tmotor = motorTorque(vehicle, input, slipAngleMatrix, wheel_fz_array, wheel_omega_array)
%MOTORTORQUE returns the torque split between the vehicles 4 motors.
maxGripLimitedFx = zeros(1,length(wheel_omega_array));

torqueRatio = (input(1)+input(3))/(input(2) + input(4));
for iMotor = 1:length(wheel_omega_array)
    maxFx = max(vehicle.Motors(wheel_omega_array(iMotor)*vehicle.GR))*vehicle.GR/vehicle.R;
    maxGripLimitedFx(iMotor) = min(maxFx, vehicle.TireMaxFx(slipAngleMatrix(iMotor), wheel_fz_array(iMotor)));
end
gripRatio = (maxGripLimitedFx(1)+maxGripLimitedFx(3))/(maxGripLimitedFx(2) + maxGripLimitedFx(4));

if gripRatio < torqueRatio % the grip limit is on the left side
    % we always want to make sure we calculate the force split with the
    % side that has less grip compared to the torque split requested, for
    % example: If the requested split is 2:1 and available grip is
    % 1.75:1.15, then we apply all 1.75 torque that is available to the
    % left track and apply 1.75/2 to the right track
    Tmotor(1) = maxGripLimitedFx(1)*input(1)/(input(1)+input(3))/vehicle.GR*vehicle.R;
    Tmotor(3) = maxGripLimitedFx(3)*input(3)/(input(1)+input(3))/vehicle.GR*vehicle.R;
    
    totalFxRight = (maxGripLimitedFx(1)*input(1)/(input(1)+input(3)) + maxGripLimitedFx(3)*input(3)/(input(1)+input(3)))/torqueRatio;
    
    
    Tmotor(2) = totalFxRight*input(2)/(input(2)+input(4))/vehicle.GR*vehicle.R;
    Tmotor(4) = totalFxRight*input(4)/(input(2)+input(4))/vehicle.GR*vehicle.R;

else % the grip limit is on the right side
    Tmotor(2) = maxGripLimitedFx(2)*input(2)/(input(2)+input(4))/vehicle.GR*vehicle.R;
    Tmotor(4) = maxGripLimitedFx(4)*input(4)/(input(2)+input(4))/vehicle.GR*vehicle.R;
    
    totalFxLeft = (maxGripLimitedFx(2)*input(2)/(input(2)+input(4)) + maxGripLimitedFx(4)*input(4)/(input(2)+input(4)))*torqueRatio;
    
    
    Tmotor(1) = totalFxLeft*input(1)/(input(1)+input(3))/vehicle.GR*vehicle.R;
    Tmotor(3) = totalFxLeft*input(3)/(input(1)+input(3))/vehicle.GR*vehicle.R;

end

