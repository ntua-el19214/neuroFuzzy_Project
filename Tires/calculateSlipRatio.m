function [SR_FL,SR_FR,SR_RL,SR_RR] = calculateSlipRatio (omega_FL,omega_FR,omega_RL,omega_RR,yawrate,Vx,vehicle)

Rw = vehicle.R;
tf = vehicle.tf;
tr = vehicle.tf;

VxFR = Vx+0.5*yawrate*tf;
VxFL = Vx-0.5*yawrate*tf;
VxRR = Vx+0.5*yawrate*tr;
VxRL = Vx-0.5*yawrate*tr;

smol = 0.01;
SR_FR = (omega_FR*Rw- VxFR)/(VxFR + smol);
SR_FL = (omega_FL*Rw- VxFL)/(VxFL + smol);
SR_RR = (omega_RR*Rw- VxRR)/(VxRR + smol);
SR_RL = (omega_RL*Rw- VxRL)/(VxRL + smol);

minrot = 1;
if abs(omega_FR)< minrot && abs(VxFR)<0.1
    SR_FR = 0;
end

if abs(omega_FL)< minrot && abs(VxFL)<0.1
    SR_FL = 0;
end

if abs(omega_RR)< minrot && abs(VxRR)<0.1
    SR_RR = 0;
end

if abs(omega_RL)< minrot && abs(VxRL)<0.1
    SR_RL = 0;
end
end
