clear;clc;close all;

k  = -0.125:0.025:0.125;
sa = -12:2:12;

fz = 250*4.44822162;
ia = 0*pi/180;

n1 = length(sa);
n2 = length(k);
Fx = zeros(n1,n2);
Fy = zeros(n1,n2);

for i =1:n1
    for j=1:n2
        Fx(i,j) =  F_longit(sa(i)*pi/180,k(j),fz,ia);
        Fy(i,j) = F_lateral(sa(i)*pi/180,k(j),fz,ia);
    end
end

plot(Fy',Fx')
grid on
hold on
plot(Fy,Fx)
hold off
xlabel("Fy [N]")
ylabel("Fx [N]")
title('Hoosier 16.0x7.5-10 R20 tire budget @8psi/0IA/250lbs')
axis square