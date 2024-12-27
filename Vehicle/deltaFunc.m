function [delta, delta_dot] = deltaFunc(t)
    
if t < 0.5
    delta = 0;
    delta_dot = 0;
elseif t < 1.5
    phase = 2 * pi / 2 * 0.5;
    delta = -deg2rad(10) * sin(2 * pi / 2 * t - phase);
    delta_dot = -deg2rad(10)*pi*cos(2 * pi / 2 * t - phase);
elseif t < 4
    phase = 2 * pi / 2.5 * 1.5;
    delta = deg2rad(20) * sin(2 * pi / 2.5 * t - phase);
    delta_dot = deg2rad(20)*2 * pi /2.5*cos(2 * pi / 2.5 * t - phase);
elseif t < 5
    phase = 2 * pi / 2 * 4;
    delta = deg2rad(10) * sin(2 * pi / 2 * t - phase);
    delta_dot = deg2rad(10)*pi*cos(2 * pi / 2 * t - phase);
else
    delta = 0;
    delta_dot = 0;


end
