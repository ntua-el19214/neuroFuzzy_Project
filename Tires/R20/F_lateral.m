function [F_psi, Gyk] = F_lateral(SA, SL, FZ, IA)
    % if FZ > 1112.0554070627252
    %     FZ = 1112.0554070627252;
    % end

    C_tire =0.66;
    % Lateral Fy parameters
    P = [1.409665529, 2.466217747,	-0.448172761,	10.31326152,	-1.364132404,	-1.689558598,...
		-0.061603776, -14.0961386,	-25.08892486,	0.764874741,	0.910896929,	0.994775646,...
		-0.009451576,-0.083058302,	-0.060281292,  -0.022530596 	1.414847712 	-1.976753548];

   	Fz0 = 1112.0554070627252;
	dfz = (FZ-Fz0)./Fz0;
	
    Sh = 0 + 0.*dfz + P(14).*IA;%P(12) + P(13).*dfz + P(14).*IA;
    ax = SA + Sh;

    C = P(1);
    D = FZ.*(P(2)+P(3).*dfz).*(1-P(4).*IA.*IA);
    E  = (P(5)+P(6).*dfz).*(1-(P(7)+P(8).*IA).*sign(ax));
    Kya = P(9).*Fz0.*sin(2.*atan(FZ./(Fz0.*P(10)))).*(1-P(11).*abs(IA));
    B  = Kya./(C.*D);
    Sv = FZ.*((0+0.*dfz)+(P(17)+P(18).*dfz).*IA);%FZ.*((P(15)+P(16).*dfz)+(P(17)+P(18).*dfz).*IA);
    
    y = D.*sin(C.*atan(B.*ax-E.*(B.*ax-atan(B.*ax))));
    Y = y + Sv;

    F_psi0 = C_tire.*Y;
    [Gyk, Svyk] = WeightFun(SL,SA,FZ, IA, P);

    F_psi = Gyk.*F_psi0;% + C_tire.*Svyk;
end

function [Gyk, Svyk] = WeightFun(k,a,fz,ia,j)
    p = [1,17.60322662,-12.91541201,0,0.048359834,-0.951640148,0.024791051,0.024791051,...
        -7.944434745,-11.76075801,-28.26039287,-6.401728279,0.008778823,26.17521773];

    fz0 = 1112.05540706273;
    dfz = (fz-fz0)./fz0;
    Cyk  = p(1); % RCY1
    Shyk = 0;%p(7) + p(8).*dfz; % RHY1, RHY2
    ks   = k + Shyk;
    Byk  = p(2).*cos(atan(p(3).*(a-p(4)))); % RBY1, RBY2, RBY3
    Eyk  = p(5)+p(6).*dfz; % REY1, REY2  

    Dy  = fz.*(j(2)+j(3).*dfz).*(1-j(4).*ia.*ia);
    Dvyk= Dy*(p(9)+p(10)*dfz+p(11)*ia)*cos(atan(p(12)*a));
    Svyk= Dvyk*sin(p(13)*atan(p(14)*k));

    Gyk0 = cos(Cyk.*atan(Byk.*Shyk-Eyk.*(Byk.*Shyk-atan(Byk.*Shyk)))); 
    Gyk = cos(Cyk.*atan(Byk.*ks-Eyk.*(Byk.*ks-atan(Byk.*ks))))./Gyk0; 
end