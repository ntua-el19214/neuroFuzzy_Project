function [F_chi, Gxa] = F_longit(sa,kappa,Fz,IA)
    C_tire = 0.66;
    
    
    P = [1.506366622,2.321423444, 0, -0.517980743,-1.040847319, -4.14611736,-7.888048269, -0.208846481,40.16867967,...
        0.006972367, -0.521175988, -0.001790684,3.3578E-05, 0.070939652,-0.016185597];
		
    Fz0 = 1112.0554070627252; % Fz0 is 250lbs
	dfz = (Fz-Fz0)./Fz0;	
		
	Sh = 0;%P(12) + P(13)*dfz # PHX1, PHX2
    kx = kappa + Sh;


    C = P(1); % PCX1
    D = Fz.*(P(2)+P(3).*dfz).*(1-P(4).*IA.*IA); % PDX1, PDX2, PDX3
    E  = (P(5)+P(6).*dfz+P(7).*dfz.*dfz).*(1-P(8).*sign(kx)); % PEX1,PEX2, PEX3, PEX4
    Kxk = Fz.*(P(9)+P(10).*dfz).*exp(P(11).*dfz); % PKX1, PKX2, PKX3
    B  = Kxk./(C.*D);
    Sv = 0;%Fz*(p(14)+p(15)*dfz) # PVY1, PVY2

    
    y = D.*sin(C.*atan(B.*kx-E.*(B.*kx-atan(B.*kx))));
    F_chi0 = 0.96*C_tire*(y + Sv);

    Gxa = WeightFun(kappa,sa,Fz);
    F_chi = F_chi0.*Gxa;
end

function Gxa = WeightFun(k,a,fz)
    p = [1, -4.498596913, 9.492725758,-5.839626368,0,-0.095783917];

    fz0     = 1112.05540706273;
    dfz     = (fz-fz0)./fz0;
    Shxa    = 0;%p(6);
    alpha_s = a + Shxa;
    Cxa     = p(1);
    Bxa     = p(2).*cos(atan(p(3).*k))  ;  
    Exa     = p(4) + p(5).*dfz;
    Gxa0    = cos(Cxa.*atan(Bxa.*Shxa-Exa.*(Bxa.*Shxa-atan(Bxa.*Shxa))));
    Gxa     = cos(Cxa.*atan(Bxa.*alpha_s-Exa.*(Bxa.*alpha_s-atan(Bxa.*alpha_s))))./Gxa0;
end