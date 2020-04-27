function [ Fx,Fy,Mz ] = pacejka_combined_slip(k,a,Fz,Fz0,Pac_const_x,Pac_const_y,Pac_const_mz)
%Pocita koeficienty pro vypocet combined slipu
% in = [] =[longitudal slip, sideslip angle, 4 koef. pro posunuti krivek]
% out = [Fx,Fy]

% prozatim zanedbano
Shx = 0;
Svx = 0;
Shy = 0;
Svy = 0;

% pacejka coefficientss
Bx = Pac_const_x(1);
Cx = Pac_const_x(2);
Dx = Pac_const_x(3);
Ex = Pac_const_x(4);

By = Pac_const_y(1);
Cy = Pac_const_y(2);
Dy = Pac_const_y(3);
Ey = Pac_const_y(4);

Bz = Pac_const_mz(1);
Cz = Pac_const_mz(2);
Dz = Pac_const_mz(3);
Ez = Pac_const_mz(4);

Kx = Bx*Dx*Cx;
Ky = By*Dy*Cy;

% Pure slips
kx = k + Shx;
ay = a + Shy;

Fx0 = Dx*sin((Cx*atan(Bx*kx - Ex*(Bx*kx - atan(Bx*kx)))) + Svx);
Fy0 = Dy*sin((Cy*atan(By*ay - Ey*(By*ay - atan(By*ay)))) + Svy);
Mz = Dz*sin((Cz*atan(Bz*ay - Ez*(Bz*ay - atan(Bz*ay)))));

Fx0 = Fz/Fz0*Fx0;
Fy0 = Fz/Fz0*Fy0;

%% combined - friction elipse method
kc = k + Shx + Svx/Kx;
ac = a + Shy + Svy/Ky;
a_star = sin(ac);
beta = acos(abs(kc)/sqrt(kc^2 + a_star^2));

% friction coefficients
mu_x_act = (Fx0 - Svx)/Fz;
mu_y_act = (Fy0 - Svy)/Fz;

mu_x_max = Dx/Fz;
mu_y_max = Dy/Fz;

mu_x =         1/sqrt((1/mu_x_act)^2 + (tan(beta)/mu_y_max)^2 );
mu_y = tan(beta)/sqrt((1/mu_x_max)^2 + (tan(beta)/mu_y_act)^2 );

Fx = abs(mu_x/mu_x_act)*Fx0;
Fy = abs(mu_y/mu_y_act)*Fy0;

% if k == 0
%     Fx = 0;
% end

if isnan(Fx)
    Fx = 0;
end
if isnan(Fy)
    Fy = 0;
end
% if a == 0
%     Fy = 0;
% end

if Fz == 0
    Fx = Fx0;
    Fy = Fy0;
end

%
% if abs(Vx) < 1
%    Fx = wr;
% end
% disp([Vx,wr])

end

