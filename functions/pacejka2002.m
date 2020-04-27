function [Fx,Fy,Mz,My,Mx,du1,dv1,slipratio,slipangle] = pacejka2002(Vx,Vy,omega,Re,R0,gamma,Fz,u1,v1,bool_use_dynamics,str_side)
% function [Fx,Fy] = pacejka2002(slipratio,slipangle,gamma,Fz,Vx)

% tyre_dimension={275,35,20}

FNOMIN=4100;
LONGVL=16.5;

%  [SCALING_COEFFICIENTS]
LFZ0=1;
LCX=1;
LMUX=1; LMUX=3;
LEX=1;
LKX=1; % LKX=0.5;
LHX=1;
LVX=1;
LGAX=1;
LCY=1;
LMUY=1; LMUY=3;
LEY=1;
LKY=1;  %LKY=0.5;
LHY=1;
LVY=1;
LGAY=1;
LTR=1;
LRES=0;
LGAZ=1;
LXAL=1;
LYKA=1;
LVYKA=1;
LS=1;
LSGKP=1;
LSGAL=1;
LGYR=1;
LMX=1;
LVMX=1;
LMY=1;
%  [LONGITUDINAL_COEFFICIENTS]
PCX1=1.63;
PDX1=1.06;
PDX2=-0.0492;
PDX3=-2.29;
PEX1=0.5;
PEX2=-0.11;
PEX3=-0.06;
PEX4=0;
PKX1=19.7;
PKX2=-0.15;
PKX3=0.18;
PHX1=-0.0005;
PHX2=8.5e-5;
PVX1=0;
PVX2=0;
RBX1=9.0;
RBX2=-8.6;
RCX1=1.131;
REX1=0.081;
REX2=-0.15;
RHX1=-0.029;
PTX1=1.98;
PTX2=0.0003;
PTX3=-0.31;
%  [LATERAL_COEFFICIENTS]
PCY1=1.28;
PDY1=-0.92;
PDY2=0.22;
PDY3=-4.55;
PEY1=-1.1;
PEY2=0.65;
PEY3=-0.65;
PEY4=-12.41;
PKY1=-13.06;
PKY2=1.77;
PKY3=0.19;
PHY1=0.0034;
PHY2=-0.003;
PHY3=0.044;
PVY1=0.044;
PVY2=-0.030;
PVY3=-0.176;
PVY4=-0.44;
RBY1=6.4;
RBY2=7.91;
RBY3=-0.059;
RCY1=1.16;
REY1=0.22;
REY2=0.43;
RHY1=0.0007;
RHY2=0.023;
RVY1=0;
RVY2=0;
RVY3=0;
RVY4=10;
RVY5=1.94;
RVY6=-50;
PTY1=1.8;
PTY2=1.8;
%  [ALIGNING_COEFFICIENTS]
QBZ1=8.40;
QBZ2=-2.96;
QBZ3=0.45;
QBZ4=-0.45;
QBZ5=-0.45;
QBZ9=3.45;
QBZ10=0;
QCZ1=1.2;
QDZ1=0.1;
QDZ2=-0.003;
QDZ3=-0.55;
QDZ4=8.4;
QDZ6=-0.003;
QDZ7=0.005;
QDZ8=-0.11;
QDZ9=0.11;
QEZ1=-2.9;
QEZ2=-0.50;
QEZ3=0;
QEZ4=-0.12;
QEZ5=-3.71;
QHZ1=0.003;
QHZ2=0.00084;
QHZ3=0.158;
QHZ4=0.121;
QSX1=0.032;
QSX2=0.49;
QSX3=0.12;
QSY1=0.010;
QSY2=0; % toto je OK ze je to nevyuzite
QSY3=0;
QSY4=0;
SSZ1=0.023;
SSZ2=0.019;
SSZ3=0.5;
SSZ4=-0.28;
QTZ1=0.2;
MBELT=4.10;


Fz0 = FNOMIN;
Fz0_bar = Fz0*LFZ0;
dfz = (Fz -Fz0_bar)/Fz0_bar;
eps_x = 1e-3;
Vref = LONGVL;


zeta0 = 1;
zeta1 = 1;
zeta2 = 1;
zeta3 = 1;
zeta4 = 1;
zeta5 = 1;

%% transient behaviour - not working!
Vsy = Vy;
Vsx = Vx - omega*Re;
gamma_y = gamma*LGAY;


sigmak = Fz*(PTX1 + PTX2*dfz)*exp(PTX3*dfz)*(R0/Fz0)*LSGKP;
sigmaa = PTY1*sin(2*atan(Fz/(PTY2*Fz0*LFZ0)))*(1-PKY3*abs(gamma_y))*R0*LFZ0*LSGAL;

k_bar = u1/sigmak * sin(Vx);
a_bar = atan(v1/sigmaa);


dv1 = (sigmaa*Vsy - abs(Vx)*v1)/sigmaa;
du1 = (sigmak*Vsx - abs(Vx)*u1)/sigmak;


if bool_use_dynamics
    slipratio = k_bar;
    slipangle = a_bar;
else
    [ slipratio,slipangle ] = slips( Vx,Vy,omega,Re );
%     slipangle = -Vy/abs(Vx);  
end
if str_side == 'R'
    slipangle = -slipangle;
end

%% pure longtitudal force
SHX = (PHX1 + PHX2*dfz)*LHX;
SVX = Fz * (PVX1 + PVX2*dfz)*(LVX*LMUX*zeta1);
% Kxk = Fz* (PKX1 + PKX2*dfz)*exp(PKX3*dfz)*(1 + PPX1*dpi + PPX2 * dpi^2);
Kxk = Fz* (PKX1 + PKX2*dfz)*exp(PKX3*dfz)*LKX; % 2.pdf
kx = slipratio + SHX;
% lamKxk =
Ex = min((PEX1 + PEX2*dfz + PEX3 * dfz^2)*(1-PEX4 * sign(kx))*LEX,1);
% mux = (PDX1 + PDX2*dfz)*(1 + PPX3*dpi + PPX4*dpi^2 )*(1-PDX3*gamma^2)*LMUX_star;
gamma_x = gamma*LGAX;
mux = (PDX1 + PDX2*dfz)*(1-PDX3*gamma_x^2)*LMUX;
Dx = max( mux*Fz* zeta1,1e-3);
Cx = max(PCX1 * LCX,1e-3);
Kx = Kxk;
Bx = Kxk/(Cx*Dx + eps_x);
Fx0 = Dx*sin( Cx* atan(Bx*kx - Ex*(Bx*kx - atan(Bx*kx)))) + SVX;


%% pure lateral force


% Kyg0 = Fz*(PKY6 + PKY7*dfz)*(1 + PPY5*dpi)*LKYg;
% By = Kya/(Cy*Dy + eps_y);
% Kya = PKY1*Fz0_bar*(1 + PPY1*dpi)*(1-PKY3*abs(gamma_star))*...
%         sin(PKY4 * atan( Fz/Fz0_bar/(PKY2 + PKY5*gamma_star^2)/(1+PPY2*dpi) ))*zeta3*LKYa;
% Ey = (PEY1 + PEY2*dfz)*( 1+ PEY5*gamma_star^2 - (PEY3+PEY4*gamma_star)*sign(ay) )*LEY; % <=1
% SVyg = Fz*(PVY3 + PVY4*dfz)*gamma_star * LKYg * LMUY_bar*zeta2;
% SHY = (PHY1 + PHY2*dfz)*LHY + (Kyg0*gamma_star - SVyg)/(Kya + eps_K)*zeta0 + zeta4 -1;
% SVY = Fz* (PVY1 + PVY2*dfz)*(LVY*LMUY_bar*zeta2) + SVyg;
% muy = ( PDY1 + PDY2*dfz )*( 1 + PPY3*dpi + PPY4*dpi^2)*(1-PDY3*gamma_star^2)*LMUY_star;

Ky0 = PKY1 * Fz0 * sin(2*atan(Fz/(PKY2*Fz0*LFZ0))); % !! Fz0
Kyg0 = Fz*(PVY3 + PVY4*dfz) + Ky0*PHY3;
SVY = Fz*((PVY1 + PVY2*dfz)*LVY + ( PVY3 + PVY4*dfz )*gamma_y)*LMUY*zeta4;
SHY = (PHY1 + PHY2*dfz) * LHY + PHY3*gamma_y * zeta0 + zeta4 -1;
ay = slipangle + SHY;
Ky = Ky0*(1-PKY3*abs(gamma_y))*zeta3;
Ey = min((PEY1 + PEY2*dfz)*(1-(PEY3 + PEY4*gamma_y)*sign(ay))*LEY,1);
muy = (PDY1 + PDY2*dfz)*(1-PDY3*gamma_y^2)*LMUY;
Dy = muy*Fz*zeta2;
Cy = max(PCY1*LCY,eps(1));
By = Ky/(Cy*Dy + 1e-3);
Fy0 = Dy*sin( Cy * atan(By*ay - Ey*(By*ay - atan(By*ay)))  ) + SVY;
%% combined longitudinal

SHxa = RHX1;
as = ay + SHxa;
Bxa = RBX1*cos(atan(RBX2*kx))*LXAL;
Cxa = RCX1;
Exa = min(REX1 + REX2*dfz,1);
Dxa = Fx0/cos( Cxa*atan(Bxa*SHxa - Exa*(Bxa*SHxa - atan(Bxa*SHxa))) );
Gxa = cos(Cxa * atan(Bxa*as - Exa*(Bxa*as - atan(Bxa*as))))/...
    cos(Cxa*atan(Bxa*SHxa - Exa*(Bxa*SHxa - atan(Bxa*SHxa))));

Fx = Fx0*Gxa;

%% combined lateral
SHyk = RHY1 + RHY2*dfz;

ks = kx + SHyk;

Byk = RBY1*cos(atan(RBY2*(ay - RBY3)))*LYKA;
Cyk = RCY1;
Eyk = min(REY1 + REY2*dfz,1);

Dyk = Fy0/cos(Cyk*atan(Byk*SHyk - Eyk*(Byk*SHyk - atan(Byk*SHyk))));
DVyk = muy*Fz*(RVY1 + RVY2*dfz + RVY3*gamma)*cos(atan(RVY4*ay));

SVyk = DVyk*sin(RVY5*atan(RVY6*kx))*LVYKA;

% Fy = Dyk*cos(Cyk*atan(Byk*ks - Eyk*(Byk*ks - atan(Byk*ks))))+SVyk;

Gyk = max(cos(Cyk*atan(Byk*ks - Eyk*(Byk*ks - atan(Byk*ks))))/...
    cos(Cyk*atan(Byk*SHyk - Eyk*(Byk*SHyk - atan(Byk*SHyk)))),0);
Fy = Fy0*Gyk+SVyk;

% 
% 
% Fx = Fx0;
% Fy = Fy0;




%% Mz0 pure aligning moment
%  tyre_dimension={275,35,20}
R0 = 0.35; %% TODO co znamenaji tyre_dimension?
LT = LTR; % lambda t
LR = LRES; % lambda r
zeta6 = 1;
zeta7 = 1;
zeta8 = 1;


gammaz = gamma*LGAZ;
SHt = QHZ1 + QHZ2*dfz + (QHZ3 + QHZ4*dfz)*gammaz;
at = slipangle+ SHt;

Bt = (QBZ1 + QBZ2*dfz + QBZ3*dfz^2)*(1+QBZ4*gammaz + QBZ5*abs(gammaz))*LKY/LMUY;
Ct = QCZ1;
Dt = Fz * (QDZ1 + QDZ2*dfz)*(1+QDZ3*gammaz + QDZ4*gammaz^2)*R0/Fz0*LT*zeta5;
Et = min((QEZ1 + QEZ2*dfz + QEZ3*dfz^2)*(1 + (QEZ4 + QEZ5*gammaz)*(2/pi*atan(Bt*Ct*at))),1);
Br = (QBZ9*LKY/LMUY + QBZ10 * By*Cy)*zeta6;
Cr = zeta7;
Dr = Fz*( (QDZ6+QDZ7*dfz)*LR + (QDZ8 + QDZ9*dfz)*gammaz)*R0*LMUY + zeta8 -1;
t = Dt * cos(Ct*atan(Bt*at - Et*(Bt*at - atan(Bt*at))))*cos(slipangle);

Kz = -t*Ky; % approximation of aligning moment stiffness

SHf = SHY + SVY/Ky;
ar = slipangle*SHf;
Mzr = Dr*cos(Cr*atan(Br*ar))*cos(slipangle);

Mz0 = -t*Fy0+Mzr;

%% Mz combined aligning moment
% ateq = atan(sqrt(  tan(at)^2 + (Kx/Ky)^2*slipratio^2 * sign(at) ));
ateq = atan(sqrt(  tan(at)^2 + (Kx/Ky)^2*slipratio^2  ));
areq = atan(tan(ar)^2 + (Kx/Ky)^2 * slipratio^2 * sign(ar));
t = Dt * cos(Ct*atan(Bt*ateq - Et*(Bt*ateq - atan(Bt*ateq))))*cos(slipangle);




Fy_bar = Fy - SVyk;
Mzr = Dr*cos(Cr*atan(Br*areq))*cos(slipangle);
gamma_star = sin(gamma);
s = R0*(SSZ1 + SSZ2*(Fy/Fz0_bar))+(SSZ3 + SSZ4*dfz)*gamma_star*LS;
Mz = -t*Fy_bar + Mzr + s*Fx; % <-- toto hodne osciluje



Mz = Mz0;


%% Mx combined overturning moment
Mx = R0*Fz*(QSX1*LVMX - QSX2*gamma + QSX3*Fy/Fz0)*LMX;
%% My combined rolling resistance
My = R0*Fz*(QSY1 + QSY3*Fx/Fz0 + QSY3*abs(Vx/Vref) + QSY4*(Vx/Vref)^4);




%% 
if str_side == 'R'
    Fy = -Fy;
    Mz = -Mz;
end

