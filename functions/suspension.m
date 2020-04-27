function [ F_R,k,alpha,Fx,Mz,VTRi_steered,RiTV_steered,VTE,deltas_l,d_deltas_l,My ] = suspension(car,v,omega,euler,euler_rates,deltas_steer,dro,position_earth, Fz_disturbance)
%UNTITLED2 Summary of this function goes here
% Inputs
%   car = structure containing the car parameters
%   v = [vx;vy;vz]              vehicle-fixed coordinates
%   omega = [wx;wy;wz]          vehicle-fixed coordinates
%   euler = [roll;pitch;yaw]    inertial-fixed coordinates
%   deltas_l
%   d_deltas_l
%   deltas_steer
%   dro = 1x4 vector of vehicle speeds
%   Fz_disturbance - input for disturbances of normal force acting on wheels.
%   Has 4 values (one for each wheel) that are greater than 0. 1 means 100%
%   of nominal normal force (acts as no disturbance case).

VTE =  Rot_VTE(euler);  % earth -> vehicle
ETV = VTE'; % TODO can I really just transpose this?

RiTV = nan(3,3,4);
for i=1:4
    RiTV(:,:,i) = Rot_RiTV(euler(1),euler(2),deltas_steer(i));
end
RiTV_steered = RiTV(:,:,1);
VTRi_steered = RiTV_steered';
%--------------------------------------------------------------------------------
% spring displacement and its velocity
%--------------------------------------------------------------------------------
A1_new = (ETV*car.r1 - car.r1);
A2_new = (ETV*car.r2 - car.r2);
A3_new = (ETV*car.r3 - car.r3);
A4_new = (ETV*car.r4 - car.r4);
% anchor points displacements (from initial position) in earth coordinates
deltas_l = [A1_new(end);A2_new(end);A3_new(end);A4_new(end)] + position_earth(3);
% za = [A1_new(end);A2_new(end);A3_new(end);A4_new(end)];

A = [omega,omega,omega,omega];
B = [ car.r1, car.r2, car.r3, car.r4];
d_ri = ETV*(cross(A,B,1) + [v,v,v,v]); % TODO can I use this matrix for velocities?
% Shouldnt I differentiate it and use dETV*cross + ETV*d_cross?
% is this shit correct..?

% anchor points velocities in earth coordinates
d_deltas_l = d_ri(3,:);

%--------------------------------------------------------------------------------
% spring-damper
%--------------------------------------------------------------------------------
ca = [car.caF,car.caF,car.caR,car.caR];
da = [car.daF,car.daF,car.daR,car.daR];
V_FF = nan(3,4);    % spring force
F_load = nan(1,4);  % load o wheels

for i=1:4
    pom = -((ca(i) * deltas_l(i)) + (da(i)*d_deltas_l(i)));
    V_FF(:,i) = pom*VTE*[0;0;1];
    F_load(i) = pom*Fz_disturbance(i);
end


rA = [car.r1,car.r2,car.r3,car.r4];

V_r_Ri = nan(3,4);  % Vehicle fixed position of wheel center
V_v_Ri = nan(3,4);  % Vehicle fixed velocity of wheel center
R_v_Ri = nan(3,4);  % Wheel   fixed velocity of wheel center
k = nan(4,1);       % longitudal slips
alpha = nan(4,1);   % slip angles
Fx = nan(4,1);      % actual Fx in wheel coordinates
Fy = nan(4,1);      % actual Fy in wheel coordinates
Fx0 = nan(4,1);     % Fx0 = Fx WITHOUT considering friction elipse
Fy0 = nan(4,1);     % Fy0 = Fy WITHOUT considering friction elipse
Mz = nan(4,1);      % returning moment Mz
F_R = nan(3,4);     % Force vector in vehicle coordinates
Vx = nan(4,1);
Vy = nan(4,1);
My = nan(4,1);
Mx = nan(4,1);
% Pacejka dynamics
% du1 = nan(4,1); 
% dv1 = nan(4,1);

% sides = ['L','R','L','R'];
for i = 1:4
    V_r_Ri(:,i) = rA(:,i) + VTE*[0;0;-deltas_l(i)];
    V_v_Ri(:,i) = v + cross(omega,V_r_Ri(:,i)) + VTE*[0;0;-d_deltas_l(i)];
    R_v_Ri(:,i) = RiTV(:,:,i)*V_v_Ri(:,i);
    
    Vx(i) = R_v_Ri(1,i); Vy(i) = R_v_Ri(2,i);
    
    
    [Fx(i),Fy(i),Mz(i),My(i),Mx(i), k(i),alpha(i)] = tire_models(i,car,Vx(i),Vy(i),dro(i),car.r,0,F_load(i));
    
    F_R(:,i) =  RiTV(:,:,i)'*[Fx(i);Fy(i);0] + V_FF(:,i);
end






end


