function [ d_states,F_R,k,alpha,Mz,VTRi_steered,RiTV_steered,VTE,dl,ddl,drag,roll_resist, Fx ] ...
    = twintrack( car,states,delta_steering,torques, Fz_scaling)
% Inputs:
% car - structure with car parameters
% states - [16x1] vector of 16 states: [v;omega;euler;position_earth;dro]
% delta_steering - [4x1] vector of steering angles
% torques - [4x1] vector of input torques on individual wheels
% Fz_scaling - [4x1] vector. Tire normal forces are multiplied by this.

v = states(1:3);
omega = states(4:6);
euler = states(7:9);
position_earth  = states(10:12);
dro = states(13:16);

%--------------------------------------------------------------------------------
% euler angles 
%--------------------------------------------------------------------------------
phi = euler(1);
th = euler(2);
euler_mat = [1, sin(phi)*tan(th),cos(phi)*tan(th);...
            0, cos(phi),-sin(phi);...
            0, sin(phi)/cos(th), cos(phi)/cos(th) ];
euler_rates = euler_mat*omega;


%--------------------------------------------------------------------------------
% suspension 
%--------------------------------------------------------------------------------

[ F_R,k,alpha,Fx,Mz,VTRi_steered,RiTV_steered,VTE,dl,ddl,My ] = ...
    suspension(car,v,omega,euler,euler_rates,delta_steering,dro,position_earth, Fz_scaling);

%--------------------------------------------------------------------------------
% rigid body
%--------------------------------------------------------------------------------
[ v_dot,omega_dot,drag ] = ...
    rigid_body( car,v,omega,euler,F_R );


%--------------------------------------------------------------------------------
% earth-fixed velocity
%--------------------------------------------------------------------------------
dot_pos_earth = VTE'*v;

%--------------------------------------------------------------------------------
% wheel acceleration,  drag and roll resistance
%--------------------------------------------------------------------------------

% roll resist
dro_sign = dro;
dro_sign(dro_sign>1) = 1; % smoother sign function (no bullshit around 0)
dro_sign(dro_sign<-1) = -1;
roll_resist = dro_sign.*My;

ddro = (torques - car.r*Fx - car.use_resistances*(roll_resist))./car.Jwheel;


%--------------------------------------------------------------------------------
% OUTPUT
%--------------------------------------------------------------------------------
% v_dot(2) = 0;
d_states = [v_dot;omega_dot;euler_rates;dot_pos_earth;ddro];

end

