function [ v_dot,omega_dot,drag ] = rigid_body( car,v,omega,euler,F_R )
%rigid_body( car,v,omega,euler,F_R )
%calculates rigid-body dynamics
% Inputs
%   car = structure containing the car parameters
%   v = [vx;vy;vz]              vehicle-fixed coordinates
%   omega = [wx;wy;wz]          vehicle-fixed coordinates
%   euler = [roll;pitch;yaw]    inertial-fixed coordinates
%   F_R = [FRix;FRiy;FRiz]      vehicle-fixed coordinates


% make sure inputs are column vectors
v = v(:);
omega = omega(:);
euler = euler(:);

G =  car.m*Rot_VTE(euler)*[0;0;-car.g]; % gravity force in vehicle coordinates
drag = 1/2*car.rho/car.g*car.Cd*car.A  * (v'*v); % assumption that car.A is the same for all poses of the vehicle
Fw = -drag*v/norm(v,1); % drag has the opposite direction than v
Fw = car.use_resistances * Fw;
v_dot = 1/car.m*(sum(F_R,2) + G + Fw) - cross(omega,v);

A = [ car.r1, car.r2, car.r3, car.r4];
B = F_R;
aero = [0;0;0]; % TODO

omega_dot = car.JmatInv*( sum(cross(A,B,1),2) + aero - cross(omega,car.Jmat*omega));



end

