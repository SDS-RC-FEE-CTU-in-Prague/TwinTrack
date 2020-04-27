function [ v_dot,omega_dot ] = rigid_body( car,v,omega,euler,F_R )
%UNTITLED Summary of this function goes here
% Inputs
%   car = structure containing the car parameters
%   v = [vx;vy;vz]              vehicle-fixed coordinates
%   omega = [wx;wy;wz]          vehicle-fixed coordinates
%   euler = [roll;pitch;yaw]    inertial-fixed coordinates
%   Fi = [FRix;FRiy;FRiz]       vehicle-fixed coordinates


% Outputs
% v_dot, omega_dot, euler_dot


% make sure inputs are column vectors
v = v(:);
omega = omega(:);
euler = euler(:);

G =  car.m*Rot_VTE(euler)*[0;0;-car.g]; % gravity force in vehicle coordinates
Fw = 0; % TODO aerodynamic resistance forces
v_dot = 1/car.m*(sum(F_R,2) + G + Fw) - cross(omega,v);

A = [ car.r1, car.r2, car.r3, car.r4];
B = F_R;
aero = [0;0;0]; % TODO

omega_dot = car.JmatInv*( sum(cross(A,B,1),2) + aero - cross(omega,car.Jmat*omega));



end

