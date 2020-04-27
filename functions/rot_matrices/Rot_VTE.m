function [ out ] = Rot_VTE( in )
%Rot_VTE inertial(world)-fixed to body(vehicle)-fixed
% in = [roll, pitch, yaw] = [phi, theta, psi]
% body-fixed = VTE * inertial-fixed

phi   = in(1);
theta = in(2);
psi   = in(3);


% inverse matrix
ETV = [cos(theta)*cos(psi),   sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi),   cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
       cos(theta)*sin(psi),   sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi),   cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
               -sin(theta),                                sin(phi)*cos(theta),                                cos(phi)*cos(theta)];
% inverting back to what we want           
out = ETV';

end

