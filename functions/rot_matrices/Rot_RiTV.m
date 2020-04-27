function [ out ] = Rot_RiTV( Roll,Pitch,delta )
%Rot_RiTV body(vehicle)-fixed to wheel-fixed
%   in = [phi,theta, delta] = [Roll,Pitch,(steering angle)]



%inverse matrix
VTRi = [cos(delta)*cos(Pitch),                                  -sin(delta)*cos(Pitch),                                 -sin(Pitch);
        sin(Roll)*sin(Pitch)*cos(delta) + cos(Roll)*sin(delta), -sin(Roll)*sin(Pitch)*sin(delta) + cos(Roll)*cos(delta), sin(Roll)*cos(Pitch);
        cos(Roll)*sin(Pitch)*cos(delta)-sin(Roll)*sin(delta),   -cos(Roll)*sin(Pitch)*sin(delta) - sin(Roll)*cos(delta), cos(Roll)*cos(Pitch)];
    
out = VTRi';    

% 


end

