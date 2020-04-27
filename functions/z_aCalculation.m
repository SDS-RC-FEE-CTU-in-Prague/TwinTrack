function [ out ] = z_aCalculation(in)
%roll-pitch-yaw TRANS 
%   Detailed explanation goes here

% constant parameters
lv      = in(1);
lh      = in(2);
sl      = in(3);
sr      = in(4);
sz      = in(5);

roll     = in(6);
pitch   = in(7);
yaw    = in(8);

% anchor points
A1 = [lv,sl,-sz]';
A2 = [lv,-sr,-sz]';
A3 = [-lh,sl,-sz]';
A4 = [-lh,-sr,-sz]';

ETV = Rot_VTE([roll,pitch,yaw])';

% anchor points displacements (from initial position)
A1_new = (ETV*A1 - A1);
A2_new = (ETV*A2 - A2);
A3_new = (ETV*A3 - A3);
A4_new = (ETV*A4 - A4);

out = [A1_new(end);A2_new(end);A3_new(end);A4_new(end)];

end

