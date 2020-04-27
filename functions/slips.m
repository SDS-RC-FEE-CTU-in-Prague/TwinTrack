function [ k,alpha ] = slips( Vx,Vy,omega,r )
%SLIPS calculate slip on a wheel
%   [ k,alpha ] = slips( Vx,Vy,omega,r )
%   In:
%   Vx,Vy - wheel velocities, in wheel-fixed coordinate system! [m/s]
%   omega - rotational velocity [rad/s]
%   r - radius of the wheel [m]
%
%   Out:
%   k,alpha - longitudinal slip and sideslip
Vth = 0.1;

% Vsx = Vx - omega*r;
Vsy = Vy;

    %% Longitudinal Slip Ratios
    k = (omega*r - Vx)/max(omega*r, abs(Vx));
    
    k(k>1) = 1;
    k(k<-1) = -1;
    
    
alpha = atan2(Vsy,abs(Vx));

% smooth transition across 0
% if abs(Vx) < Vth
% k = -2*Vsx/(Vth + Vx^2/Vth);
% end

% if abs(Vx) < Vth
% k = -Vsx/(abs(Vx)+0.1);
% end

% saturation
% if abs(k) > 10
%     k = sign(k)*10;
% end

end

