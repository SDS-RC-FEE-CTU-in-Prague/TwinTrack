function [ Fx,Fy,Mz,My,Mx,k,a ] = tire_models(num,car,Vx,Vy,omega,Re,gamma,Fz)
%UNTITLED2 Summary of this function goes here
%   Inputs:
%   num - number of current tire: [1,2,3,4] <-> [FL,FR,RL,RR]
%   TODO

%% select tire model
switch round(car.tire_select) % floor is used to fix weird rounding error when linearizing
    case 1
        switch num
            case 1
                [ Fx,Fy,Mz,My,Mx,k,a ] = pacejka2002(Vx,Vy,omega,Re,car.r,gamma,Fz,0,0,0,'L');
            case 2
                [ Fx,Fy,Mz,My,Mx,k,a ] = pacejka2002(Vx,Vy,omega,Re,car.r,gamma,Fz,0,0,0,'R');
            case 3
                [ Fx,Fy,Mz,My,Mx,k,a] = pacejka2002(Vx,Vy,omega,Re,car.r,gamma,Fz,0,0,0,'L');
            otherwise
                [ Fx,Fy,Mz,My,Mx,k,a ] = pacejka2002(Vx,Vy,omega,Re,car.r,gamma,Fz,0,0,0,'R');
        end
    case 2
        [k,a] = slips( Vx,Vy,omega,car.r );
        Fz0 = car.Fz0;
        Pac_const_x = car.Pac_const_x;
        Pac_const_y = car.Pac_const_y;
        Pac_const_mz = car.Pac_const_mz;
        My = 0;
        
        dro_sign = omega;
        dro_sign(dro_sign>1) = 1; % smoother sign function (no bullshit around 0)
        dro_sign(dro_sign<-1) = -1;
        Mx = car.r*(dro_sign .* (car.fr*Fz));
        
        [Fx,Fy,Mz] = pacejka_combined_slip(k,a,Fz,Fz0,Pac_const_x,Pac_const_y,Pac_const_mz);
    case 3 % slipinput
        % Vy = slipratio
        % omega = Vy
        k = Vy;
        a = atan2(omega,abs(Vx));
        Fz0 = car.Fz0;
        Pac_const_x = car.Pac_const_x;
        Pac_const_y = car.Pac_const_y;
        Pac_const_mz = car.Pac_const_mz;
        My = 0;
        
        Mx = 0;
        
        [Fx,Fy,Mz] = pacejka_combined_slip(k,a,Fz,Fz0,Pac_const_x,Pac_const_y,Pac_const_mz);
        
        
        case 4 % slipinput
             % Vy = slipratio
        % omega = Vy
        
        k = Vy;
        Vy = omega;
        omega = (k*abs(Vx) + Vx)/car.r;
            
        switch num
            case 1
                [ Fx,Fy,Mz,My,Mx,k,a ] = pacejka2002(Vx,Vy,omega,Re,car.r,gamma,Fz,0,0,0,'L');
            case 2
                [ Fx,Fy,Mz,My,Mx,k,a ] = pacejka2002(Vx,Vy,omega,Re,car.r,gamma,Fz,0,0,0,'R');
            case 3
                [ Fx,Fy,Mz,My,Mx,k,a ] = pacejka2002(Vx,Vy,omega,Re,car.r,gamma,Fz,0,0,0,'L');
            otherwise
                [ Fx,Fy,Mz,My,Mx,k,a ] = pacejka2002(Vx,Vy,omega,Re,car.r,gamma,Fz,0,0,0,'R');
        end
        
        
        
    otherwise
        Fx = 0;Fy = 0;Mz = 0;My = 0;Mx = 0;
end
end

%% -------------------------------------------------------------------------------
%% wrap functions for tire models that don't have output in format [Fx,Fy,Mz,My,Mx]

function [ Fx,Fy,Mz,My,Mx ] = pacejka_combined_slip_wrap(k,a,Fz,Fz0,Pac_const_x,Pac_const_y,Pac_const_mz,Mx,My)
[Fx,Fy,Mz] = pacejka_combined_slip(k,a,Fz,Fz0,Pac_const_x,Pac_const_y,Pac_const_mz);
end

function [ Fx,Fy,Mz,My,Mx ] = default_tire()
Fx = 0;Fy = 0;Mz = 0;My = 0;Mx = 0;
end

