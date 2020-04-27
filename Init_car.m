addpath('./m-files/') 
% booleans
car.use_resistances = 1;

% Parameters of the car
car.m = 1300; % mass of vehicle
car.g = 9.81; % gravity acceleration
car_pom.Jxx = 200; % inetria X
car_pom.Jyy = 1300; % inetria Y
car_pom.Jzz = 1400;  % inertia Z

car_pom.Sz = 0.25; % distance from CG to to the point where springs are anchored
car.brakeTorqueMax = 2000; % Nm 

% car.CG_offset = -0.0275; % offset from center along positive X axis (positive = forward)
% car.CG_offset = -0.5; % offset from center along positive X axis (positive = forward)
car.CG_offset = 0;

car.wheelbase = 2.745;

car.fr = 0.0113; % roll resistance constant (should be function)
car.Cd = 0.18; % front aerodynamic constant
car.rho = 1.22; % air density
car.A = 2.0; % frontal area (for aerodynamics)

car.wheel_vel_threshold = 0.01; % If the wheel velocity is below this, use different slip calculation.
car.Jwheel = 1.0; % wheel moment of inetria
car.Jwheel = 1.0; % wheel moment of inetria
% car.r   = 0.254; % wheel radius
car.r   = 0.33; % wheel radius co sedi


% spring parameters, Front and Rear
car.caF = 30000; % spring stiffness
car.daF = 8000; % damper constant
car.caR = 40000; % spring stiffness 
car.daR = 8000; % damper constant

car.torqueMaxF = 1200;
car.torqueMaxR = 2000; % Nm
car.torqueRate = 60*1000; % Nm/s (prevedeno z 60Nm/ms)
car.powerMaxF = 150000; % W
car.powerMaxR = 200000; % W

car.steerMaxDeg = 720; % deg
car.steerRate = 500*50; % deg/s
car.steerGain = 12;

car.stepSize = 0.001; % s

car.tire_select = 1; % 1 - Rimac, 2 - simplifiedPacejka



% order: B C D E !
% D constant is multiplied by nominal load! (eg write 0.9*3900 instead of just 0.9)
% car.Pac_const_x = [ 10       1.6       4300      0.0];
% car.Pac_const_y = [-8.11       1.4       0.9*3900      -5.2];

% car.Pac_const_x = [ 12       1.55       0.9*4300      0];
% car.Pac_const_y = [-8.11       1.2       0.9*3900      -5.2];

car.Pac_const_mz = [-15.11       1.4413       369.9      0.22279];
% 
% zmena pro lepsi jezdeni
car.Pac_const_y = [-8.11       1.3       2*3900      0.2];
car.Pac_const_x = [ 7       1.6       2*4300      -0.5];


car.Fz0 = 4000;
car.Fyexp = 0.7;


car_pom.Sl = 1.7/2; % distance from CG to positive Y
car_pom.Sr = 1.7/2; % distance from CG to negative Y
%------------------------------------
car_pom.Lv = car.wheelbase/2-car.CG_offset; % distance from CG to positive X
car_pom.Lh = car.wheelbase/2+car.CG_offset; % distance from CG to negative X

car.r1 = [car_pom.Lv , car_pom.Sl , -car_pom.Sz]';
car.r2 = [car_pom.Lv , -car_pom.Sr, -car_pom.Sz]';
car.r3 = [-car_pom.Lh, car_pom.Sl , -car_pom.Sz]';
car.r4 = [-car_pom.Lh, -car_pom.Sr, -car_pom.Sz]';

car.Jmat = diag([car_pom.Jxx,car_pom.Jyy,car_pom.Jzz]);
car.JmatInv = diag([1/car_pom.Jxx,1/car_pom.Jyy,1/car_pom.Jzz]);
%------------------------------------

%% Initial conditions of the car

% car translation veloticy [m/s]
carInit.Vx = 16.6843;
% carInit.Vx = 41.2/3.6;

carInit.Vy = 0;
carInit.Vz = 0.0244;

% car rotation velocity [rad/s]
carInit.omega_x = 0;
carInit.omega_y = 0;
carInit.omega_z = 0;

carInit.roll = 0;
carInit.pitch = 0.0015;
% carInit.yaw = -0.045; % <--- toto bylo v souteznim aute
carInit.yaw = 0;

% car position [m]
carInit.position_earth_x = 0;
carInit.position_earth_y = 0;
carInit.position_earth_z = -0.0426;

% wheel velocities [rad/s]
% whvel = 65.6864;
whvel = carInit.Vx/car.r;
carInit.dro_FL = whvel;
carInit.dro_FR = whvel;
carInit.dro_RL = whvel;
carInit.dro_RR = whvel;

%% stuff needed for Simulink
busInfo = Simulink.Bus.createObject(car);
pom = Simulink.Bus.objectToCell({busInfo.busName});
pom2 = pom{1};
pom2{1} = 'car_bus';
pom{1} = pom2;
Simulink.Bus.cellToObject(pom);

car_init = struct2array(carInit)';
clear(busInfo.busName)
