% changes parameters on the fly


Init_car; % reset to default parameters
%% YOUR CHANGES HERE

car.tire_select = 1; % change tire model


%% upload to Simulink
model_name = gcs;
set_param(strcat(model_name,'/car_struct'),'Value','car');