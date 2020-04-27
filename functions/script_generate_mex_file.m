% tires_and_body(car,car_init,0,[0;0;0;0]);


codegen tires_and_body.prj


% quick speed comparison
N = 10000;
tic
for k=1:N
tires_and_body(car,car_init,0,[0;0;0;0]);
end
toc


tic
for k=1:N
tires_and_body_mex(car,car_init,0,[0;0;0;0]);
end
toc