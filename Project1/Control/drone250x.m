function params = drone250x()
% Basic parameters for 250x quadrotor
%   drone250X outputs the basic parameters (mass, gravity and inertia) of
%   the quadrotor

m = 0.500; %kg: Quad 250x without gumstix and camera
g = 9.81;
I = [0.00025,   0,          2.55e-6;
     0,         0.000232,   0;
     2.55e-6,   0,          0.0003738];

params.mass = m;
params.I    = I;
params.invI = inv(I);
params.grav = g;
params.arm_length = 0.125; %half of diagonal length of 250x

% Ixx = I(1,1);
% Iyy = I(2,2);
% Izz = I(3,3);

params.maxangle = 40*pi/180; %you can specify the maximum commanded angle here
params.maxF = 4.0*m*g;
params.minF = 0.05*m*g;
 
% params.k = [8 8 10; 5 5 6]; 
% params.k_angle = [1.4 1.4 1.6; 0.2 0.2 0.2];
% params.k = [6 6 8; 5 5 6]; 
% params.k_angle = [1.3 1.3 1.5; 0.1 0.1 0.1];
params.k = [6 6 8; 7 7 8]; 
params.k_angle = [1.3 1.3 1.5; 0.1 0.1 0.1];

% You can add any fields you want in params
% for example you can add your controller gains by
% params.k = 0, and they will be passed into controller.m

end
