function out=non_linear_dynamic2_no_thrust(in)

% In this version we are only considering the input phi, the thrust T is
% fixed and not controllable.


x1 = in(1); % x
x2 = in(2); % x'
x3 = in(3); % y
x4 = in(4); % y'
x5 = in(5); % angle
x6 = in(6); % angular velocity
u = in(7);  % angle of the thrust, phi

load('rocketParameters.mat');

dx1 = x2;
dx2 = -T/M*sin(x5 + u) - bx/M*x2;
dx3 = x4;
dx4 = T/M*cos(x5 + u) - g - by/M*x4;
dx5 = x6;
dx6 = -T*L/J*sin(u);


out = [dx1; dx2; dx3; dx4; dx5; dx6];
