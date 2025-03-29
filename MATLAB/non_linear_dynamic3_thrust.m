function out=non_linear_dynamic3_thrust(in)

% In this version we are considering the inputs T (the thrust force) and
% phi (the thrust direction)

x1 = in(1); % x
x2 = in(2); % x'
x3 = in(3); % y
x4 = in(4); % y'
x5 = in(5); % angle
x6 = in(6); % angular velocity
u1 = in(7); % force of the thrust, T 
u2 = in(8); % angle of the thrust, phi

load('rocketParameters.mat');

dx1 = x2;
dx2 = -u1/M*sin(x5 + u2) - bx/M*x2;
dx3 = x4;
dx4 = u1/M*cos(x5 + u2) - g - by/M*x4;
dx5 = x6;
dx6 = -u1*L/J*sin(u2);


out = [dx1; dx2; dx3; dx4; dx5; dx6];
