function out=non_linear_dynamic_simplified(in)

% In this simplified version we are only considering the rotation component. Therefore, the only two state variables are theta and theta'.
% The only input is phi.
% The only dynamic equation is J*theta'' = - T*L*sin(phi)

x1 = in(1); % angle
x2 = in(2); % angular velocity
u = in(3);  % angle of the motor


load('rocketParameters.mat', 'T', 'L', 'J');

dx1 = x2;
dx2 = -T*L/J*sin(u);

out = [dx1;dx2];
