% In this simplified version we are only considering the rotation component. Therefore, the only two state variables are theta and theta'.
% The only input is phi.
% The only dynamic equation is J*theta'' = - T*L*sin(phi)

clc
clear 
close all

%% Define the parameter
% M = 1.5;        % Mass of the rocket
% J = 0.125;      % Moment of inertia
% g = 9.81;       % Gravitational constant
% T = M * g;      % Thrust at equilibrium (for hover)
% bx = 0.1;       % Drag coefficient along x
% by = 0.1;       % Drag coefficient along y
% L = 0.4;        % Distance from center to thrust line
load('rocketParameters.mat');


%% Define the matrices
% q1 = angle of the rocket from the vertical position
% q2 = q1' = angular velocity of the rocket
% u = phi = angle of the thrust with respect of the rocket axis
% y = output = q1

% First define the matrices to use their elements easily
A = [0 1; 0 0];
B = [0; -T*L/J];
C = [1 0]; 
D = 0;

sys = ss(A,B,C,D);
eig(A)
pole(sys); % this gives the same result as eig(A)
Sc = ctrb(sys) % controllability matrix
rank(Sc) % = 2 --> the system is completely controllable
So = obsv(sys) % observability matrix
rank(So) % = 2 --> the system is not completely observable
rlocus(sys)

