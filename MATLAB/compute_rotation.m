% This script computes the rotations that provide the directions of the
% thrust in the global (fixed) frame. 
% There are two steps of the rotation: 1) the rotation of the thrust
% respect to the rocket 2) the rotation of the rocket respect to the fixed
% frame

syms T alpha beta phi theta

Ti = [0; 0; T];
% Rxb_alpha = rotx(alpha);
% Ryb_beta = roty(beta);
% Ry_phi = roty(phi);
% Rx_theta = rotx(theta);

Rxb_alpha = [1 0            0;
             0 cos(alpha)   -sin(alpha);
             0 sin(alpha)   cos(alpha)];

Ryb_beta = [cos(beta)   0   sin(beta);
            0           1   0;
            -sin(beta)  0   cos(beta)];

Ry_phi =  [cos(phi)   0   sin(phi);
           0           1   0;
           -sin(phi)  0   cos(phi)];

Rx_theta = [1 0            0;
            0 cos(theta)   -sin(theta);
            0 sin(theta)   cos(theta)];


R = Rx_theta * Ry_phi * Ryb_beta * Rxb_alpha

Tf = R*Ti

subs(Tf, [alpha beta phi theta], [30*pi/180 00*pi/180 0 30*pi/180] )