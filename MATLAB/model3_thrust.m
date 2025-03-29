% In this model, we assume that we can now control both the rocket thrust
% angle phi and the thrust T. 
% The state variables are x,y and theta, and their derivatives.
% The only input is phi.
% The three dynamic equation are along x, y and theta (J*theta'' = -T*L*sin(phi))


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

%% SIMULATION PARAMETERS
x0 = [0; 0; 0; 0; 5*pi/180; 0];
xf = [0; 0; 2; 0; 0; 0];

%% Define the matrices
% q1 = x
% q2 = x'
% q3 = y
% q4 = y'
% q5 = angle of the rocket from the vertical position
% q6 = q5' = angular velocity of the rocket
% u1 = T = thrust intensity
% u2 = phi = angle of the thrust with respect of the rocket axis
% y = output = q5

A = [0, 1,      0, 0,       0, 0;
     0, -bx/M,  0, 0,       -g, 0;
     0, 0,      0, 1,       0, 0;
     0, 0,      0, -by/M,   0, 0;
     0, 0,      0, 0,       0, 1;
     0, 0,      0, 0,       0, 0];

B = [0   0;     
     0   -T/M;
     0   0;
     1/M 0;
     0   0;
     0   -T*L/J];
C = [1 0 0 0 0 0;
    0 0 1 0 0 0;
    0 0 0 0 1 0]; 
D = [0 0; 0 0; 0 0];

sys = ss(A,B,C,D);
eig(A)
pole(sys); % this gives the same result as eig(A)
[P, J] = jordan(A)
Sc = ctrb(sys) % controllability matrix
rank(Sc) % = 6 --> the system is now completely controllable!
So = obsv(sys) % observability matrix
rank(So) % = 6 --> the system is completely observable
%rlocus(sys)



%% Regolatore con K di retroazione statico 
% We are now able to control all the poles. So, now it will be easier than
% in the model2 (which had only 4 reachable states)

K = place(A, B, [-1,-2, -3, -4, -1, -2]')
A1 = A - B*K
reg = ss(A1, sys.b, sys.c, 0)
% Gcl = feedback(sys, reg);

%% DECENTRALIZED CONTROL
% Permute:
% Permutation matrix P
P = [0, 0, 1, 0, 0, 0;
     0, 0, 0, 1, 0, 0;
     1, 0, 0, 0, 0, 0;
     0, 1, 0, 0, 0, 0;
     0, 0, 0, 0, 1, 0;
     0, 0, 0, 0, 0, 1];
% Apply the permutation
A_new = P * A * P'  % Rearrange rows and columns of A
B_new = P * B       % Rearrange rows of B
C_new = C * P'      % Rearrange columns of C
D_new = D           % D remains unchanged

% NOTE: we can see that [y, y'] are decoupled from [x x' theta theta'].
% However, we don't have enough inputs to decouple the second block.
% A1 = A([1,2,5,6], [1,2,5,6]);
% B1 = B([1,2,5,6],:);
% C1 =   [1 0 0 0;
%         0 0 1 0];
% % Let's find the relative degree
% First_degree = C1*B1; % = 0, hence I have to continue
% Second_degree = C1*A1*B1; % != 0, so I have found that the relative degree is equal to 2
% r = 2; % relative degree
% kv = inv(C1*A1^(r-1)*B1);
% kc = inv(C1*A1^(r-1)*B1)*C1*A1^r;

%% OPTIMAL CONTROL
Ts = 0.05; % NOTA: il tempo di campionamento deve essere scelto almeno 5-10 volte più piccolo del tempo dei poli del sistema
sysD = c2d(sys, Ts, 'zoh'); % discretizziamo con ZOH
Ad = sysD.a;
Bd = sysD.b;
Cd = sysD.c;
Dd = sysD.d;

% Verifichiamo la raggiungibilità --> Essendo passati da Tc a Td, essa non
% è più garantita.
Rd = ctrb(Ad,Bd)
disp(rank(Rd))
% Ok, viene ancora a rango 4 --> continuiamo a poter controllare x, x',
% theta e theta'. Continuiamo a non poter controllare y e y'

% risolviamo il problema di controllo ottimo in maniera analitica
% costruiamo la matrice di raggiungibilità in p passi
p = 60; % we are working at 20Hz --> p = 60 means to gain the control in 3 seconds.
Rp = Bd;
for i = 2:p
    Rp = [Bd Ad*Rp];
end

u = pinv(Rp)*(xf - Ad^p*x0);
% at this point, u is a vector with 2*p elements [u1(t=1); u2(1); u1(2);
% u2(2), ...]. I want to reshape it to a matrix with one column per each
% input
u = reshape(u, 2, []).';
figure("Name","Optimal control - Control")
plot(u)
legend("u1: T", "u2: phi")
hold on

% Let's impose that the derivative is zero in the final state
% xf = Ad*xf + Bd*uf --> uf = pinv(Bd)*(I-Ad)*xf
uf = pinv(Bd)*(eye(6)-Ad)*xf;
u1 = [uf'; u];


% Check of the feasibility of the control: the control should be between -5
% and +5 degrees. The thrust between 0 and 2*M*g (since we working in the 
% linearized system, it becomes from -M*g to M*g)
if(min(u1(:,2)) < -5 || max(u1(:,2)) > 5) && (min(u1(:,2)) < - M*g || max(u1(:,2)) > M*g)
    disp("Control over the limit!")
else
    disp("The optimal control is feasable")
end

% Let's test this optimal controller on simulink
timing = Ts*(0:size(u1,1)-1).';
uSim = [timing, flipud(u1)];
y_lin = lsim(sys, flipud(u1), timing, x0, 'zoh');
ySim = [timing, y_lin];
hold off
figure("Name","Optimal Control")
plot(y_lin)
legend("x","y","theta")
% pause
%% Optimal control, using lsqlin
% Without forcing the final derivative to 0
p = 60;
C_lsqlin = eye(2*p);
d_lsqlin = zeros(2*p,1);
Rp = Bd;
for i = 2:p
    Rp = [Bd Ad*Rp];
end
Aeq = Rp;
beq = xf - Ad^p*x0;
% lower and upper bounds
bound_thrust = M*g;
bound_angle = 5;
lb =-bound_angle*ones(2*p,1);
lb(1:2:end) = -bound_thrust;
ub = bound_angle*ones(2*p,1);
ub(1:2:end) = bound_thrust;

[u_lsqlin,~,~,exitflag1] = lsqlin(C_lsqlin,d_lsqlin,[],[],Aeq,beq,lb,ub);
% Reshape the vector u
u_lsqlin = reshape(u_lsqlin, 2, []).';

% We can see that we get the same result:
figure("Name","Optimal control - lsqlin - control")
hold on
plot(u_lsqlin)
plot(u)
%legend("u_{lsqlin}", "u")
hold off
disp("La norma della differenza dei due vettori vale: ")
disp(norm(u-u_lsqlin))

u_lsqlin = [uf'; u_lsqlin];


timing = Ts*(0:size(u_lsqlin,1)-1).';
uSim = [timing, flipud(u_lsqlin)];
y_lin = lsim(sys, flipud(u_lsqlin), timing, x0, 'zoh');
ySim = [timing, y_lin];
hold off
figure("Name","Optimal control - lsqlin")
plot(y_lin)
legend("x","y","theta")
% pause

%% Optimal control, minimum time, using lsqlin
exitflag = 0;
p = 1;
Rp = Bd;
while (exitflag <= 0) 
    p = p+1;
    fprintf('processing p = %d\n', p);    
    Rp = [Bd Ad*Rp];
    C_lsqlin = eye(2*p);
    d_lsqlin = zeros(2*p,1);
    Aeq = Rp;
    beq = xf - Ad^p*x0;
    % lower and upper bounds
    bound_thrust = M*g;
    bound_angle = 5;
    lb =-bound_angle*ones(2*p,1);
    lb(1:2:end) = -bound_thrust;
    ub = bound_angle*ones(2*p,1);
    ub(1:2:end) = bound_thrust;
    % solution with lsqlin
    [u_lsqlin,~,~,exitflag] = lsqlin(C_lsqlin,d_lsqlin,[],[],Aeq,beq,lb,ub);
    fprintf('exitflag = %d\n', exitflag);    
end

% Reshape the vector u
% At this point, u is a vector with 2*p elements [u1(t=1); u2(1); u1(2);
% u2(2), ...]. I want to reshape it to a matrix with one column per each
% input
u_lsqlin = reshape(u_lsqlin, 2, []).';
figure("Name","Optimal Control - minimum time - control")
plot(u_lsqlin)
legend("T", "phi")
hold on


u_lsqlin = [uf'; u_lsqlin];
timing = Ts*(0:size(u_lsqlin,1)-1).';
uSim = [timing, flipud(u_lsqlin)];
y_lin = lsim(sys, flipud(u_lsqlin), timing, x0, 'zoh');
ySim = [timing, y_lin];
hold off
figure("Name","Optimal Control - minimum time")
plot(y_lin)
legend("x","y","theta")

% We can see that the minimum is p = 6 -> 0.3 seconds


%% DECENTRALIZED CONTROL
P = [   0 0 1 0 0 0
        0 0 0 1 0 0
        1 0 0 0 0 0
        0 1 0 0 0 0
        0 0 0 0 1 0
        0 0 0 0 0 1];
Ac = P*A*P';
Bc = P*B;
Cc = C*P';
Dc = D;

% first subsystem (Ay, By)
Ay = Ac(1:2, 1:2);
By = Bc(1:2, 1);
Cy = [1 0];
Dy = [0];
% second subsystem (Axt, Bxt)
Axt = Ac(3:6, 3:6);
Bxt = Bc(3:6, 2);
Cxt = [ 1     0     0     0;
        0     0     1     0];
Dxt = [0; 0];





