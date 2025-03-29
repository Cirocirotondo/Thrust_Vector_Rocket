% In this model, we assume that we can only control the rocket thrust
% angle, phi. The thrust is fixed to 2*M*g.
% The state variables are x,y and theta, and their derivatives.
% The only input is phi.
% The three dynamic equation are along x, y and theta (J*theta'' = -T*L*sin(phi))


clc
clear all
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
% q1 = x
% q2 = x'
% q3 = y
% q4 = y'
% q5 = angle of the rocket from the vertical position
% q6 = q5' = angular velocity of the rocket
% u = phi = angle of the thrust with respect of the rocket axis
% y = output = q5

A = [0, 1, 0, 0, 0, 0;
     0, -bx/M, 0, 0, -T/M, 0;
     0, 0, 0, 1, 0, 0;
     0, 0, 0, -by/M, 0, 0;
     0, 0, 0, 0, 0, 1;
     0, 0, 0, 0, 0, 0];

B = [0;
     -T/M;
     0;
     0;
     0;
     -T*L/J];
C = [1 0 0 0 0 0;
    0 0 1 0 0 0;
    0 0 0 0 1 0]; 
D = [0; 0; 0];

sys = ss(A,B,C,D);
eig(A)
pole(sys); % this gives the same result as eig(A)
[P, J] = jordan(A)
Sc = ctrb(sys) % controllability matrix
rank(Sc) % = 4 --> the system is not completely controllable --> y and y' are not reachable
So = obsv(sys) % observability matrix
rank(So) % = 6 --> the system is completely observable
%rlocus(sys)



%% Regolatore con K di retroazione statico
% Non siamo in grado di controllare tutti i poli. Se ci proviamo, ci viene
% restituito errore. Però, possiamo fare il placing degli stati raggiungibili
reachable = [1,2,5,6]; % Reachable states: [x, x', theta, theta']
Ar = A(reachable, reachable)
Br = B(reachable)
Kr = place(Ar, Br, [-1,-2, -3, -4]')
K = [Kr([1,2]), 0, 0, Kr([3,4])]
A1 = A - B*K
reg = ss(A1, sys.b, sys.c, 0)
% Gcl = feedback(sys, reg);


%% OPTIMAL CONTROL
Ts = 0.05; % NOTA: il tempo di campionamento deve essere scelto almeno 5-10 volte più piccolo del minore tempo dei poli del sistema
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
p = 40; % we are working at 20Hz --> p = 40 means to gain the control in 2 seconds.
Rp = Bd;
for i = 2:p
    Rp = [Bd Ad*Rp];
end
x0 = [0; 0; 0; 0; 5*pi/180; 0];
xf = zeros(6,1);
u = pinv(Rp)*(xf - Ad^p*x0);
plot(u)
hold on

% Let's impose that the derivative is zero in the final state
% xf = Ad*xf + Bd*uf --> uf = pinv(Bd)*(I-Ad)*xf
uf = pinv(Bd)*(eye(6)-Ad)*xf;
u1 = pinv(Ad*Rp)*(xf-Bd*uf - Ad^(p+1)*x0);
u1 = [uf; u1];
plot(u1)
legend("u1", "u2")

% Check of the feasibility of the control: the control should be between -5
% and +5 degrees
if(min(u1) < -5 || max(u1) > 5)
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
plot(y_lin)
legend("x","y","theta")

%% Optimal control, using lsqlin
% Without forcing the final derivative to 0
p = 40;
C_lsqlin = eye(p);
d_lsqlin = zeros(p,1);
Rp = Bd;
for i = 2:p
    Rp = [Bd Ad*Rp];
end
Aeq = Rp;
beq = xf - Ad^p*x0;
lb = -5*ones(p,1);
ub = 5*ones(p,1);

[u_lsqlin,~,~,exitflag1] = lsqlin(C_lsqlin,d_lsqlin,[],[],Aeq,beq,lb,ub);
C_lsqlin1 = C_lsqlin;
d_lsqlin1 = d_lsqlin;
Aeq1 = Aeq;
beq1 = beq;
lb1 = lb;
ub1 = ub;

% We can see that we get the same result:
hold on
plot(u_lsqlin)
plot(u)
legend("u_{lsqlin}", "u")
hold off
disp("La norma della differenza dei due vettori vale: ")
disp(norm(u-u_lsqlin))

u_lsqlin = [uf; u_lsqlin];

plot(u_lsqlin)
timing = Ts*(0:size(u_lsqlin,1)-1).';
uSim = [timing, flipud(u_lsqlin)];
y_lin = lsim(sys, flipud(u_lsqlin), timing, x0, 'zoh');
ySim = [timing, y_lin];
hold off
plot(y_lin)
legend("x","y","theta")

%% Optimal control, minimum time, using lsqlin
exitflag = 0;
p = 1;
Rp = Bd;
while (exitflag <= 0) 
    p = p+1;
    fprintf('processing p = %d\n', p);    
    Rp = [Bd Ad*Rp];
    C_lsqlin = eye(p);
    d_lsqlin = zeros(p,1);
    Aeq = Rp;
    beq = xf - Ad^p*x0;
    % lower and upper bounds
    bound = 5;
    lb =-bound*ones(p,1);
    ub = bound*ones(p,1);
    % solution with lsqlin
    [u_lsqlin,~,~,exitflag] = lsqlin(C_lsqlin,d_lsqlin,[],[],Aeq,beq,lb,ub);
    fprintf('exitflag = %d\n', exitflag);    
end

u_lsqlin = [uf; u_lsqlin];
plot(u_lsqlin)
timing = Ts*(0:size(u_lsqlin,1)-1).';
uSim = [timing, flipud(u_lsqlin)];
y_lin = lsim(sys, flipud(u_lsqlin), timing, x0, 'zoh');
ySim = [timing, y_lin];
hold off
plot(y_lin)
legend("x","y","theta")

% We can see that the minimum is p = 6 -> 0.3 seconds


