%% OPTIMAL CONTROL
Ts = 0.05; 
N = 60; % we are working at 20Hz --> N = 60 means 3 seconds.

sysD = c2d(sys, Ts, 'zoh'); % system discretization using ZOH
Ad = sysD.a;
Bd = sysD.b;
Cd = sysD.c;
Dd = sysD.d;

% Controllability Check
Rd = ctrb(Ad,Bd);
disp(rank(Rd));

% Reachability matrix
Rn = Bd;
for i = 2:N
    Rn = [Bd Ad*Rn];
end

% Control Input Calculation
u = pinv(Rn)*(xf - Ad^N*x0);
u = reshape(u, 2, []).';  % Reshape to get T in the first column and phi in the second

% Final State Adjustment
% xf = Ad*xf + Bd*uf --> uf = pinv(Bd)*(I-Ad)*xf
uf = pinv(Bd)*(eye(6)-Ad)*xf;
u1 = [uf'; u];

% Test the optimal controller and plot
timing = Ts*(0:size(u1,1)-1).';
y_lin = lsim(sys, flipud(u1), timing, x0, 'zoh');
hold off
plot(y_lin)
legend("x","y","theta")
