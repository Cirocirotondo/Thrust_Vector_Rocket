%% Define symbolic variables for states and inputs
clc
syms x1 x2 x3 x4 x5 x6 T phi bx by M J g l

% Parameters
% M = 5;       % Mass of the rocket
% J = 2;       % Moment of inertia
% g = 9.81;    % Gravitational constant
% T = M * g;   % Thrust at equilibrium (for hover)
% bx = 0.1;    % Drag coefficient along x
% by = 0.1;    % Drag coefficient along y
% l = 1;       % Distance from center to thrust line

% Define the state vector and input vector
x = [x1; x2; x3; x4; x5; x6];  % States: x, dx, y, dy, theta, dtheta
u = [T; phi];                  % Inputs: Thrust T and angle phi

% Define the equations based on the system dynamics
dx1 = x2; % dx/dt = x2
dx2 = -(T/M)*sin(x5 + phi) - (bx/M)*x2; % d(dx)/dt
dx3 = x4; % dy/dt = x4
dx4 = (T/M)*cos(x5 + phi) - g - (by/M)*x4; % d(dy)/dt
dx5 = x6; % d(theta)/dt = x6
dx6 = -(T*l/J)*sin(phi); % d(dtheta)/dt

% Combine into a state derivative vector
f = [dx1; dx2; dx3; dx4; dx5; dx6];

% Display the non-linear state-space function
disp('Non-linear state-space equations:');
disp(f);

%% Finding the equilibrium points
% Set up the equations for equilibrium by setting each derivative to zero
eq1 = dx1 == 0; % 1. Horizontal velocity must be zero
eq2 = dx2 == 0; % 2. Horizontal acceleration must be zero
eq3 = dx3 == 0; % 3. Vertical velocity must be zero
eq4 = dx4 == 0; % 4. Vertical acceleration must be zero
eq5 = dx5 == 0; % 5. Angular velocity must be zero
eq6 = dx6 == 0; % 6. Angular acceleration must be zero

% Set up the system of equations
equilibrium_eqs = [eq1, eq2, eq3, eq4, eq5, eq6];
disp(equilibrium_eqs)

% Substitute conditions based on equilibrium analysis
% Assuming T is non-zero, then sin(x5 + phi) = 0 implies x5 + phi = 0
% And to balance the vertical acceleration, we have T = Mg / cos(x5 + phi)
equilibrium_conditions = [x2 == 0, x4 == 0, x6 == 0, x5 + phi == 0, T == M*g];

% % Solve the system for equilibrium points
% solutions = solve(equilibrium_eqs, [x2, x4, x6, x5, phi, T]);
% 
% % Display equilibrium points
% disp('Equilibrium points:');
% disp(solutions);

%% LINEARIZATION
% Compute the Jacobian matrices
A = jacobian(f, x)  % Jacobian of f with respect to x
B = jacobian(f, u)  % Jacobian of f with respect to u
C = [1 0 0 0 0 0;
    0 0 1 0 0 0;
    0 0 0 0 1 0];
D = 0;

% Substitute equilibrium conditions into A and B matrices and simplify
% x2 = x4 = x6 = 0, x5 = 0, phi = 0, T = M*g
A_eq = simplify(subs(A, [x2,x4,x5,x6,phi, T], [0,0,0,0,0,M*g]));
B_eq = simplify(subs(B, [x2,x4,x5,x6,phi, T], [0,0,0,0,0,M*g]));
C_eq = C;

% Display the linearized matrices
disp('Linearized A matrix:');
disp(A_eq);

disp('Linearized B matrix:');
disp(B_eq);

% A = [0, 1, 0, 0, 0, 0;
%      0, -bx/M, 0, 0, -T/M, 0;
%      0, 0, 0, 1, 0, 0;
%      0, 0, 0, -by/M, 0, 0;
%      0, 0, 0, 0, 0, 1;
%      0, 0, 0, 0, 0, 0];
% 
% B = [0, 0;
%      0, -T/M;
%      0, 0;
%      1/M, 0;
%      0, 0;
%      0, -T*l/J];



%% Analysis of the linearized model

%% Reachability
% Define the number of states
n = size(A_eq, 1);

% Initialize the reachability matrix
Reachability_Matrix = B_eq;

% Compute [B A*B A^2*B ... A^(n-1)*B]
for i = 1:n-1
    Reachability_Matrix = [Reachability_Matrix, A_eq^i * B_eq];
end

% Simplify the reachability matrix
Reachability_Matrix = simplify(Reachability_Matrix);

% Display the reachability matrix
disp('Reachability Matrix:');
disp(Reachability_Matrix);

% Calculate the rank of the reachability matrix
rank_Reachability_Matrix = rank(Reachability_Matrix);

% Display the rank of the reachability matrix
disp('Rank of the Reachability Matrix:');
disp(rank_Reachability_Matrix);

% Check if the system is reachable
if rank_Reachability_Matrix == n
    disp('The system is reachable.');
else
    disp('The system is not reachable.');
end

%% Observability

% Initialize the observability matrix
Observability_Matrix = C_eq;

% Compute [C; C*A; C*A^2; ...; C*A^(n-1)]
for i = 1:n-1
    Observability_Matrix = [Observability_Matrix; C_eq * A_eq^i];
end

% Simplify the observability matrix
Observability_Matrix = simplify(Observability_Matrix);

% Display the observability matrix
disp('Observability Matrix:');
disp(Observability_Matrix);

% Calculate the rank of the observability matrix
rank_Observability_Matrix = rank(Observability_Matrix);

% Display the rank of the observability matrix
disp('Rank of the Observability Matrix:');
disp(rank_Observability_Matrix);

% Check if the system is observable
if rank_Observability_Matrix == n
    disp('The system is observable.');
else
    disp('The system is not observable.');
end

%% Eigenvalues
% Calculate the eigenvalues
[P, J] = jordan(A_eq)
inv(P)
eigenvalues = eig(A_eq);

% Display the eigenvalues
disp('The eigenvalues of the matrix A are:');
disp(eigenvalues);

% Step 2: Compute the arithmetic multiplicity
% Count the occurrence of each eigenvalue
unique_eig = unique(eigenvalues);  % Unique eigenvalues
arithmetic_multiplicities = arrayfun(@(x) sum(eigenvalues == x), unique_eig);  % Count occurrences

disp('Eigenvalues and their arithmetic multiplicities:');
disp(table(unique_eig, arithmetic_multiplicities));

% Step 3: Compute the geometric multiplicity
% Geometric multiplicity: Find null space of A - lambda*I for each eigenvalue
geometric_multiplicities = arrayfun(@(x) size(null(A - x*eye(size(A))), 2), unique_eig);

disp('Eigenvalues and their geometric multiplicities:');
disp(table(unique_eig, geometric_multiplicities));


%% Diagonalize to see which state variables diverge

[P, D] = eig(A_eq)
P*D*inv(P)

%%

% Create state-space model
sys = ss(A, B, eye(6), zeros(6,2));

% Display the system
disp(sys);

%% Substitute specific values for parameters
dx_substituted = subs(dx, [M, J, g, bx, by, l], [5, 2, 9.81, 0.1, 0.1, 1]);

% Display the result
disp(dx_substituted);



