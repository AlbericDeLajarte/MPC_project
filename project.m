%% Setup
addpath("/Users/Alberic/Downloads/casadi-osx-matlabR2015a-v3.5.1")

cd /Library/gurobi811/mac64/matlab
gurobi_setup;

%% Part 1
quad = Quad();
Tf = 1.0; % Time to simulate for
x0 = zeros(12,1); % Initial state
u = [1;1;1;1]; % Input to apply
sim = ode45(@(t, x) quad.f(x, u), [0, Tf], x0); % Solve the system ODE
quad.plot(sim, u); % Plot the result

%% Part 2

[xs,us] = quad.trim(); % Compute steady?state for which 0 = f(xs,us)
sys = quad.linearize(xs, us); % Linearize the nonlinear model

sys_transformed = sys * inv(quad.T); % New system is A * x + B * inv(T) * v

[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);

%% Part 3

Ts = 1/5;
quad = Quad(Ts);
[xs, us] = quad.trim();
sys = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);

% Design MPC controller
mpc_x = MPC_Control_x(sys_x, Ts);

% Get control inputs with
ux = mpc_x.get_u([0.08; 0; 0; 0])

