%% Setup
setup();

%% Part 1
quad = Quad();
BIAS = -0.1;
Tf = 40; % Time to simulate for
x0 = zeros(12,1); % Initial state
x0 = [0 0 0 0 0 0 0 0 0 0 0 0];
u = [1;1;1;1]; % Input to apply
sim = ode45(@(t, x) quad.f(x, u), [0, Tf], x0); % Solve the system ODE
%quad.plot(sim, u); % Plot the result

%% Part 2

[xs,us] = quad.trim(); % Compute steady?state for which 0 = f(xs,us)
sys = quad.linearize(xs, us); % Linearize the nonlinear model

sys_transformed = sys * inv(quad.T); % New system is A * x + B * inv(T) * v

[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);

%% Part 3
clc
Ts = 1/5;
quad = Quad(Ts);
[xs, us] = quad.trim();

sys = quad.linearize(xs, us);

[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);

% Design MPC controller 
mpc_x   = MPC_Control_x(sys_x, Ts);
mpc_y   = MPC_Control_y(sys_y, Ts);
mpc_z   = MPC_Control_z(sys_z, Ts);
mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);

sim = quad.sim(mpc_x,mpc_y,mpc_z,mpc_yaw,BIAS);
quad.plot(sim); % Plot the result

