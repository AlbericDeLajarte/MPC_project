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

sim = quad.sim(mpc_x,mpc_y,mpc_z,mpc_yaw);
quad.plot(sim); % Plot the result

%% Optimization
clc;
Ts = 1/5; T_max = 10; % seconds
quad = Quad(Ts);
[xs, us] = quad.trim();

sys = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);

mpc_x = MPC_Control_x(sys_x, Ts);
mpc_y = MPC_Control_y(sys_y, Ts);
mpc_z = MPC_Control_z(sys_z, Ts);
mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);

x = zeros(4, T_max/Ts); x(:, 1) = [0;-5e-3;0;2-2];
y = zeros(4, T_max/Ts); y(:, 1) = [0;5e-3;0;-2-2];
z = zeros(2, T_max/Ts); z(:, 1) = [0;0];
yaw = zeros(2, T_max/Ts); yaw(:, 1) = [0;0];

for k = 1:T_max/Ts -1
    x(:, k+1) = mpc_x.A*x(:, k) + mpc_x.B*mpc_x.get_u(x(:, k), -2);
    y(:, k+1) = mpc_y.A*y(:, k) + mpc_y.B*mpc_y.get_u(y(:, k), -2);
    z(:, k+1) = mpc_z.A*z(:, k) + mpc_z.B*mpc_z.get_u(z(:, k), -2);
    yaw(:, k+1) = mpc_yaw.A*yaw(:, k) + mpc_yaw.B*mpc_yaw.get_u(yaw(:, k), 0.785);
end

% Plot
figure(1); 
plot(Ts:Ts:T_max, x(4, :)); hold on;
plot(Ts:Ts:T_max, y(4, :)); hold on;
plot(Ts:Ts:T_max, z(2, :)); hold on;

plot(Ts:Ts:T_max, ones(1,T_max/Ts)*(-2+2/1000), 'k'); hold on;
plot(Ts:Ts:T_max, ones(1,T_max/Ts)*(-2-2/1000), 'k'); hold on;
xlabel("Time [s]"); ylabel("Position [m]");
legend("x position", "y position", "z position");

figure(2);
plot(Ts:Ts:T_max, x(2, :)); hold on;
plot(Ts:Ts:T_max, y(2, :)); hold on;
plot(Ts:Ts:T_max, yaw(2, :)); hold on;

plot(Ts:Ts:T_max, ones(1,T_max/Ts)*(0.785+0.785/1000), 'k'); hold on;
plot(Ts:Ts:T_max, ones(1,T_max/Ts)*(0.785-0.785/1000), 'k'); hold on;
plot(Ts:Ts:T_max, ones(1,T_max/Ts)*(0.785/1000), 'k'); hold on;
plot(Ts:Ts:T_max, ones(1,T_max/Ts)*(-0.785/1000), 'k'); hold on;
xlabel("Time [s]"); ylabel("angle [rad]");
legend("pitch", "roll", "yaw");


