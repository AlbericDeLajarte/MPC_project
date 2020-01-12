%% Setup
setup();

%% Optimization
clc;
Ts = 1/5; T_max = 10; %seconds
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

%% Plot
figure(1); 
plot(0:Ts:T_max-Ts, x(4, :)); hold on;
plot(0:Ts:T_max-Ts, y(4, :)); hold on;
plot(0:Ts:T_max-Ts, z(2, :)); hold on;

plot(0:Ts:T_max-Ts, ones(1,T_max/Ts)*(-2+2/1000), 'k'); hold on;
plot(0:Ts:T_max-Ts, ones(1,T_max/Ts)*(-2-2/1000), 'k'); hold on;
xlabel("Time [s]"); ylabel("Position [m]");
legend("x position", "y position", "z position");

figure(2);
plot(0:Ts:T_max-Ts, x(2, :)); hold on;
plot(0:Ts:T_max-Ts, y(2, :)); hold on;
plot(0:Ts:T_max-Ts, yaw(2, :)); hold on;

plot(0:Ts:T_max-Ts, ones(1,T_max/Ts)*(0.785+0.785/1000), 'k'); hold on;
plot(0:Ts:T_max-Ts, ones(1,T_max/Ts)*(0.785-0.785/1000), 'k'); hold on;
plot(0:Ts:T_max-Ts, ones(1,T_max/Ts)*(0.785/1000), 'k'); hold on;
plot(0:Ts:T_max-Ts, ones(1,T_max/Ts)*(-0.785/1000), 'k'); hold on;
xlabel("Time [s]"); ylabel("angle [rad]");
legend("pitch", "roll", "yaw");
