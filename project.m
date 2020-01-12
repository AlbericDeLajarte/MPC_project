%% Setup
setup();

%% Controller
clc;
Ts = 1/5; T_max = 10; % seconds
quad = Quad(Ts);
[xs, us] = quad.trim();

sys = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);


for i=0:0.1:1
    i
    % Declare Q and R for the LQR controller
    Q = 10^(i)*eye(4); Q(3,3) = 6.3; Q(4,4) = 8.5;
    R = 3.1;
    
    % Create 4 independants controllers
    mpc_x = MPC_Control_x(sys_x, Ts, Q, R);
    mpc_y = MPC_Control_y(sys_y, Ts, Q, R);
    mpc_z = MPC_Control_z(sys_z, Ts, Q(3:end, 3:end), R);
    mpc_yaw = MPC_Control_yaw(sys_yaw, Ts, Q(3:end, 3:end), R);
    
    % Initialize 4 states vector
    x = zeros(4, T_max/Ts); x(:, 1) = [0;0;0;2];
    y = zeros(4, T_max/Ts); y(:, 1) = [0;0;0;-2];
    z = zeros(2, T_max/Ts); z(:, 1) = [0;2];
    yaw = zeros(2, T_max/Ts); yaw(:, 1) = [0;0.785];
    
    % Run simulation with MPC controller
    for k = 1:T_max/Ts -1
        x(:, k+1) = mpc_x.A*x(:, k) + mpc_x.B*mpc_x.get_u(x(:, k));
        y(:, k+1) = mpc_y.A*y(:, k) + mpc_y.B*mpc_y.get_u(y(:, k));
        z(:, k+1) = mpc_z.A*z(:, k) + mpc_z.B*mpc_z.get_u(z(:, k));
        yaw(:, k+1) = mpc_yaw.A*yaw(:, k) + mpc_yaw.B*mpc_yaw.get_u(yaw(:, k));
    end
    
    % Plot Pitch, Yaw and Roll
    figure(2);
    plot(0:Ts:T_max-Ts, x(2, :)); hold on;
    plot(0:Ts:T_max-Ts, y(2, :)); hold on;
    plot(0:Ts:T_max-Ts, yaw(2, :)); hold on;

    plot(0:Ts:T_max-Ts, ones(1,T_max/Ts)*0.785/1000, 'k'); hold on;
    plot(0:Ts:T_max-Ts, -ones(1,T_max/Ts)*0.785/1000, 'k'); hold on;
    xlabel("Time [s]"); ylabel("angle [rad]");
    legend("pitch", "roll", "yaw");
    movegui([0 600]);
    
    % Plot X, Y and Z
    figure(1);
    plot(0:Ts:T_max-Ts, x(4, :)); hold on;
    plot(0:Ts:T_max-Ts, y(4, :)); hold on;
    plot(0:Ts:T_max-Ts, z(2, :)); hold on;

    plot(0:Ts:T_max-Ts, ones(1,T_max/Ts)*2/1000, 'k'); hold on;
    plot(0:Ts:T_max-Ts, -ones(1,T_max/Ts)*2/1000, 'k'); hold on;
    xlabel("Time [s]"); ylabel("Position [m]");
    legend("x position", "y position", "z position");
    movegui([0 0]);

    pause;
    close(1);
    close(2);
end