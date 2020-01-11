%% Setup
setup();

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
clc;
Ts = 1/5;
quad = Quad(Ts);
[xs, us] = quad.trim();

sys = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);

% Design MPC controller 
mpc_x = MPC_Control_x(sys_x, Ts);
mpc_y = MPC_Control_y(sys_y, Ts);
mpc_z = MPC_Control_z(sys_z, Ts);


T_max = 20; % seconds
x = zeros(12, T_max/Ts);
x(:, 1) = [0; 0; 0; 0; 0; 0; 0; 0; 0; 2; 0; 0]; % theta' theta  p' p

%quad.plot(sim, u); % Plot the result


for i = 1:T_max/Ts -1
    
    % Design MPC controller 
    
    Mb = mpc_x.get_u([x(2, i); x(5, i); x(7, i); x(10, i)]);
    Ma = mpc_y.get_u([x(1, i); x(4, i); x(8, i); x(11, i)]);
    Mg = 0;
    F = mpc_z.get_u([x(9, i); x(12, i)]);
    
    v = [F; Ma; Mb; Mg];
    u = quad.T\v;
    
    %[~, x_new] = ode45(@(t, x) quad.f(x, u), [(i-1)*Ts, i*Ts], x(i,:));  % Solve the system ODE
    %x(i+1,:) = x_new(end, :);
    
    %x(i+1,:) = quad.f(x(i,:)', u)' + x(i,:);
    x(:, i+1) = sys.A*x(:, i) + sys.B*u;
    i
end

%% plot
t = Ts:Ts:T_max;

figure(1); plot(t, x(1:3, :)); xlabel("Time"); ylabel("angle");
legend("roll", "pitch", "yaw");

figure(2); plot(t, x(10:12, :)); xlabel("Time"); ylabel("position");
legend("x", "y", "z");

%% Optimization
clc;
Ts = 1/5; T_max = 10; % seconds
quad = Quad(Ts);
[xs, us] = quad.trim();

sys = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);


for i=0:0.1:0
    i
    Q = 10^(1)*eye(4); Q(3,3) = 6.3; Q(4,4) = 8.5;
    R = 3.1;
    
    mpc_x = MPC_Control_x(sys_x, Ts, Q, R);
    
    x = zeros(4, T_max/Ts); x(:, 1) = [0;0;0;2];
    y = zeros(4, T_max/Ts); y(:, 1) = [0;0;0;2];
    z = zeros(2, T_max/Ts); z(:, 1) = [0;2];
    yaw = zeros(2, T_max/Ts); yaw(:, 1) = [0;0.7];
    
    for k = 1:T_max/Ts -1
        x(:, k+1) = mpc_x.A*x(:, k) + mpc_x.B*mpc_x.get_u(x(:, k));
        y(:, k+1) = mpc_y.A*y(:, k) + mpc_y.B*mpc_y.get_u(y(:, k));
        z(:, k+1) = mpc_z.A*z(:, k) + mpc_z.B*mpc_z.get_u(z(:, k));
        yaw(:, k+1) = mpc_yaw.A*yaw(:, k) + mpc_yaw.B*mpc_yaw.get_u(yaw(:, k));
    end
    figure(1); plot(Ts:Ts:T_max, x(4, :));hold on;
    plot(Ts:Ts:T_max, ones(1,T_max/Ts)*2/1000); hold on;
    plot(Ts:Ts:T_max, -ones(1,T_max/Ts)*2/1000);
    xlabel("Time [s]"); ylabel("X position [m]");
    pause;
    close(1);
end
