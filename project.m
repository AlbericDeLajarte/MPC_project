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


T_max = 1; % seconds
x = zeros(T_max/Ts, 12);
x(1,:) = [0 0 0 0 0 0 0 0 0 0 0 0]; % ??' ?  p' p

%quad.plot(sim, u); % Plot the result


for i= 1:T_max/Ts -1
    
    % Design MPC controller 
    
    Mb = mpc_x.get_u([x(i, 2); x(i, 5); x(i, 7); x(i, 10)]);
    Ma = mpc_y.get_u([x(i, 1); x(i, 4); x(i, 8); x(i, 11)]);
    Mg = 0;
    F = mpc_z.get_u([x(i, 9); x(i, 12)]);    
    
    v = [F; Ma; Mb; Mg];
    u = quad.T\v
    
    [~, x_new] = ode45(@(t, x) quad.f(x, u), [(i-1)*Ts, i*Ts], x(i,:));  % Solve the system ODE
    x(i+1,:) = x_new(end, :);
    i
end

%% plot
t = Ts:Ts:T_max;

figure(1); plot(t, x(:, 1:3)); xlabel("Time"); ylabel("angle");
legend("roll", "pitch", "yaw");

figure(2); plot(t, x(:, 10:12)); xlabel("Time"); ylabel("position");
legend("x", "y", "z");