classdef MPC_Control_z < MPC_Control
  properties
    A_bar, B_bar, C_bar % Augmented system for disturbance rejection    
    L                   % Estimator gain for disturbance rejection
  end
  
  methods
    function mpc = MPC_Control_z(sys, Ts)
      mpc = mpc@MPC_Control(sys, Ts);
      
      [mpc.A_bar, mpc.B_bar, mpc.C_bar, mpc.L] = mpc.setup_estimator();
    end
    
    % Design a YALMIP optimizer object that takes a steady-state state
    % and input (xs, us) and returns a control input
    function ctrl_opt = setup_controller(mpc)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   x(:,1) - initial state (estimate)
      %   d_est  - disturbance estimate
      %   xs, us - steady-state target
      % OUTPUTS
      %   u(:,1) - input to apply to the system
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      [n,m] = size(mpc.B);
      
      % Steady-state targets (Ignore this before Todo 3.3)
      xs = sdpvar(n, 1);
      us = sdpvar(m, 1);
      
      % Disturbance estimate (Ignore this before Part 5)
      d_est = sdpvar(1);

      % SET THE HORIZON HERE
      N = 15;
      
      % Predicted state and input trajectories
      x = sdpvar(n, N);
      u = sdpvar(m, N-1)-us;    

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 

      % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are 
      %       the DISCRETE-TIME MODEL of your system

      % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
      Q = eye(n); R = 20;
      M = [1; -1]; m = [0.3; 0.2]; 
     
      [K, Qf, ~] = dlqr(mpc.A, mpc.B, Q, R);
      K = -K;
      
       % Compute maximal invariant set
       Xf = polytope(M*K,m);

    %   figure(3); plot(Xf.projection(3:4)); hold on;
      
       Acl =  mpc.A + mpc.B*K;
       while 1
           prevXf = Xf;
           [T,t] = double(Xf);
           preXf = polytope(T*Acl,t);
           Xf = intersect(Xf, preXf);
           if isequal(prevXf, Xf)
               break
           end
     %      plot(Xf.projection(3:4)); hold on;
           %pause;
       end
      [Ff,ff] = double(Xf);
      
   %   figure(4);plot(Xf.projection(3:4)); xlabel("beta angle"); ylabel("beta speed"); 
    %  figure(5);plot(Xf.projection(1:2)); xlabel("x position"); ylabel("x speed"); 

      con = (x(:,2)-xs == mpc.A*(x(:,1)-xs) + mpc.B*(u(1)-us)) + (M*(u(1)-us) <= m);
      obj = ((x(:,1)-xs)'*Q*(x(:,1)-xs))+(u(:,1)-us)'*R*(u(:,1)-us);
      
      for i = 2:N-1
          con = [con, (x(:,i+1)-xs) == mpc.A*(x(:,i)-xs) + mpc.B*(u(i)-us)];     % System dynamics
          con = [con, M*(u(i)-us) <= m];                       % Input constraints
          obj = obj + (x(:,i)-xs)'*Q*(x(:,i)-xs) + (u(i)-us)'*R*(u(i)-us);  % Cost function
      end 
      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      ctrl_opt = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
        {x(:,1), xs, us, d_est}, u(:,1));
    end
    
    
    % Design a YALMIP optimizer object that takes a position reference
    % and returns a feasible steady-state state and input (xs, us)
   function target_opt = setup_steady_state_target(mpc)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   ref    - reference to track
      %   d_est  - offset, this note is added by Yves
      % OUTPUTS
      %   xs, us - steady-state target
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      % Steady-state targets
      n = size(mpc.A,1);
      xs = sdpvar(n, 1);
      us = sdpvar;
      
      % Reference position (Ignore this before Todo 3.2)  
      ref = sdpvar;
      % WRITE THE CONSTRAINTS AND OBJECTIVE HERE
      d_est = sdpvar(1);
      
      Q = eye(n); R = 1;
      M = [1; -1]; m = [0.3; 0.2];      
  
      con = [M*us <= m          ,...
             xs == mpc.A*xs + mpc.B*us + mpc.B*d_est];

      obj  = (mpc.C*xs - ref + d_est)^2; 
      
     % Compute the steady-state target
      target_opt = optimizer(con, obj, sdpsettings('solver', 'gurobi'), {ref,d_est}, {xs, us});
   end
    
    % Compute augmented system and estimator gain for input disturbance rejection
    function [A_bar, B_bar, C_bar, L] = setup_estimator(mpc)
      
      %%% Design the matrices A_bar, B_bar, L, and C_bar
      %%% so that the estimate x_bar_next [ x_hat; disturbance_hat ]
      %%% converges to the correct state and constant input disturbance
      %%%   x_bar_next = A_bar * x_bar + B_bar * u + L * (C_bar * x_bar - y);
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
      
      nx   = size(mpc.A,1);
      nu   = size(mpc.B,2);
      ny   = size(mpc.C,1);
      A_bar = [mpc.A          , mpc.B;
               zeros(1,nx),1          ];
      B_bar = [mpc.B;zeros(1,nu)];
      C_bar = [mpc.C,ones(ny,1)];

      L = -place(A_bar',C_bar',[0.3,0.4,0.5])';
      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end   
  end
end
