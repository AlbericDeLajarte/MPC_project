classdef MPC_Control_yaw < MPC_Control
  
  methods
    % Design a YALMIP optimizer object that takes a steady-state state
    % and input (xs, us) and returns a control input
    function ctrl_opt = setup_controller(mpc)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   x(:,1) - initial state (estimate)
      %   xs, us - steady-state target
      % OUTPUTS
      %   u(:,1) - input to apply to the system
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      [n,m] = size(mpc.B);
      
      % Steady-state targets (Ignore this before Todo 3.2)
      xs = sdpvar(n, 1);
      us = sdpvar(m, 1);
      
      % SET THE HORIZON HERE
      N = 14;
      
      % Predicted state and input trajectories
      x = sdpvar(n, N);
      u = sdpvar(m, N-1);
      

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 

      % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are 
      %       the DISCRETE-TIME MODEL of your system

      % WRITE THE CONSTRAINTS AND OBJECTIVE HERE
      Q = eye(2); Q(1,1) = 6.3; Q(2,2) = 8.5;
      R = 3.1;
      M = [1; -1]; m = [0.2; 0.2]; 
      
      [K, Qf, ~] = dlqr(mpc.A, mpc.B, Q, R);
      K = -K;
      
       % Compute maximal invariant set
       Xf = polytope([M*K],[m]);

       %figure(3); plot(Xf.projection(3:4)); hold on;
      
       Acl =  mpc.A + mpc.B*K;
       while 1
           prevXf = Xf;
           [T,t] = double(Xf);
           preXf = polytope(T*Acl,t);
           Xf = intersect(Xf, preXf);
           if isequal(prevXf, Xf)
               break
           end
           %plot(Xf.projection(3:4)); hold on;
           %pause;
       end
      [Ff,ff] = double(Xf);
      
      %figure(4);plot(Xf.projection(3:4)); ylabel("x position"); xlabel("x speed");
      %figure(5);plot(Xf.projection(1:2)); xlabel("beta angle"); ylabel("beta speed"); 

      con = (x(:,2) == mpc.A*x(:,1) + mpc.B*u(1)) + (M*u(1) <= m);
      obj = u(1)'*R*u(1);
      
      for i = 2:N-1
          con = [con, x(:,i+1) == mpc.A*x(:,i) + mpc.B*u(i)];   % System dynamics                        % State constraint
          con = [con, M*u(i) <= m];                             % Input constraints
          obj = obj + x(:,i)'*Q*x(:,i) + u(i)'*R*u(i);          % Cost function
      end
      con = [con, Ff*x(:,N) <= ff]; % Terminal constraint
      obj = obj + x(:,N)'*Qf*x(:,N); % Terminal weight
      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      
      ctrl_opt = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
        {x(:,1), xs, us}, u(:,1));
    end
    
    
    % Design a YALMIP optimizer object that takes a position reference
    % and returns a feasible steady-state state and input (xs, us)
    function target_opt = setup_steady_state_target(mpc)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   ref    - reference to track
      % OUTPUTS
      %   xs, us - steady-state target
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      % Steady-state targets
      n = size(mpc.A,1);
      xs = sdpvar(n, 1);
      us = sdpvar;
      
      % Reference position (Ignore this before Todo 3.2)
      ref = sdpvar;            
            
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE       
      % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
      con = [];
      obj = 0;

      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      
      % Compute the steady-state target
      target_opt = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
      
    end
  end
end
