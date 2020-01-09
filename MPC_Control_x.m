classdef MPC_Control_x < MPC_Control
  
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
      N = 15;  %aka 8sec /Ts, sachant que l'énoncé veut qu'on prenne 8 secondes pour remettre à l'origine le drone écarté de 2m sur x ou y
      
      % Predicted state and input trajectories
      x = sdpvar(n, N);
      u = sdpvar(m, N-1);
      

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 

      % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are 
      %       the DISCRETE-TIME MODEL of your system

      % WRITE THE CONSTRAINTS AND OBJECTIVE HERE
      Q = eye(n)*10; R = 1;
      M = [1; -1]; m = [0.3; 0.3]; 
      F = [0 1 0 0; 0 -1 0 0]; f = [0.035; 0.035];
      
      [K, Qf, ~] = dlqr(mpc.A, mpc.B, Q, R);
      K = -K;
      
       % Compute maximal invariant set
       Xf = polytope([F;M*K],[f;m]);

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

      con = (x(:,2)-xs == mpc.A*(x(:,1)-xs) + mpc.B*(u(1)-us)) + (M*(u(1)-us) <= m-M*us);
      obj = ((x(:,1)-xs)'*Q*(x(:,1)-xs))+(u(:,1)-us)'*R*(u(:,1)-us);
       
      for i = 2:N-1
          con = [con, (x(:,i+1)-xs) == mpc.A*(x(:,i)-xs) + mpc.B*(u(i)-us)];     % System dynamics
          con = [con, M*(u(i)-us) <= m-M*us];                       % Input constraints
          obj = obj + (x(:,i)-xs)'*Q*(x(:,i)-xs) + (u(i)-us)'*R*(u(i)-us);  % Cost function
          con = [con, F*(x(:,i)-xs) <= f-F*xs];                       % State constraint
      end
      
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
      
      % WRITE THE CONSTRAINTS AND OBJECTIVE HERE
      
      nx = size(mpc.A,1);
      nu = size(mpc.B,2);
      
      Q = eye(n); R = 2;
      M = [1; -1]; m = [0.3; 0.3]; 
      F = [0 1 0 0; 0 -1 0 0]; f = [0.035; 0.035];      

      con = [M*us <= m          ,...
             F*xs <= f          ,...
             xs == mpc.A*xs + mpc.B*us ];

      obj  = (mpc.C*xs - ref)'*(mpc.C*xs - ref);    
      % Compute the steady-state target
      target_opt = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
      
    end
  end
end
