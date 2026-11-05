function result = jackal_terminal_design(mode, varargin)
% Reference-dependent terminal matrices for the four-state Jackal model.
%
% Design:
%   result = jackal_terminal_design;
%
% Evaluate at a reference:
%   pair = jackal_terminal_design('evaluate', result, psi_r, v_r);
%
% Paper (Koehler et al., reference generic terminal ingredients):
% Parameterization: equations (17)-(18), state-only basis (allowed by Lemma 2).
% Discrete-time terminal LMI: Lemma 2, equation (19), gridding as in (23).
% Discrete model: one RK4 step, h = 0.05 s, zero-order hold (TMPC model).
% Stage cost of the TMPC is h*(Q,R) (acados cost scaling), so (19b) uses hQ, hR.
%
% State order: [p_x; p_y; psi; v].
% Input order: [a; omega].
%
% This file computes P,K only.
% Passing a finite-grid check is not a proof over the continuous domain.

  if nargin == 0
    mode = 'design';
  end

  %% Evaluate a previously calculated design
  if strcmpi(mode, 'evaluate')

    if numel(varargin) ~= 3
      error(['Use jackal_terminal_design(''evaluate'', ' ...
             'result, psi_r, v_r).']);
    end

    design = varargin{1};
    psi = varargin{2};
    v = varargin{3};

    validateattributes(psi, {'numeric'}, ...
                       {'scalar', 'real', 'finite'});

    validateattributes(v, {'numeric'}, ...
                       {'scalar', 'real', 'finite'});

    if ~design.check.passed
      error('The design did not pass its independent grid check.');
    end

    if ~any(v >= design.cfg.v_intervals(:,1) & ...
            v <= design.cfg.v_intervals(:,2))
      error('Reference speed is outside the design intervals.');
    end


    [X, Y] = evaluate_xy( ...
        design.X_coef, design.Y_coef, psi, v);

    if min(eig(X)) <= 0
      error('X is not positive definite at this reference.');
    end

    result = struct( ...
        'P', X\eye(4), ...
        'K', Y/X, ...
        'X', X, ...
        'Y', Y);

    return;
  end

  if ~strcmpi(mode, 'design') || ~isempty(varargin)
    error('Use the documented design or evaluate mode.');
  end

  if exist('sdpvar', 'file') == 0
    error('YALMIP must be available on the MATLAB path.');
  end

  %% Reference bounds and retained numerical settings
  % Reference bounds do not replace the physical robot limits.
  cfg.v_intervals = [-1.5, -0.1;
                      0.1,  1.5];
  cfg.a_bounds = [-2.0, 2.0];
  cfg.omega_bounds = [-2.0, 2.0];

  % Weights from the supplied solve_sdp.m.
  cfg.Q = diag([20, 20, 1, 0.2]);
  cfg.R = diag([1, 20]);

  cfg.epsilon = 0.001;

  % TMPC sampling time (one RK4 step with zero-order hold).
  cfg.h = 0.05;

  % Omega_r enters 19.b non-linearly, so we grid it (paper Sec. III-B.2). 
  %The a_r enter affinely, so its two vertices are sufficient 
  cfg.n_omega = 5;
  cfg.n_omega_check = 9;

  cfg.npsi = 24;
  cfg.nv = 15;

  cfg.npsi_check = 48;
  cfg.nv_check = 29;

  cfg.solver_tol = 1e-6;
  cfg.solver = 'sedumi';

  n = 4;
  m = 2;

  %% Parameterization: equations (17)-(18)
  %
  % theta = [
  %   cos(psi);
  %   sin(psi);
  %   v*sin(psi);
  %   v*cos(psi)
  % ];
  %
  % X = X0 + theta1*X1 + ... + theta4*X4
  % Y = Y0 + theta1*Y1 + ... + theta4*Y4
  %
  % MATLAB indices 1,...,5 represent coefficients 0,...,4.

  X_coef = cell(1, 5);
  Y_coef = cell(1, 5);

  for j = 1:5
    X_coef{j} = sdpvar(n, n, 'symmetric');
    Y_coef{j} = sdpvar(m, n, 'full');
  end

  X_min = sdpvar(n, n, 'symmetric');

  % Positive-definiteness floor using the retained tolerance.
  constraints = [X_min >= cfg.solver_tol*eye(n)];

  % Stage cost of the TMPC is h*(||e||_Q^2 + ||du||_R^2)
  SQ = chol(cfg.h*cfg.Q + cfg.epsilon*eye(n));  
  SR = chol(cfg.h*cfg.R);

  %{
  SQ = chol(cfg.Q + cfg.epsilon*eye(n));
  SR = chol(cfg.R);

  % Input ordering: [a; omega].
  B = [
    0, 0;
    0, 0;
    0, 1;
    1, 0
  ]; 
  %}
  

  %% Design grid
  psi_grid = linspace(-pi, pi, cfg.npsi + 1);

  % Avoid repeating the equivalent periodic endpoint.
  psi_grid(end) = [];

  v_grid = speed_grid(cfg.v_intervals, cfg.nv);

  omega_grid = linspace(cfg.omega_bounds(1), cfg.omega_bounds(2), cfg.n_omega);

  %% Construct continuous-time tracking LMIs: equation (40)
  % old code
  %{
  n_blocks = 0;

  for psi = psi_grid
    for v = v_grid

      [X, Y] = evaluate_xy(X_coef, Y_coef, psi, v);

      constraints = [constraints; X >= X_min];

      A = jacobian_a(psi, v);
      H = A*X + B*Y;

      % At fixed psi,v, the full LMI is affine in a_r,omega_r.
      % Their endpoint combinations cover their entire input box.
      %
      % Speed is gridded because the complete parameter-dependent
      % LMI is not generally affine in speed.

      for a = cfg.a_bounds
        for omega = cfg.omega_bounds

          X_dot = evaluate_xdot( ...
              X_coef, psi, v, a, omega);

          M = [
            H + H' - X_dot, (SQ*X)', (SR*Y)';
            SQ*X,          -eye(n), zeros(n,m);
            SR*Y,          zeros(m,n), -eye(m)
          ];

          constraints = [constraints; M <= 0];

          n_blocks = n_blocks + 1;
        end
      end
    end
  end

  %% Objective: equation (40a)
  % Minimize the worst-case terminal-cost bound.
  objective = -logdet(X_min);

  options = sdpsettings( ...
      'solver', cfg.solver, ...
      'verbose', 1);

  fprintf('Tracking LMI blocks: %d\n', n_blocks);
  %}
  %% Construct discrete-time tracking LMIs: equation (19)
  n_blocks = 0;
  n_skipped = 0;

  for psi = psi_grid
      for v = v_grid

          [X, Y] = evaluate_xy(X_coef, Y_coef, psi, v);

          constraints = [constraints; X >= X_min];

          %for a = cfg.a_bounds
          for a = admissible_a(v, cfg)
              for omega = omega_grid

                  % Next reference r+ = f_d(r): psi and v are exact under ZOH.
                  psi_next = psi + cfg.h*omega;
                  v_next = v + cfg.h*a;

                  % Only pairs with r+ inside the design region, equation (23).
                  %{
                  if ~speed_in_intervals(v_next, cfg.v_intervals)
                      n_skipped = n_skipped + 1;
                      continue;
                  end
                  %}

                  [X_next, ~] = evaluate_xy(X_coef, Y_coef, psi_next, v_next);

                  [Ad, Bd] = rk4_jacobian(psi, v, a, omega, cfg.h);
                  H = Ad*X + Bd*Y;

                  % Equation (19b).
                  M = [
                      X,     H',          (SQ*X)',     (SR*Y)';
                      H,     X_next,      zeros(n,n),  zeros(n,m);
                      SQ*X,  zeros(n,n),  eye(n),      zeros(n,m);
                      SR*Y,  zeros(m,n),  zeros(m,n),  eye(m)
                      ];

                  constraints = [constraints; M >= 0];

                  n_blocks = n_blocks + 1;
              end
          end
      end
  end

  %% Objective: equation (19a)
  % Minimize the worst-case terminal-cost bound.
  objective = -logdet(X_min);

  options = sdpsettings( ...
      'solver', cfg.solver, ...
      'verbose', 1);

  %fprintf('Tracking LMI blocks: %d (skipped r+ outside region: %d)\n', ...
  %    n_blocks, n_skipped);




  %% Solve
  timer = tic;

  diagnostics = optimize( ...
      constraints, objective, options);

  solve_time = toc(timer);


%if diagnostics.problem ~= 0
 %   error( ...
  %    'SDP unsuccessful (code %d): %s. No result exported.', ...
   %   diagnostics.problem, diagnostics.info);
  %end

  % Code 4 (numerical problems) is accepted: the result is verified on the
  % design grid and an independent finer grid below, and rejected there if
  % any LMI is violated.
 if diagnostics.problem == 4
     warning('SDP solver reported numerical problems; relying on grid checks.');
 elseif diagnostics.problem ~= 0
     error( ...
         'SDP unsuccessful (code %d): %s. No result exported.', ...
         diagnostics.problem, diagnostics.info);
 end

  %% Extract numerical coefficients
  X_numeric = zeros(n, n, 5);
  Y_numeric = zeros(m, n, 5);

  for j = 1:5
    X_numeric(:,:,j) = value(X_coef{j});
    Y_numeric(:,:,j) = value(Y_coef{j});
  end

  X_min_numeric = value(X_min);

  if any(~isfinite(X_numeric(:))) || ...
     any(~isfinite(Y_numeric(:))) || ...
     any(~isfinite(X_min_numeric(:))) || ...
     min(eig(X_min_numeric)) <= 0

    error(['Nonfinite coefficients or nonpositive X_min. ' ...
           'No result exported.']);
  end

  %% Verify at the design points
  %{
  design_check = check_grid( ...
      X_numeric, Y_numeric, X_min_numeric, cfg, ...
      psi_grid, v_grid); 
  %}
  design_check = check_grid(X_numeric, Y_numeric, X_min_numeric, cfg, psi_grid, v_grid, omega_grid);


  %% Independently verify intermediate reference points
  psi_check = linspace( ...
      -pi, pi, cfg.npsi_check + 1);

  psi_check(end) = [];

  % Shift headings relative to the design grid.
  psi_check = psi_check + pi/cfg.npsi_check;

  %v_check = speed_grid(cfg.v_intervals, cfg.nv_check);


  %check = check_grid( ...
  %    X_numeric, Y_numeric, X_min_numeric, cfg, ...
  %    psi_check, v_check);
  
  v_check = speed_grid(cfg.v_intervals, cfg.nv_check);

  omega_check = linspace(cfg.omega_bounds(1), cfg.omega_bounds(2), cfg.n_omega_check);

  check = check_grid(X_numeric, Y_numeric, X_min_numeric, cfg, psi_check, v_check, omega_check);


  fprintf( ...
      'Design-grid maximum normalized violation: %.9e\n', ...
      design_check.max_normalized_violation);

  fprintf( ...
      'Independent-grid maximum normalized violation: %.9e\n', ...
      check.max_normalized_violation);

  fprintf( ...
      'Independent-grid minimum eigenvalue of X: %.9e\n', ...
      check.min_eig_X);
  
  fprintf( ...
      'Independent-grid max spectral radius of Ad+Bd*K: %.9e\n', ...
      check.max_spectral_radius);

  fprintf( ...
      'Worst [psi,v,a,omega]: [%.15g, %.15g, %.15g, %.15g]\n', ...
      check.worst_reference);

  if ~design_check.passed || ~check.passed
    error([ ...
      'Reference-dependent matrices failed finite-grid verification. ' ...
      'No result exported. Grid refinement or parameterization ' ...
      'review is still required.']);
  end

  %% Store the matrix functions
  output_dir = fullfile(getenv('HOME'), 'HMPC_code', 'jackal_hmpc', 'src', 'mpc', 'mpc_solver', 'scripts', 'include', 'offline_computations', 'mpc-sdp');

  if ~isfolder(output_dir)
      mkdir(output_dir);
  end 

  result = struct( ...
      'cfg', cfg, ...
      'X_coef', X_numeric, ...
      'Y_coef', Y_numeric, ...
      'X_min', X_min_numeric, ...
      'solver_info', diagnostics.info, ...
      'solve_time', solve_time, ...
      'design_check', design_check, ...
      'check', check);

  save(fullfile(output_dir, 'P_K_reference_dependent.mat'), 'result');

  fprintf([ ...
      'Saved P_K_reference_dependent.mat. Finite-grid checks passed; ' ...
      'nonlinear terminal-set verification remains separate.\n']);
end


%% Continuous-time state Jacobian
%{
function A = jacobian_a(psi, v)

  A = [
    0, 0, -v*sin(psi), cos(psi);
    0, 0,  v*cos(psi), sin(psi);
    0, 0, 0, 0;
    0, 0, 0, 0
  ];
end
%}
%% Jacobian of the discrete model, equation (9)
function [Ad, Bd] = rk4_jacobian(psi, v, a, omega, h)
% One RK4 step with zero-order-hold input [a; omega].
% psi and v are exact under ZOH, so k2 = k3 and
%   p_x+ = p_x + h/6*(v*cos(psi) + 4*vm*cos(pm) + v1*cos(p1)),
% with vm = v + h*a/2, pm = psi + h*omega/2, v1 = v + h*a, p1 = psi + h*omega
% (p_y with sin).

    vm = v + h*a/2;
    v1 = v + h*a;
    
    pm = psi + h*omega/2;
    p1 = psi + h*omega;
    
    c0 = cos(psi); s0 = sin(psi);
    cm = cos(pm);  sm = sin(pm);
    c1 = cos(p1);  s1 = sin(p1);
    
    Ad = eye(4);
    Ad(1,3) = -h/6*(v*s0 + 4*vm*sm + v1*s1);
    Ad(1,4) =  h/6*(c0 + 4*cm + c1);
    Ad(2,3) =  h/6*(v*c0 + 4*vm*cm + v1*c1);
    Ad(2,4) =  h/6*(s0 + 4*sm + s1);
    
    % Input ordering: [a; omega].
    Bd = zeros(4, 2);
    Bd(1,1) =  h^2/6*(2*cm + c1);
    Bd(2,1) =  h^2/6*(2*sm + s1);
    Bd(4,1) =  h;
    Bd(1,2) = -h^2/6*(2*vm*sm + v1*s1);
    Bd(2,2) =  h^2/6*(2*vm*cm + v1*c1);
    Bd(3,2) =  h;
end

%% Evaluate X(r),Y(r)
function [X, Y] = evaluate_xy(X_coef, Y_coef, psi, v)

  b = [
    1;
    cos(psi);
    sin(psi);
    v*sin(psi);
    v*cos(psi)
  ];

  if iscell(X_coef)

    % Symbolic coefficient matrices during optimization.
    X = b(1)*X_coef{1};
    Y = b(1)*Y_coef{1};

    for j = 2:5
      X = X + b(j)*X_coef{j};
      Y = Y + b(j)*Y_coef{j};
    end

  else

    % Numerical coefficient matrices after optimization.
    X = b(1)*X_coef(:,:,1);
    Y = b(1)*Y_coef(:,:,1);

    for j = 2:5
      X = X + b(j)*X_coef(:,:,j);
      Y = Y + b(j)*Y_coef(:,:,j);
    end

    X = (X + X')/2;
  end
end


%% Time derivative of X along the reference dynamics

% To be deleted later

%{

function X_dot = evaluate_xdot(X_coef, psi, v, a, omega)

  % psi_dot = omega_r; v_dot = a_r.
  %
  % X_dot = dX/dpsi * omega_r + dX/dv * a_r.

  b_dot = [
    0;
    -sin(psi)*omega;
     cos(psi)*omega;
     a*sin(psi) + v*cos(psi)*omega;
     a*cos(psi) - v*sin(psi)*omega
  ];

  if iscell(X_coef)

    X_dot = 0*X_coef{1};

    for j = 2:5
      X_dot = X_dot + b_dot(j)*X_coef{j};
    end

  else

    X_dot = zeros(4);

    for j = 2:5
      X_dot = X_dot + b_dot(j)*X_coef(:,:,j);
    end

    X_dot = (X_dot + X_dot')/2;
  end
end
%}

%% Verify the recovered P(r),K(r): equation (33)

%{
function check = check_grid( ...
    X_coef, Y_coef, X_min, cfg, psi_grid, v_grid)

  B = [
    0, 0;
    0, 0;
    0, 1;
    1, 0
  ];

  SQ = chol(cfg.Q + cfg.epsilon*eye(4));

  check.max_normalized_violation = -Inf;
  check.min_eig_X = Inf;
  check.max_Xmin_violation = -Inf;
  check.worst_reference = NaN(1, 4);
  check.n_blocks = 0;

  for psi = psi_grid
    for v = v_grid

      [X, Y] = evaluate_xy(X_coef, Y_coef, psi, v);

      lam = min(eig(X));

      check.min_eig_X = min(check.min_eig_X, lam);

      check.max_Xmin_violation = max( ...
          check.max_Xmin_violation, ...
          max(eig(X_min - X)));

      if ~isfinite(lam) || lam <= 0

        check.max_normalized_violation = Inf;
        check.worst_reference = [psi, v, NaN, NaN];

        continue;
      end

      P = X\eye(4);
      K = Y/X;

      Ac = jacobian_a(psi, v) + B*K;

      for a = cfg.a_bounds
        for omega = cfg.omega_bounds

          X_dot = evaluate_xdot( ...
              X_coef, psi, v, a, omega);

          % Derivative of the inverse matrix.
          P_dot = -P*X_dot*P;

          D = Ac'*P + P*Ac + P_dot ...
              + cfg.Q + cfg.epsilon*eye(4) ...
              + K'*cfg.R*K;

          D = (D + D')/2;

          D_normalized = SQ'\D/SQ;

          violation = max(eig( ...
              (D_normalized + D_normalized')/2));

          if ~isfinite(violation)
            violation = Inf;
          end

          if violation > check.max_normalized_violation

            check.max_normalized_violation = violation;

            check.worst_reference = [psi, v, a, omega];
          end

          check.n_blocks = check.n_blocks + 1;
        end
      end
    end
  end

  check.passed = ...
      check.min_eig_X > 0 && ...
      check.max_normalized_violation <= cfg.solver_tol && ...
      check.max_Xmin_violation <= cfg.solver_tol;
end
%}

%% Verify the recovered P(r),K(r): equation (10)
function check = check_grid( ...
    X_coef, Y_coef, X_min, cfg, psi_grid, v_grid, omega_grid)

  SQ = chol(cfg.h*cfg.Q + cfg.epsilon*eye(4));

  check.max_normalized_violation = -Inf;
  check.min_eig_X = Inf;
  check.max_Xmin_violation = -Inf;
  check.max_spectral_radius = -Inf;
  check.worst_reference = NaN(1, 4);
  check.n_blocks = 0;
  check.n_skipped = 0;

  for psi = psi_grid
    for v = v_grid

      [X, Y] = evaluate_xy(X_coef, Y_coef, psi, v);

      lam = min(eig(X));

      check.min_eig_X = min(check.min_eig_X, lam);

      check.max_Xmin_violation = max( ...
          check.max_Xmin_violation, ...
          max(eig(X_min - X)));

      if ~isfinite(lam) || lam <= 0

        check.max_normalized_violation = Inf;
        check.worst_reference = [psi, v, NaN, NaN];

        continue;
      end

      P = X\eye(4);
      K = Y/X;

      %for a = cfg.a_bounds
      for a = admissible_a(v, cfg)
        for omega = omega_grid

          psi_next = psi + cfg.h*omega;
          v_next = v + cfg.h*a;
          
          %{
          if ~speed_in_intervals(v_next, cfg.v_intervals)
            check.n_skipped = check.n_skipped + 1;
            continue;
          end
          %}

          [X_next, ~] = evaluate_xy(X_coef, Y_coef, psi_next, v_next);

          lam_next = min(eig(X_next));
          check.min_eig_X = min(check.min_eig_X, lam_next);

          if ~isfinite(lam_next) || lam_next <= 0
            check.max_normalized_violation = Inf;
            check.worst_reference = [psi, v, a, omega];
            continue;
          end

          P_next = X_next\eye(4);

          [Ad, Bd] = rk4_jacobian(psi, v, a, omega, cfg.h);
          Acl = Ad + Bd*K;

          % Equation (10) with stage cost h*(Q,R).
          D = Acl'*P_next*Acl - P ...
              + cfg.h*cfg.Q + cfg.epsilon*eye(4) ...
              + K'*(cfg.h*cfg.R)*K;

          D = (D + D')/2;

          D_normalized = SQ'\D/SQ;

          violation = max(eig( ...
              (D_normalized + D_normalized')/2));

          if ~isfinite(violation)
            violation = Inf;
          end

          if violation > check.max_normalized_violation

            check.max_normalized_violation = violation;

            check.worst_reference = [psi, v, a, omega];
          end

          check.max_spectral_radius = max( ...
              check.max_spectral_radius, max(abs(eig(Acl))));

          check.n_blocks = check.n_blocks + 1;
        end
      end
    end
  end

  check.passed = ...
      check.min_eig_X > 0 && ...
      check.max_normalized_violation <= cfg.solver_tol && ...
      check.max_Xmin_violation <= cfg.solver_tol;
end


%%
function v_grid = speed_grid(intervals, n_per_interval)
% Grid each speed interval separately and join them.
    v_grid = [];
    for i = 1:size(intervals, 1)
        v_grid = [v_grid, linspace(intervals(i,1), intervals(i,2), ...
            n_per_interval)]; %#ok<AGROW>
    end
end

%%
function inside = speed_in_intervals(v, intervals)
    % True if v lies in one of the design speed intervals.
    inside = any(v >= intervals(:,1) & v <= intervals(:,2));
end

%%
function a_set = admissible_a(v, cfg)
% Inputs a_r for which r+ stays in the design region, equation (23).
% (19b) is affine in a_r, so the two endpoints of this interval suffice
% (paper Sec. III-B.2: vertices of the corresponding constraint set).
tol = 1e-9;
k = find(v >= cfg.v_intervals(:,1) - tol & v <= cfg.v_intervals(:,2) + tol, 1);
if isempty(k)
    a_set = [];
    return;
end
a_lo = max(cfg.a_bounds(1), (cfg.v_intervals(k,1) - v)/cfg.h);
a_hi = min(cfg.a_bounds(2), (cfg.v_intervals(k,2) - v)/cfg.h);
if a_lo > a_hi + tol
    a_set = [];
else
    a_set = unique([a_lo, max(a_lo, a_hi)]);
end
end
