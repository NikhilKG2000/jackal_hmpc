function result = jackal_terminal_design(mode, varargin)
% Reference-dependent terminal matrices for the four-state Jackal model.
%
% Design:
%   result = jackal_terminal_design;
%
% Evaluate at a reference:
%   pair = jackal_terminal_design('evaluate', result, psi_r, v_r);
%
% Paper:
% Parameterization: equations (17)-(18).
% Continuous-time terminal inequalities: Appendix C, (33), (40).
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

  cfg.npsi = 24;
  cfg.nv = 8;

  cfg.npsi_check = 48;
  cfg.nv_check = 15;

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

  SQ = chol(cfg.Q + cfg.epsilon*eye(n));
  SR = chol(cfg.R);

  % Input ordering: [a; omega].
  B = [
    0, 0;
    0, 0;
    0, 1;
    1, 0
  ];

  %% Design grid
  psi_grid = linspace(-pi, pi, cfg.npsi + 1);

  % Avoid repeating the equivalent periodic endpoint.
  psi_grid(end) = [];

  v_grid = speed_grid(cfg.v_intervals, cfg.nv);


  %% Construct continuous-time tracking LMIs: equation (40)
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
  design_check = check_grid( ...
      X_numeric, Y_numeric, X_min_numeric, cfg, ...
      psi_grid, v_grid);

  %% Independently verify intermediate reference points
  psi_check = linspace( ...
      -pi, pi, cfg.npsi_check + 1);

  psi_check(end) = [];

  % Shift headings relative to the design grid.
  psi_check = psi_check + pi/cfg.npsi_check;

  v_check = speed_grid(cfg.v_intervals, cfg.nv_check);


  check = check_grid( ...
      X_numeric, Y_numeric, X_min_numeric, cfg, ...
      psi_check, v_check);

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
function A = jacobian_a(psi, v)

  A = [
    0, 0, -v*sin(psi), cos(psi);
    0, 0,  v*cos(psi), sin(psi);
    0, 0, 0, 0;
    0, 0, 0, 0
  ];
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


%% Verify the recovered P(r),K(r): equation (33)
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

%%
function v_grid = speed_grid(intervals, n_per_interval)
% Grid each speed interval separately and join them.
v_grid = [];
for i = 1:size(intervals, 1)
    v_grid = [v_grid, linspace(intervals(i,1), intervals(i,2), ...
        n_per_interval)]; %#ok<AGROW>
end
end
