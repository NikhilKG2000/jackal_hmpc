%{

function output = jackal_check_alpha1(alpha_candidate, n_samples)
% Nonlinear continuous-time terminal decrease check.
%
% Paper:
%   Appendix C, equation (31).
%   Sampling strategy based on Algorithm 1.
%
% This checks one candidate level. It does not automatically reduce it.
% Finite sampling does not establish a continuous-domain guarantee.
%
% Usage:
%   output = jackal_check_alpha1([], 100);
%
% Empty alpha_candidate means use the calculated alpha2.

  validateattributes(n_samples, {'numeric'}, {'scalar', 'integer', 'positive'});
  output_dir = fullfile(getenv('HOME'), 'HMPC_code', 'jackal_hmpc', 'src', 'mpc', 'mpc_solver', 'scripts', 'include', 'offline_computations', 'mpc-sdp');

  loaded_design = load(fullfile(output_dir, 'P_K_reference_dependent.mat'), 'result');


  loaded_alpha2 = load(fullfile(output_dir, 'alpha2_reference_dependent.mat'), 'output');

  design = loaded_design.result;
  constraint_result = loaded_alpha2.output;

  if isempty(alpha_candidate)
    alpha_candidate = constraint_result.alpha2;
  end

  validateattributes(alpha_candidate, {'numeric'}, ...
      {'scalar', 'real', 'finite', 'positive'});

  if alpha_candidate > constraint_result.alpha2
    error('Candidate must not exceed the calculated alpha2.');
  end

  Q = design.cfg.Q;
  R = design.cfg.R;

  psi_grid = constraint_result.psi_grid;
  v_grid = constraint_result.v_grid;

  a_bounds = [
    constraint_result.reference_lb(4), ...
    constraint_result.reference_ub(4)
  ];

  omega_bounds = [
    constraint_result.reference_lb(5), ...
    constraint_result.reference_ub(5)
  ];

  % Record the random state without changing it.
  initial_rng_state = rng;

  worst_ratio = -Inf;
  worst_reference = NaN(1,4);
  worst_error = NaN(4,1);
  worst_level = NaN;

  n_checked = 0;
  n_failed = 0;

  for psi = psi_grid
    for v = v_grid

      pair = jackal_terminal_design( ...
          'evaluate', design, psi, v);

      P = pair.P;
      K = pair.K;

      % P = S'*S, so e = S\z maps a sphere to the ellipsoid.
      S = chol(P);

      % Random directions.
      directions = randn(4, n_samples);
      lengths = sqrt(sum(directions.^2, 1));

      % Regenerate any zero direction.
      while any(lengths == 0)
        zero_columns = lengths == 0;
        directions(:, zero_columns) = ...
            randn(4, sum(zero_columns));
        lengths = sqrt(sum(directions.^2, 1));
      end

      directions = directions ./ lengths;

      % Check boundary errors and random interior errors.
      %
      % Sampling distribution is an implementation detail:
      % radius rand^(1/4) gives uniform volume sampling in 4D.
      boundary_errors = ...
          sqrt(alpha_candidate)*(S\directions);

      radii = rand(1, n_samples).^(1/4);

      interior_errors = boundary_errors .* radii;

      errors = [boundary_errors, interior_errors];

      % Position can be zero here:
      % the model and terminal matrices are independent of absolute position.
      xr = [0; 0; psi; v];

      for a = a_bounds
        for omega = omega_bounds

          ur = [a; omega];

          X_dot = reference_xdot( ...
              design.X_coef, psi, v, a, omega);

          P_dot = -P*X_dot*P;

          reference_derivative = dynamics(xr, ur);

          for j = 1:size(errors, 2)

            e = errors(:, j);

            x = xr + e;
            u = ur + K*e;

            error_derivative = ...
                dynamics(x, u) - reference_derivative;

            V_dot = 2*e'*P*error_derivative ...
                    + e'*P_dot*e;

            stage_cost = e'*Q*e + (K*e)'*R*(K*e);

            % Equation (31) requires this residual <= 0.
            residual = V_dot + stage_cost;

            % Normalize for reporting; sign is unchanged.
            ratio = residual/stage_cost;

            n_checked = n_checked + 1;

            if ~isfinite(ratio)
              ratio = Inf;
            end

            if ratio > 0
              n_failed = n_failed + 1;
            end

            if ratio > worst_ratio
              worst_ratio = ratio;
              worst_reference = [psi, v, a, omega];
              worst_error = e;
              worst_level = e'*P*e;
            end
          end
        end
      end
    end
  end

  passed = n_failed == 0;

  fprintf('Candidate level: %.15g\n', alpha_candidate);
  fprintf('Nonlinear samples checked: %d\n', n_checked);
  fprintf('Samples failing equation (31): %d\n', n_failed);
  fprintf('Maximum normalized residual: %.9e\n', worst_ratio);

  fprintf('Worst [psi,v,a,omega]: [%.15g, %.15g, %.15g, %.15g]\n', worst_reference);

  if passed
    fprintf('Candidate passed the sampled continuous-time check.\n');
  else
    fprintf('Candidate failed. No final alpha established.\n');
  end

  output = struct( ...
      'alpha_candidate', alpha_candidate, ...
      'passed', passed, ...
      'n_samples', n_samples, ...
      'n_checked', n_checked, ...
      'n_failed', n_failed, ...
      'worst_normalized_residual', worst_ratio, ...
      'worst_reference', worst_reference, ...
      'worst_error', worst_error, ...
      'worst_level', worst_level, ...
      'initial_rng_state', initial_rng_state);

  save(fullfile(output_dir, 'alpha1_check.mat'), 'output');
end


function f = dynamics(x, u)

  psi = x(3);
  v = x(4);

  a = u(1);
  omega = u(2);

  f = [
    v*cos(psi);
    v*sin(psi);
    omega;
    a
  ];
end


function X_dot = reference_xdot(X_coef, psi, v, a, omega)

  theta_dot = [
    -sin(psi)*omega;
     cos(psi)*omega;
     a*sin(psi) + v*cos(psi)*omega;
     a*cos(psi) - v*sin(psi)*omega
  ];

  X_dot = zeros(4);

  for j = 1:4
    X_dot = X_dot + theta_dot(j)*X_coef(:,:,j+1);
  end

  X_dot = (X_dot + X_dot')/2;
end

%}
%%
function output = jackal_check_alpha1(alpha_candidate, n_samples, shrink)
% Discrete-time local stability level alpha1.
%
% Paper (Koehler et al.):
%   Algorithm 1, decrease condition (4a), grid (r,r+) as in (23).
%   "Starting with alpha1 = alpha2, the value alpha1 is iteratively
%    decreased until all considered combinations (r, r+, x_i) satisfy (4a)."
%
% Model: one RK4 step, h = design.cfg.h, zero-order hold (TMPC model).
% Stage cost: h*(||e||_Q^2 + ||K e||_R^2) (TMPC cost scaling).
% Level convention: ||e||_P^2 <= alpha (Koehler), radius = sqrt(alpha).
%
% Usage:
%   output = jackal_check_alpha1([], 100);        % start at alpha2
%   output = jackal_check_alpha1([], 100, 0.9);   % shrink factor
%
% Finite sampling does not establish a continuous-domain guarantee.

  if nargin < 3 || isempty(shrink)
    shrink = 0.9;
  end

  validateattributes(n_samples, {'numeric'}, {'scalar', 'integer', 'positive'});
  validateattributes(shrink, {'numeric'}, {'scalar', 'real', '>', 0, '<', 1});

  output_dir = fullfile(getenv('HOME'), 'HMPC_code', 'jackal_hmpc', 'src', 'mpc', 'mpc_solver', 'scripts', 'include', 'offline_computations', 'mpc-sdp');

  loaded_design = load(fullfile(output_dir, 'P_K_reference_dependent.mat'), 'result');
  loaded_alpha2 = load(fullfile(output_dir, 'alpha2_reference_dependent.mat'), 'output');

  design = loaded_design.result;
  constraint_result = loaded_alpha2.output;

  if isempty(alpha_candidate)
    alpha_candidate = constraint_result.alpha2;
  end

  validateattributes(alpha_candidate, {'numeric'}, ...
      {'scalar', 'real', 'finite', 'positive'});

  if alpha_candidate > constraint_result.alpha2
    error('Candidate must not exceed the calculated alpha2.');
  end

  h = design.cfg.h;
  Q = design.cfg.Q;
  R = design.cfg.R;

  % Reference grid (r) from the alpha2 computation.
  psi_grid = constraint_result.psi_grid;
  v_grid = constraint_result.v_grid;
  v_intervals = constraint_result.reference_v_intervals;

  a_bounds = [constraint_result.reference_lb(4), constraint_result.reference_ub(4)];
  omega_bounds = [constraint_result.reference_lb(5), constraint_result.reference_ub(5)];

  % (4a) is nonlinear in u_r, so u_r is gridded.
  n_a = 5;
  n_omega = 5;

  % Record the random state without changing it.
  initial_rng_state = rng;

  max_iterations = 60;
  alpha_level = alpha_candidate;
  history = zeros(0, 3);

  for iteration = 1:max_iterations

    result = check_level(alpha_level, design, psi_grid, v_grid, ...
        a_bounds, omega_bounds, v_intervals, n_a, n_omega, n_samples, h, Q, R);

    history(end+1, :) = [alpha_level, result.n_failed, result.worst_ratio]; %#ok<AGROW>

    fprintf('Level %.9g (radius %.6g): %d of %d samples fail (4a), max normalized residual %.3e\n', ...
        alpha_level, sqrt(alpha_level), result.n_failed, result.n_checked, result.worst_ratio);

    if result.n_failed == 0
      break;
    end

    alpha_level = shrink*alpha_level;
  end

  passed = result.n_failed == 0;

  fprintf('\nalpha1 level: %.15g (radius %.15g)\n', alpha_level, sqrt(alpha_level));
  fprintf('Worst [psi,v,a,omega]: [%.15g, %.15g, %.15g, %.15g]\n', result.worst_reference);

  if passed
    fprintf('Level passed the sampled discrete-time check (Algorithm 1).\n');
  else
    fprintf('No passing level found within %d iterations.\n', max_iterations);
  end

  % alpha_candidate holds the final level (used by jackal_export_offline).
  output = struct( ...
      'alpha_candidate', alpha_level, ...
      'alpha1', alpha_level, ...
      'alpha_start', alpha_candidate, ...
      'shrink', shrink, ...
      'history', history, ...
      'passed', passed, ...
      'n_samples', n_samples, ...
      'n_checked', result.n_checked, ...
      'n_failed', result.n_failed, ...
      'worst_normalized_residual', result.worst_ratio, ...
      'worst_reference', result.worst_reference, ...
      'worst_error', result.worst_error, ...
      'n_a', n_a, ...
      'n_omega', n_omega, ...
      'initial_rng_state', initial_rng_state);

  save(fullfile(output_dir, 'alpha1_check.mat'), 'output');
end


function r = check_level(level, design, psi_grid, v_grid, ...
    a_bounds, omega_bounds, v_intervals, n_a, n_omega, n_samples, h, Q, R)

  r.n_checked = 0;
  r.n_failed = 0;
  r.worst_ratio = -Inf;
  r.worst_reference = NaN(1, 4);
  r.worst_error = NaN(4, 1);

  omega_grid = linspace(omega_bounds(1), omega_bounds(2), n_omega);

  for psi = psi_grid
    for v = v_grid

      pair = jackal_terminal_design('evaluate', design, psi, v);

      P = pair.P;
      K = pair.K;

      % P = S'*S, so e = S\z maps a sphere to the ellipsoid.
      S = chol(P);

      directions = randn(4, n_samples);
      lengths = sqrt(sum(directions.^2, 1));

      while any(lengths == 0)
        zero_columns = lengths == 0;
        directions(:, zero_columns) = randn(4, sum(zero_columns));
        lengths = sqrt(sum(directions.^2, 1));
      end

      directions = directions ./ lengths;

      % Boundary errors and uniform interior errors (radius rand^(1/4) in 4D).
      boundary_errors = sqrt(level)*(S\directions);
      radii = rand(1, n_samples).^(1/4);
      errors = [boundary_errors, boundary_errors .* radii];

      KE = K*errors;

      V = sum(errors .* (P*errors), 1);
      stage = h*(sum(errors .* (Q*errors), 1) + sum(KE .* (R*KE), 1));

      % Position can be zero: the model and P,K do not depend on it.
      xr = [0; 0; psi; v];

      for a = admissible_a(v, a_bounds, v_intervals, h, n_a)
        for omega = omega_grid

          ur = [a; omega];

          % Reference step r+ (Assumption 1: x_r+ = f(x_r, u_r)).
          xr_next = rk4_step(xr, ur, h);

          pair_next = jackal_terminal_design('evaluate', design, xr_next(3), xr_next(4));

          % Closed loop with k_f(x,r) = u_r + K(r)(x - x_r).
          x_next = rk4_step(xr + errors, ur + KE, h);
          e_next = x_next - xr_next;

          V_next = sum(e_next .* (pair_next.P*e_next), 1);

          % Equation (4a) requires V_next - V + stage <= 0.
          ratio = (V_next - V + stage) ./ stage;
          ratio(~isfinite(ratio)) = Inf;

          r.n_checked = r.n_checked + numel(ratio);
          r.n_failed = r.n_failed + nnz(ratio > 0);

          [worst, j] = max(ratio);

          if worst > r.worst_ratio
            r.worst_ratio = worst;
            r.worst_reference = [psi, v, a, omega];
            r.worst_error = errors(:, j);
          end
        end
      end
    end
  end
end


function a_set = admissible_a(v, a_bounds, v_intervals, h, n_a)
% Reference inputs a_r with r+ inside the reference region, equation (23).
  tol = 1e-9;
  k = find(v >= v_intervals(:,1) - tol & v <= v_intervals(:,2) + tol, 1);

  if isempty(k)
    a_set = [];
    return;
  end

  a_lo = max(a_bounds(1), (v_intervals(k,1) - v)/h);
  a_hi = min(a_bounds(2), (v_intervals(k,2) - v)/h);

  if a_lo > a_hi + tol
    a_set = [];
  else
    a_set = unique(linspace(a_lo, max(a_lo, a_hi), n_a));
  end
end


function x_next = rk4_step(x, u, h)
% One RK4 step with zero-order hold, column-wise for x (4xN), u (2xN).
  k1 = dynamics(x, u);
  k2 = dynamics(x + h/2*k1, u);
  k3 = dynamics(x + h/2*k2, u);
  k4 = dynamics(x + h*k3, u);

  x_next = x + h/6*(k1 + 2*k2 + 2*k3 + k4);
end


function f = dynamics(x, u)
% State [p_x; p_y; psi; v], input [a; omega].
  f = [
    x(4,:) .* cos(x(3,:));
    x(4,:) .* sin(x(3,:));
    u(2,:) .* ones(1, size(x, 2));
    u(1,:) .* ones(1, size(x, 2))
  ];
end
