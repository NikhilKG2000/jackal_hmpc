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