function output = jackal_alpha2()
    
    output_dir = fullfile(getenv('HOME'), 'HMPC_code', 'jackal_hmpc', 'src', 'mpc', 'mpc_solver', 'scripts', 'include', 'offline_computations', 'mpc-sdp');
    loaded = load(fullfile(output_dir, 'P_K_reference_dependent.mat'), 'result');
    design = loaded.result;

    if ~design.check.passed || ~design.design_check.passed
        error('The saved matrix did not pass both grid checks');
    end


    names = {'p_x'; 'p_y'; 'v'; 'a'; 'omega'};

    physical_lb = [-15; -15;  -1.5; -2.0; -2.0];
    physical_ub = [ 15;  15;   1.5;  2.0;  2.0];

    reference_lb = [-14; -14; -1.0; -1.0; -1.0];
    reference_ub = [ 14;  14;  1.0;  1.0;  1.0];

    % The reference speed may not be near zero, so it has two intervals.
    reference_v_intervals = [-1.0, -0.2;
                              0.2,  1.0];

    psi_bounds = [-pi, pi];

    %
    % Ensure every bound is a column vector.
    physical_lb  = physical_lb(:);
    physical_ub  = physical_ub(:);
    reference_lb = reference_lb(:);
    reference_ub = reference_ub(:);

    % Check the number of entries before comparing.
    assert(numel(physical_lb) == 5 && ...
        numel(physical_ub) == 5 && ...
        numel(reference_lb) == 5 && ...
        numel(reference_ub) == 5, ...
        'Each bounds vector must contain five entries.');

    if any(reference_lb < physical_lb) || any(reference_ub > physical_ub)
        error('Reference bounds exceed the physical constraints.');
    end

    % Each reference speed interval must lie inside one design speed interval.
    v_inside = true;
    for i = 1:size(reference_v_intervals, 1)
        v_inside = v_inside && any( ...
            reference_v_intervals(i,1) >= design.cfg.v_intervals(:,1) & ...
            reference_v_intervals(i,2) <= design.cfg.v_intervals(:,2));
    end
    
    if ~v_inside || ...
            reference_lb(4) < design.cfg.a_bounds(1) || ...
            reference_ub(4) > design.cfg.a_bounds(2) || ...
            reference_lb(5) < design.cfg.omega_bounds(1) || ...
            reference_ub(5) > design.cfg.omega_bounds(2)
    
        error('Reference region exceeds the matrix-design region.');
    end


    % Reference grid

    % Reuse previous independent - check grid counts. Include the endpoints
    % of the new reference region. 

    psi_grid = linspace(psi_bounds(1), psi_bounds(2), design.cfg.npsi_check);
    v_grid = [];
    for i = 1:size(reference_v_intervals, 1)
        v_grid = [v_grid, linspace(reference_v_intervals(i,1), reference_v_intervals(i,2), design.cfg.nv_check)]; %#ok<AGROW>
    end

    % Physical constraint rows
    selected = [1,2,4,5,6];
    L = zeros(10, 6);

    for j = 1:5
        L(2*j-1, selected(j)) = -1;
        L(2*j, selected(j)) = 1;
    end

    % worst availabe slack

    % P and K depend on psi, v. For position input constraints, the
    % smallest slack occurs at the corresponding reference-bound endpoint. 

    slack = zeros(10, 1);
    slack(1:2:end) = reference_lb - physical_lb;
    slack(2:2:end) = physical_ub - reference_ub;

    per_constraint = Inf(10, 1);
    worst_psi_v = NaN(10,2);

    for psi = psi_grid
        for v = v_grid

            pair = jackal_terminal_design('evaluate', design, psi, v);

            X = pair.X;
            G = [eye(4);
                 pair.K];
            W = L*G;

            denominator = sum((W*X).*W, 2);

            if any(~isfinite(denominator)) || any(denominator < 0)
                error("Invalid ellipsoid support calculation");
            end

            local_slack = slack;

            local_slack(5) = v - physical_lb(3);
            local_slack(6) = physical_ub(3) - v;

            candidate = Inf(10, 1);
            active = denominator > 0;
            
            candidate(active) = local_slack(active).^2 ./ denominator(active);

            % A zero denominator means this constraint row has no error
            % deviation. With non-negative slack, it places no restriction
            % on the ellipsoid level
               
            improved = candidate < per_constraint; 
            per_constraint(improved, :) = candidate(improved);

            worst_psi_v(improved, :) = repmat([psi,v], sum(improved), 1);
        end
    end

    [alpha2, limiting_row] = min(per_constraint);

    if ~isfinite(alpha2) || alpha2 < 0
        error ('No finite non-negative alpha2 was obtained');
    end
    labels = cell(10, 1);

    for j = 1:5
        labels{2*j-1} = [names{j}, 'lower'];
        lables{2*j} = [names{j}, 'upper'];
    end

    fprintf('alpha2 (grid minimum): %.15g\n', alpha2);

    fprintf('Limiting constraint: %s\n', ...
        labels{limiting_row});

    fprintf('Worst [psi_r,v_r]: [%.15g, %.15g]\n', ...
        worst_psi_v(limiting_row,:));

    fprintf('\nIndividual constraint limits:\n');

    for j = 1:10
        fprintf('%-12s %.9g\n', labels{j}, per_constraint(j));
    end

    output = struct( ...
        'alpha2', alpha2, ...
        'labels', {labels}, ...
        'per_constraint', per_constraint, ...
        'worst_psi_v', worst_psi_v, ...
        'limiting_row', limiting_row, ...
        'physical_lb', physical_lb, ...
        'physical_ub', physical_ub, ...
        'reference_lb', reference_lb, ...
        'reference_ub', reference_ub, ...
        'psi_bounds', psi_bounds, ...
        'psi_grid', psi_grid, ...
        'v_grid', v_grid, ...
        'L', L);

    % Save the output variable
    save(fullfile(output_dir, 'alpha2_reference_dependent.mat'), 'output');
    
end