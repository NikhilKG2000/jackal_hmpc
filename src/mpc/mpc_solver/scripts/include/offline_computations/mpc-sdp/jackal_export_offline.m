function offline = jackal_export_offline()
% Requires:
%   jackal_terminal_design.m
%   P_K_reference_dependent.mat
%   alpha2_reference_dependent.mat
%   alpha1_check.mat
%
% Exports reference-dependent matrices; no constant P,K substitution.
% c_s,c_o are maxima over the finite reference grid.

output_dir = fullfile(getenv('HOME'), 'HMPC_code', 'jackal_hmpc', 'src', 'mpc', 'mpc_solver', 'scripts', 'include', 'offline_computations', 'mpc-sdp');
d = load(fullfile(output_dir,'P_K_reference_dependent.mat'), 'result');
a2 = load(fullfile(output_dir, 'alpha2_reference_dependent.mat'), 'output');
a1 = load(fullfile(output_dir, 'alpha1_check.mat'), 'output');

design = d.result; 
region = a2.output;
check = a1.output; 

if ~design.check.passed || ~design.design_check.passed || ~check.passed
    error('Matrix or nonlinear sampled checks did not pass');
end

alpha_level = min(region.alpha2, check.alpha_candidate);

if ~isfinite(alpha_level) || alpha_level <= 0
    error('Terminal level must be finite and positive');
end

alpha_radius = sqrt(alpha_level);

c_s_names = {'a'; 'omega'; 'p_x'; 'p_y'; 'psi'; 'v'};

c_s = zeros(6,1);
c_o = 0;

C = [eye(2), zeros(2,2)];

c_s_grid = zeros(6, numel(region.psi_grid), numel(region.v_grid));

c_o_grid = zeros(numel(region.psi_grid), numel(region.v_grid));

% evaluate coefficient at each reference
for i = 1:numel(region.psi_grid)
    for j = 1:numel(region.v_grid)
         
        pair = jackal_terminal_design('evaluate', design, region.psi_grid(i), region.v_grid(j));
        
        X = pair.X;
        K = pair.K;
        
        assert(isequal(size(X), [4,4]), 'X must be 4-by-4.');
        assert(isequal(size(K), [2,4]), 'K must be 2-by-4.');
        
        % Combined input and state error:
        % [delta_a; delta_omega; delta_px; delta_py; delta_psi; delta_v]
        G = [K; eye(4)];                % 6-by-4
        
        covariance = G*X*G';            % 6-by-6
        covariance = (covariance + covariance')/2;
        
        % One coefficient for each input/state component.
        local_s = sqrt(diag(covariance));   % 6-by-1
        
        % Position-only coefficient for obstacle tightening.
        C = [eye(2), zeros(2,2)];
        
        position_covariance = C*X*C';       % 2-by-2
        position_covariance = ...
            (position_covariance + position_covariance')/2;
        
        local_o = sqrt(max(eig(position_covariance)));  % scalar
        
        c_s_grid(:,i,j) = local_s;
        c_o_grid(i,j) = local_o;
        
        c_s = max(c_s, local_s);
        c_o = max(c_o, local_o);
    end
end

Q = design.cfg.Q;
R = design.cfg.R;

X_coef = design.X_coef;
Y_coef = design.Y_coef; 

alpha = alpha_radius; 

system_margins = c_s*alpha_radius; 
obstacle_margin = c_o*alpha_radius; 

psi_grid = region.psi_grid; 
v_grid = region.v_grid; 

validation = struct( ...
    'matrix_design', design.design_check, ...
    'matrix_independent', design.check, ...
    'nonlinear_continuous_sampled', check, ...
    'sampled_controller_verified', false);

offline = struct( ...
    'Q', Q, ...
    'R', R, ...
    'X_coef', X_coef, ...
    'Y_coef', Y_coef, ...
    'alpha', alpha, ...
    'alpha_level', alpha_level, ...
    'alpha_radius', alpha_radius, ...
    'c_s', c_s, ...
    'c_o', c_o, ...
    'c_s_names', {c_s_names}, ...
    'system_margins', system_margins, ...
    'obstacle_margin', obstacle_margin, ...
    'c_s_grid', c_s_grid, ...
    'c_o_grid', c_o_grid, ...
    'reference_region', region, ...
    'validation', validation);

save(fullfile(output_dir, 'offline_comps_tracking_reference_dependent.mat'), ...
     'Q', 'R', 'X_coef', 'Y_coef', ...
     'alpha', 'alpha_level', 'alpha_radius', ...
     'c_s', 'c_o', 'c_s_names', ...
     'system_margins', 'obstacle_margin', ...
     'c_s_grid', 'c_o_grid', ...
     'psi_grid', 'v_grid', 'region', 'validation');

% Print
fprintf('alpha_level: %.15g\n', alpha_level);

fprintf('alpha_radius (exported alpha): %.15g\n', ...
    alpha_radius);

fprintf('\nComponent    c_s              tightening margin\n');

for j = 1:6
    fprintf('%-10s   %.9g       %.9g\n', ...
        c_s_names{j}, c_s(j), system_margins(j));
end

fprintf('c_o: %.15g\n', c_o);

fprintf('Position/obstacle margin: %.15g m\n', ...
    obstacle_margin);

fprintf('Saved offline_comps_tracking_reference_dependent.mat\n');
end

