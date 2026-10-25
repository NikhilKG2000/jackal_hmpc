import numpy as np
from include import control_modules, dynamics, helpers, objective, systems
from include.offline_computations import offline_computations
from .weights import PMPC_WEIGHTS

configuration  = "JackalGO"

# Model Options
model_options = {
    "use_input_rates": False,
    "use_slack": False,
    "use_ct_feedback_law": False,
    "use_dt_feedback_law": False,
    "use_tightened_system_constraints": True,
    "use_tightened_obstacle_constraints": True,
    "use_terminal_steady_state_constraint": True,
}

# Model and offline ingredients
robot = systems.Jackal()
model = dynamics.JackalModel(robot)
offline_comp = offline_computations.JackalOffline()

# Prediction and integration
N = 8
N_total = N + 1
n_discs = 1

integrator_options = {
    "stepsize": 0.05,
    "steps": 10,
    "use_ct_feedback_law": False,
    "use_dt_feedback_law": False,
}

initialize_input_runtime = False
print_level = 0
maximum_iterations = 300

hardcode_weights = False

# Constants exported to C++
constants = helpers.ConstantsStructure()
constants.add_constant("use_input_rates", np.array(int(model_options["use_input_rates"])),)
constants.add_constant("use_slack", np.array(int(model_options["use_slack"])),)
constants.add_constant("use_tightened_obstacle_constraints", np.array(int(model_options["use_tightened_obstacle_constraints"])),)
constants.add_constant("c_o", np.array(offline_comp.get_c_o()),)
constants.add_constant("alpha", np.array(offline_comp.get_alpha()),)
offline_comp.add_reference_constants(constants)
constants.add_constant("n_states_to_constrain", np.array(N_total -  N + 1),)

# Run time parameters and modules
params = helpers.ParameterStructure(N)
modules = control_modules.ModuleManager(N, params)

# Goal Objective
modules.add_module(control_modules.GoalOrientedModule(params))

def use_objective(stage_idx, z, p, settings):
    params.load_objectives_params(stage_idx, p)
    
    return objective.jackal_go_objective(stage_idx, z, p, settings)

#Run time weights 
weight_dict = PMPC_WEIGHTS.copy()
weights = helpers.Weights(params, weight_dict, hardcode_weights,)

# Tightened system constraints at every optimization node
modules.add_module(control_modules.TightenedSystemConstraints(params, model.nvar,offline_comp,), list(range(N)),)
# Obstacle constraints at stages 1 through N
modules.add_module(control_modules.StaticPolyhedronConstraintModule(params, n_discs, model_options["use_slack"],), list(range(1, N+1)),)

# Terminal stady-state condition
modules.add_module(control_modules.TerminalConstraintsSteadyState(model_options["use_slack"],),N,)

npar = params.get_number_of_parameters()
nh = modules.get_number_of_constraints()

print(constants)
print(f"Number of parameters: {npar}")
print(f"Number of constrints: {nh}")


