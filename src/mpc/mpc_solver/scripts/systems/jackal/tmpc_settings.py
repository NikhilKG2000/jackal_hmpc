import numpy as np

from include import (
    control_modules,
    dynamics,
    helpers,
    objective,
    systems,
)

from include.offline_computations import offline_computations


configuration = "JackalRMPC"


# Model options
model_options = {
    "use_input_rates": False,
    "use_slack": False,
    "use_ct_feedback_law": False,
    "use_dt_feedback_law": False,
    "use_terminal_set_constraint": True,
}


# Model and offline ingredients
robot = systems.Jackal()
model = dynamics.JackalModel(robot)

offline_comp = offline_computations.JackalOffline()

Q = offline_comp.get_Q()
R = offline_comp.get_R()


# Prediction and integration
N = 10
ratio = 10
n_discs = 1

integrator_options = {
    "stepsize": 0.05,
    "steps": 1,
    "use_ct_feedback_law": False,
    "use_dt_feedback_law": False,
}

initialize_input_runtime = False
print_level = 0
maximum_iterations = 300

hardcode_weights = True


# Constants exported to C++
constants = helpers.ConstantsStructure()

constants.add_constant(
    "ratio",
    np.array(ratio),
)

constants.add_constant(
    "use_input_rates",
    np.array(int(model_options["use_input_rates"])),
)

constants.add_constant(
    "use_slack",
    np.array(int(model_options["use_slack"])),
)

constants.add_constant(
    "integrator_stepsize",
    np.array(integrator_options["stepsize"]),
)

constants.add_constant(
    "integrator_steps",
    np.array(integrator_options["steps"]),
)

constants.add_constant(
    "Q",
    np.array(Q),
)

constants.add_constant(
    "R",
    np.array(R),
)

offline_comp.add_reference_constants(constants)

constants.add_constant("alpha", np.array(offline_comp.get_alpha()),)

# Runtime parameters and modules
params = helpers.ParameterStructure(N)
modules = control_modules.ModuleManager(N, params)


# State/input reference objective
modules.add_module(
    control_modules.ReferenceTrajectoryModule(
        params,
        model,
    )
)


def use_objective(stage_idx, z, p, settings):
    params.load_objectives_params(stage_idx, p)

    return objective.jackal_tmpc_objective(
        stage_idx, z, p, settings
    )


# Stage weights follow the model ordering
weight_dict = {}

for idx, name in enumerate(model.inputs):
    weight_dict["R_" + name] = R[idx, idx]

for idx, name in enumerate(model.states):
    weight_dict["Q_" + name] = Q[idx, idx]

weights = helpers.Weights(
    params,
    weight_dict,
    hardcode_weights,
)


# Terminal tracking-error ellipsoid
modules.add_module(
    control_modules.TerminalConstraintsSet(
        offline_comp,
        model_options["use_slack"],
    ),
    N,
)


# Obstacle constraints at stages 1 through N
modules.add_module(
    control_modules.StaticPolyhedronConstraintModule(
        params,
        n_discs,
        model_options["use_slack"],
    ),
    list(range(1, N + 1)),
)


npar = params.get_number_of_parameters()
nh = modules.get_number_of_constraints()

print(constants)
print(f"Number of parameters: {npar}")
print(f"Number of constraints: {nh}")