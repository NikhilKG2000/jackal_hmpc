import casadi as ca


def terminal_tracking_error(z, settings):
    nu = settings.model.nu
    nx = settings.model.nx

    x = z[nu:nu + nx]

    x_ref = ca.vertcat(*[
        getattr(settings.params, f"u_x_ref_{nu + i}")
        for i in range(nx)
    ])

    error = x - x_ref

    P = settings.offline_comp.get_P_delta(
        x_ref[2],  # Reference heading.
        x_ref[3],  # Reference speed.
    )

    return ca.mtimes([error.T, P, error])


def jackal_tmpc_objective(stage_idx, z, p, settings):
    x = z[settings.model.nu:settings.model.nvar]
    u = z[:settings.model.nu]

    cost = 0.0

    for module in settings.modules.modules:
        if module.type == "objective":
            for item in module.objectives:
                cost += item.get_value(
                    x, u, p, settings, stage_idx
                )

    return cost


def jackal_go_objective(stage_idx, z, p, settings):
    cost = 0.0

    for module in settings.modules.modules:
        if module.type == "objective":
            for item in module.objectives:
                cost += item.get_value(
                    z, p, settings, stage_idx
                )

    suffix = "_terminal" if stage_idx == settings.N else ""

    for name in ("a", "omega"):
        value = settings.model.get_input(
            z, name, required=True
        )

        weight = settings.weights.get_value(
            p,
            stage_idx,
            "input_" + name + suffix,
        )

        cost += weight * value**2

    return cost


class GoalOrientedObjective:
    def __init__(self, params):
        params.add_parameter("goal_x", "objectives")
        params.add_parameter("goal_y", "objectives")

    def get_value(self, z, p, settings, stage_idx):
        dx = (
            settings.model.get_state(z, "x", True)
            - settings.params.goal_x
        )

        dy = (
            settings.model.get_state(z, "y", True)
            - settings.params.goal_y
        )

        distance_squared = dx**2 + dy**2
        delta = 0.5

        outer_distance = ca.sqrt(
            ca.fmax(distance_squared, delta**2)
        )

        huber = ca.if_else(
            distance_squared < delta**2,
            0.5 * distance_squared,
            delta * (outer_distance - 0.5 * delta),
        )

        name = (
            "goal_xy"
            if stage_idx == settings.N
            else "goal_xy_all"
        )

        weight = settings.weights.get_value(
            p, stage_idx, name
        )

        return weight * huber


class ReferenceTrajectoryObjective:
    def __init__(self, params, model):
        self.model = model
        self.nu = model.nu
        self.nx = model.nx

        params.add_multiple_parameters(
            "u_x_ref",
            self.nu + self.nx,
            "objectives",
        )

    def get_value(self, x, u, p, settings, stage_idx):
        if stage_idx == settings.N:
            return terminal_tracking_error(
                ca.vertcat(u, x), settings
            )

        cost = 0.0

        for i, name in enumerate(self.model.inputs):
            reference = getattr(
                settings.params, f"u_x_ref_{i}"
            )

            weight = settings.weights.get_value(
                p, stage_idx, "R_" + name
            )

            cost += weight * (u[i] - reference)**2

        for i, name in enumerate(self.model.states):
            reference = getattr(
                settings.params,
                f"u_x_ref_{self.nu + i}",
            )

            weight = settings.weights.get_value(
                p, stage_idx, "Q_" + name
            )

            cost += weight * (x[i] - reference)**2

        return cost