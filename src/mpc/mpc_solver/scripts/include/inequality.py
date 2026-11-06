import casadi
import helpers
import numpy as np
import objective


""" 
Defines inequality constraints of different types. 
See control_modules.py for their integration into the controller 
"""

# Class to keep track of:
# - Which constraints are applied to which stages
# - The corresponding number of constraints nh
# - The corresponding number of parameters npar
class ConstraintManager:
    def __init__(self, N, params):
        # Initialization of constraint types bookkeeping over the prediction horizon
        self.constraints = [[] for _ in range(N + 1)]

        # Initialization of lower- and upper-bounds bookkeeping
        self.lower_bound = [[] for _ in range(N + 1)]
        self.upper_bound = [[] for _ in range(N + 1)]

        # Initialization of number of constraints per stage bookkeeping
        self.nh = [0] * (N + 1)

        # Initialization of parameter bookkeeping
        # self.npar = 0
        self.params = params
        # self.param_idx = param_idx

    def add_constraint(self, constraint, stages):
        # Make sure stages is iterable
        if not hasattr(stages, '__iter__'):
            stages = [stages]

        # Add constraint in all stages in which it should be applied
        for stage_idx in stages:
            self.constraints[stage_idx].append(constraint)
            # constraint.add_parameters(stage_idx)
            constraint.append_upper_bound(self.upper_bound[stage_idx])
            constraint.append_lower_bound(self.lower_bound[stage_idx])
            self.nh[stage_idx] = self.nh[stage_idx] + constraint.nh
        # For now, use the same parameters in every stage
        constraint.add_parameters()

    def get_constraints(self, stage_idx, z, settings):
        constraints = []

        for constraint in self.constraints[stage_idx]:
            constraint.append_constraints(constraints, z, settings)

        return constraints


# Constraints used in robust MPC to tighten system constraints

class TightenedSystemConstraints:
    def __init__(self, nvar, offline_comp):
        self.c_s = np.asarray(
            offline_comp.get_c_s(), dtype=float
        ).reshape(-1)
        self.alpha = float(offline_comp.get_alpha())
        self.nh = 2 * nvar

        if nvar != 6 or self.c_s.size != self.nh:
            raise ValueError(
                "Expected six Jackal variables and twelve margins"
            )

    def add_parameters(self):
        pass

    def append_lower_bound(self, lower_bound):
        lower_bound.extend([-np.inf] * self.nh)

    def append_upper_bound(self, upper_bound):
        upper_bound.extend([0.0] * self.nh)

    def append_constraints(self, constraints, z, settings):
        names = settings.model.inputs + settings.model.states

        for idx, name in enumerate(names):
            value = z[idx]

            lb = settings.robot.lower_bound[name]
            ub = settings.robot.upper_bound[name]

            lower_margin = self.c_s[2 * idx] * self.alpha
            upper_margin = self.c_s[2 * idx + 1] * self.alpha
            
            constraints.append(lb - value + lower_margin)
            constraints.append(value - ub + upper_margin)


class TerminalConstraintsSteadyState:
    def __init__(self, use_slack=False):
        if use_slack:
            raise ValueError("The Jackal model has no slack input")

        self.nh = 3
    
    def add_parameters(self):
        pass

    def append_lower_bound(self, lower_bound):
        lower_bound.extend([0.0] * self.nh)
    
    def append_upper_bound(self, upper_bound):
        upper_bound.extend([0.0] * self.nh)
    
    def append_constraints(self, constraints, z, settings):
        constraints.extend([
            settings.model.get_state(z, "v", True),
            settings.model.get_input(z, "a", True),
            settings.model.get_input(z, "omega", True),
        ])


class TerminalConstraintsSet:
    def __init__(self, offline_comp, use_slack = False):
        if use_slack:
            raise ValueError("The Jackal model has no slack input")
        
        self.nh = 1
        self.alpha = float(offline_comp.get_alpha())

    def add_parameters(self):
        pass

    def append_lower_bound(self, lower_bound):
        lower_bound.append(-np.inf)

    def append_upper_bound(self, upper_bound):
        upper_bound.append(0.0)

    def append_constraints(self, constraints, z, settings):
        constraints.append(objective.terminal_tracking_error(z, settings)-self.alpha**2)


# Removed "Reuse constraints" because you can now define the parameters by name
# Constraints of the form Ax <= b (+ slack)
class LinearConstraints:

    def __init__(self, params, n_discs, num_constraints, use_slack=False):
        self.params = params
        self.num_constraints = num_constraints
        self.n_discs = n_discs
        self.nh = num_constraints * n_discs
        self.use_slack = use_slack
        
    def add_parameters(self):
        for disc in range(self.n_discs):
            for i in range(self.num_constraints):
                self.params.add_parameter(self.constraint_name(disc, i) + "_a1", "constraints")
                self.params.add_parameter(self.constraint_name(disc, i) + "_a2", "constraints")
                self.params.add_parameter(self.constraint_name(disc, i) + "_b", "constraints")

    def constraint_name(self, disc_idx, constraint_idx):
        return "disc"+str(disc_idx)+"_"+str(constraint_idx)

    def append_lower_bound(self, lower_bound):
        for constraint in range(self.num_constraints):
            for disc in range(self.n_discs):
                lower_bound.append(-np.inf)

    def append_upper_bound(self, upper_bound):
        for constraint in range(self.num_constraints):
            for disc in range(self.n_discs):
                upper_bound.append(0.0)

    def append_constraints(self, constraints, z, settings):
        # Retrieve variables
        x = z[settings.model.nu:settings.model.nu + settings.model.nx]
        u = z[0:settings.model.nu]
        if self.use_slack:
            slack = u[-1]

        # States
        pos_x = settings.model.get_state(z, 'x', True)
        pos_y = settings.model.get_state(z, 'y', True)
        pos = np.array([pos_x, pos_y])

        for disc_it in range(self.n_discs):
            disc_pos = pos

            # A'x <= b
            for constraint_it in range(self.num_constraints):
                a1 = getattr(settings.params, self.constraint_name(disc_it, constraint_it) + "_a1")
                a2 = getattr(settings.params, self.constraint_name(disc_it, constraint_it) + "_a2")
                b =  getattr(settings.params, self.constraint_name(disc_it, constraint_it) + "_b")

                if self.use_slack:
                    constraints.append(a1 * disc_pos[0] + a2 * disc_pos[1] - b - slack)
                else:
                    constraints.append(a1 * disc_pos[0] + a2 * disc_pos[1] - b)
