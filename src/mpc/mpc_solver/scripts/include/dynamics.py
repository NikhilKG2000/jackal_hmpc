import casadi
import numpy as np




# Custom RK4 implementation
def solve_rk4(continuous_model, x, u, param, settings):
    dt = settings.integrator_options['stepsize']
    n_steps = settings.integrator_options['steps']
    for i in range(n_steps):
        x = rk4_step(continuous_model, x, u, param, settings, dt)
    return u, x

def rk4_step(continuous_model, x, u, param, settings, dt):
    k1 = continuous_model(x, u, param, settings)
    k2 = continuous_model(x + dt / 2 * k1, u, param, settings)
    k3 = continuous_model(x + dt / 2 * k2, u, param, settings)
    k4 = continuous_model(x + dt * k3, u, param, settings)
    return x + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6

# Dynamics, i.e. equality constraints #
# This class contains models to choose from
# They can be coupled with physical limits using Systems defined in systems.py
# See JackalModel for an example of the required parameters
class DynamicModel:

    def __init__(self, system):
        self.nvar = self.nu + self.nx
        self.system = system
        self.control_inputs = dict()
        self.possible_inputs_to_vehicle = []

    def __str__(self):
        result = 'Dynamical Model: ' + str(type(self)) + '\n' +\
               'System: ' + str(self.system) + '\n'

        if hasattr(self, 'interfaces'):
            result += 'Interfaces: '

            for interface in self.interfaces:
                result += interface + " "

            result += "\n"

        result += 'States: ' + str(self.states) + '\n'
        result += 'Inputs: ' + str(self.inputs) + '\n'
        return result

    # Appends upper bounds from system
    def upper_bound(self):
        result = np.array([])

        for input in self.inputs:
            result = np.append(result, self.system.upper_bound[input])

        for state in self.states:
            result = np.append(result, self.system.upper_bound[state])

        return result

    # Appends lower bounds from system
    def lower_bound(self):
        result = np.array([])

        for input in self.inputs:
            result = np.append(result, self.system.lower_bound[input])

        for state in self.states:
            result = np.append(result, self.system.lower_bound[state])

        return result

    def get_state(self, z, state_name, required=False):
        if state_name not in self.states:
            assert not required, f"Required state {state_name} was not part of the model"
            return 0.0
        return z[self.nu + self.states.index(state_name)]

    def get_input(self, z, input_name, required=False):
        if input_name not in self.inputs:
            assert not required, f"Required input {input_name} was not part of the model"
            return 0.0
        return z[self.inputs.index(input_name)]

    def get_z_index(self, z, name, required=False):
        if (name not in self.inputs) and (name not in self.states):
            assert not required, f"Required variable {name} was not part of the model"
            return 0.
        if name in self.inputs:
            return z[self.inputs.index(name)]
        else:
            return z[self.nu + self.states.index(name)]

    def get_idx(self, name, required=False):
        if (name not in self.inputs) and (name not in self.states):
            # print("Unknown variable {} was indexed! Setting to zero".format(name))
            assert not required, f"Required variable {name} was not part of the model"
            return 0.
        if name in self.inputs:
            return self.inputs.index(name)
        else:
            return self.nu + self.states.index(name)

class JackalModel(DynamicModel):
    
    def __init__(self,system):
        self.states = ["x", "y", "psi", "v"]
        self.inputs = ["a", "omega"]

        self.nx = len(self.states)
        self.nu = len(self.inputs)

        super().__init__(system)

    def continuous_model(self, x, u, param, settings):
        psi = x[2]
        v = x[3]

        a = u[0]
        omega = u[1]

        return casadi.vertcat(v*casadi.cos(psi), v*casadi.sin(psi), omega, a,)

    def discretize_dynamics(self, z, param, settings):
        u = z[:self.nu]
        x = z[self.nu:self.nvar]

        _, next_x = solve_rk4(self.continuous_model, x, u, param, settings,)

        return next_x
