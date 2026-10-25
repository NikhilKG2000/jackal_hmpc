from pathlib import Path

import numpy as np
import casadi as ca
from scipy.io import loadmat 

class JackalOffline:
    def __init__(self): 
        filename = Path(__file__).resolve().parent / "mpc-sdp" / "offline_comps_tracking_reference_dependent.mat"

        data = loadmat(filename, squeeze_me = True)

        shapes = { 
            "Q": (4, 4),
            "R": (2, 2),
            "X_coef": (4, 4, 5),
            "Y_coef": (2, 4, 5),
        }

        for name, shape in shapes.items():
            value = np.asarray(data[name], dtype = float)

            if value.shape != shape or not np.all(np.isfinite(value)):
                raise ValueError(f"Invalid {name}: expected finite array {shape}")

            setattr(self, name, value)

        for name in ("Q", "R"):
            value = getattr(self, name)

            if (not np.allclose(value, np.diag(np.diag(value))) or np.any(np.diag(value) <= 0)):
                raise ValueError(f"{name} must be diagonal and positive definite")
        
        self.alpha = self._scalar(data, "alpha")
        self.c_o = self._scalar(data, "c_o")

        if self.alpha <= 0 or self.c_o < 0:
            raise ValueError("Require alpha > 0 and c_0 >= 0")

        names = [str(name).strip() for name in np.atleast_1d(data["c_s_names"])]

        if names != ["a", "omega", "p_x", "p_y", "psi", "v"]:
            raise ValueError("Unexpected MATLAB c_s ordering")

        c_s = np.asarray(data["c_s"], dtype = float).reshape(-1)

        if (c_s.size != 6 or not np.all(np.isfinite(c_s)) or np.any(c_s < 0)):
            raise ValueError("Expected six finite, non-negative c_s values")

        # Lower and upper inequalities for each variable:
        # [a_lb, a_ub, omega_lb, omega_ub, ..., v_lb, v_ub].
        self.c_s = np.repeat(c_s, 2)

        self.psi_grid = np.asarray(
            data["psi_grid"], dtype=float
        ).reshape(-1)

        self.v_grid = np.asarray(
            data["v_grid"], dtype=float
        ).reshape(-1)

    @staticmethod
    def _scalar(data, name):
        value = np.asarray(data[name], dtype=float)

        if value.size != 1 or not np.all(np.isfinite(value)):
            raise ValueError(f"{name} must be a finite scalar")

        return float(value.item())

    def evaluate_xy(self, psi_r, v_r):
        symbolic = (
            isinstance(psi_r, (ca.SX, ca.MX, ca.DM))
            or isinstance(v_r, (ca.SX, ca.MX, ca.DM))
        )

        if symbolic:
            basis = ca.vertcat(
                1,
                ca.cos(psi_r),
                ca.sin(psi_r),
                v_r * ca.sin(psi_r),
                v_r * ca.cos(psi_r),
            )

            X = ca.DM(self.X_coef[:, :, 0])
            Y = ca.DM(self.Y_coef[:, :, 0])

            for j in range(1, 5):
                X = X + basis[j] * ca.DM(self.X_coef[:, :, j])
                Y = Y + basis[j] * ca.DM(self.Y_coef[:, :, j])

        else:
            basis = np.array([
                1,
                np.cos(psi_r),
                np.sin(psi_r),
                v_r * np.sin(psi_r),
                v_r * np.cos(psi_r),
            ])

            X = np.tensordot(
                self.X_coef, basis, axes=(2, 0)
            )

            Y = np.tensordot(
                self.Y_coef, basis, axes=(2, 0)
            )

        return (X + X.T) / 2, Y

    def get_P_delta(self, psi_r, v_r):
        X, _ = self.evaluate_xy(psi_r, v_r)

        if isinstance(X, (ca.SX, ca.MX, ca.DM)):
            return ca.solve(X, ca.DM.eye(4))

        return np.linalg.solve(X, np.eye(4))

    def get_K_delta(self, psi_r, v_r):
        X, Y = self.evaluate_xy(psi_r, v_r)

        if isinstance(X, (ca.SX, ca.MX, ca.DM)):
            return ca.solve(X.T, Y.T).T

        return np.linalg.solve(X.T, Y.T).T

    def add_reference_constants(self, constants):
        constants.add_constant(
            "reference_dependent_terminal", np.array(1)
        )

        # ConstantsStructure supports arrays with at most two
        # dimensions, so export each coefficient separately.
        for j in range(5):
            constants.add_constant(
                f"X_coef_{j}", self.X_coef[:, :, j]
            )

            constants.add_constant(
                f"Y_coef_{j}", self.Y_coef[:, :, j]
            )

    def get_Q(self):
        return self.Q

    def get_R(self):
        return self.R

    def get_alpha(self):
        return self.alpha

    def get_c_s(self):
        return self.c_s

    def get_c_o(self):
        return self.c_o

    