from quintic_polynomials_planner import QuinticPolynomial

import numpy as np


class StoppingSimulator:
    def __init__(self):
        self.SAFETY_DIST = 1.0  # safety distance [m]
        self.D_S_S = 0.5  # distance sampling length [m]
        self.N_S_SAMPLE = 2  # sampling number of distance
        self.DT = 0.1  # time sampling length [s]
        self.MAX_T = 3.0  # max prediction time [s]
        self.MIN_T = 0.5  # min prediction time [s]

    def run(self, ego_speed, ego_accel, obstacle_distance):
        target_s = obstacle_distance - self.SAFETY_DIST

        t = None
        s = None
        s_d = None
        s_dd = None
        s_ddd = None
        best_qp = None

        min_ddd = float('inf')  # min jerk
        for Ti in np.arange(
            self.MIN_T,
            self.MAX_T + self.DT * 0.1,
            self.DT
        ):
            for Si in np.arange(
                target_s,
                max(target_s - self.D_S_S*(self.N_S_SAMPLE + 0.1), 0.1),
                -self.D_S_S
            ):
                lon_qp = QuinticPolynomial(0., ego_speed, ego_accel, Si, 0.0, 0.0, Ti)

                ddd = lon_qp.calc_third_derivative(0.)  # we only need to check the instant jerk, i.e. the moment we start braking
                if min_ddd > ddd:
                    min_ddd = ddd  # found the better longitudinal path
                    best_qp = lon_qp
                    t = [ti for ti in np.arange(0.0, Ti, self.DT)]  # list of time points
                    s = [lon_qp.calc_point(ti) for ti in t]  # list of positions, i.e. distances
                    s_d = [lon_qp.calc_first_derivative(ti) for ti in t]  # list of speeds
                    s_dd = [lon_qp.calc_second_derivative(ti) for ti in t]  # list of accelerations
                    s_ddd = [lon_qp.calc_third_derivative(ti) for ti in t]  # list of jerks

        return t, s, s_d, s_dd, s_ddd, best_qp
