#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

import frccontrol as fct
import matplotlib.pyplot as plt
import numpy as np


class Climber(fct.System):
    def __init__(self, dt):
        """Climber subsystem.

        Keyword arguments:
        dt -- time between model/controller updates
        """
        state_labels = [("Position", "m"), ("Velocity", "m/s")]
        u_labels = [("Voltage", "V")]
        self.set_plot_labels(state_labels, u_labels)

        fct.System.__init__(
            self,
            np.array([[-12.0]]),
            np.array([[12.0]]),
            dt,
            np.zeros((2, 1)),
            np.zeros((1, 1)),
        )

    def create_model(self, states, inputs):
        # Number of motors
        num_motors = 1.0
        # Robot mass in kg
        m = 63.503
        # Radius of axle in meters
        r = 0.003175
        # Gear ratio
        G = 80 / 1
        return fct.models.elevator(fct.models.MOTOR_775PRO, num_motors, m, r, G)

    def design_controller_observer(self):
        q = [0.02, 0.4]
        r = [12.0]
        self.design_lqr(q, r)
        self.design_two_state_feedforward(q, r)

        q_pos = 0.05
        q_vel = 1.0
        r_pos = 0.0001
        self.design_kalman_filter([q_pos, q_vel], [r_pos])


def main():
    dt = 0.00505
    climber = Climber(dt)
    climber.export_cpp_coeffs("Climber", "control/")

    if "--noninteractive" in sys.argv:
        return

    try:
        import slycot

        climber.plot_pzmaps()
    except ImportError:  # Slycot unavailable. Can't show pzmaps.
        pass

    # Set up graphing
    l0 = 0.1
    l1 = l0 + 5.0
    l2 = l1 + 0.1
    t = np.arange(0, l2 + 10.0, dt)

    refs = []

    # Generate references for simulation
    for i in range(len(t)):
        if t[i] < l0:
            r = np.matrix([[0.0], [0.0]])
        elif t[i] < l1:
            r = np.matrix([[1.524], [0.0]])
        else:
            r = np.matrix([[0.0], [0.0]])
        refs.append(r)

    x_rec, ref_rec, u_rec, y_rec = climber.generate_time_responses(t, refs)
    climber.plot_time_responses(t, x_rec, ref_rec, u_rec)
    plt.show()


if __name__ == "__main__":
    main()
