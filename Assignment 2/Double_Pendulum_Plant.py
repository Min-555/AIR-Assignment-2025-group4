"""
Double Pendulum Dynamics Simulation
"""

import numpy as np
import matplotlib.pyplot as plt


class DoublePendulum:
    def __init__(self, m1, m2, r1, r2, l1, l2, I1, I2, damping=0.0,
                 gravity=9.81, torque_limit=np.inf):
        self.m1 = m1
        self.m2 = m2
        self.r1 = r1
        self.r2 = r2
        self.l1 = l1
        self.l2 = l2
        self.I1 = I1
        self.I2 = I2
        self.damping = damping
        self.gravity = gravity
        self.torque_limit = torque_limit

        self.dof = 2  # Degrees of freedom
        self.x = np.zeros((2*self.dof, 1))  # State vector [q1, q2, dq1, dq2]
        self.t = 0.0

        self.t_values = []
        self.x_values = []
        self.tau_values = []

    def set_state(self, x, time):
        """
        Set the state of the double pendulum.
        :param x: State vector [q1, q2, dq1, dq2]
        :param t: Time
        """
        self.x = x
        self.t = time

    def get_state(self):
        """
        Get the state of the double pendulum.
        :return: State vector [theta1, theta2, omega1, omega2]
        :return: Time
        """
        return self.x, self.t

    def forward_kinematics(self, pos):
        """
        Compute the forward kinematics of the double pendulum.
        :param pos: State vector [theta1, theta2]
        :return: End effector positions [x1, y1, x2, y2]
        """
        q1, q2 = pos
        x1 = self.l1 * np.sin(q1)
        y1 = -self.l1 * np.cos(q1)
        x2 = x1 + self.l2 * np.sin(q1+q2)
        y2 = y1 - self.l2 * np.cos(q1+q2)
        return np.array([x1, y1, x2, y2])

    def forward_dynamics(self, th, th_dot, tau):
        """
        Compute the forward dynamics of the double pendulum.
        :param th: State vector [theta1, theta2]
        :param th_dot: State vector [omega1, omega2]
        :param tau: Torque vector [tau1, tau2]
        :return: Acceleration vector [alpha1, alpha2]
        """
        q1, q2 = th
        dq1, dq2 = th_dot
        m1, m2 = self.m1, self.m2
        r1, r2 = self.r1, self.r2
        l1 = self.l1
        I1, I2 = self.I1, self.I2
        g = self.gravity

        # Compute the equations of motion
        M = np.array([[I1 + I2 + m2*l1**2 + 2*l1*m2*r2*np.cos(q2) + m1*r1**2 + m2*r2**2,
                       I2 + m2*l1*r2*np.cos(q2) + m2*r2**2],
                      [I2 + m2*l1*r2*np.cos(q2) + m2*r2**2,
                       m2*r2**2 + I2]])

        C = np.array([[0, -l1*m2*r2*np.sin(q2)*(2*dq1 + dq2)],
                     [l1*m2*r2*np.sin(q2)*dq1, 0]])

        G = np.array([g*(m1*r1 + m2*l1)*np.sin(q1) + g*m2*r2*np.sin(q1+q2),
                     g*m2*r2*np.sin(q1+q2)])

        # Compute the accelerations
        ddq = np.linalg.solve(M, tau - np.dot(C, th_dot) - G)

        return ddq

    def compute_derivatives(self, t, x, tau):
        """
        Compute the derivatives of the state vector.
        :param x: State vector [theta1, theta2, omega1, omega2]
        :param t: Time
        :param tau: Torque vector [tau1, tau2]
        :return: Derivative of the state vector
        """
        th = x[:self.dof]
        th_dot = x[self.dof:]
        ddq = self.forward_dynamics(th, th_dot, tau)
        dxdt = np.concatenate((th_dot, ddq))
        return dxdt

    def runge_integrator(self, t, x, dt, tau):
        """
        Runge-Kutta integrator for the double pendulum.
        :param t: Current time
        :param x: Current state vector [theta1, theta2, omega1, omega2]
        :param dt: Time step
        :param tau: Torque vector [tau1, tau2]
        :return: Updated state vector
        """
        k1 = self.compute_derivatives(t, x, tau)
        k2 = self.compute_derivatives(t + 0.5*dt, x + 0.5*dt*k1, tau)
        k3 = self.compute_derivatives(t + 0.5*dt, x + 0.5*dt*k2, tau)
        k4 = self.compute_derivatives(t + dt, x + dt*k3, tau)
        integ = (k1 + 2*(k2 + k3) + k4) / 6.0

        y_new = x + dt*integ
        # y_new[:self.dof] = np.mod(y_new[:self.dof], 2*np.pi)
        # print(f"y_new: {y_new}")
        return y_new

    def step(self, tau, dt, integrator="runge_kutta"):
        tau = np.clip(tau, -self.torque_limit, self.torque_limit)
        if integrator == "runge_kutta":
            self.x = self.runge_integrator(self.t, self.x, dt, tau)
        elif integrator == "euler":
            self.x = self.euler_integrator(self.t, self.x, dt, tau)
        self.t += dt
        # Store the time series output
        self.t_values.append(self.t)
        self.x_values.append(self.x.copy())
        self.tau_values.append(tau)

    def simulate(self, t0, x0, tf, dt, controller=None,
                 integrator="runge_kutta"):
        self.set_state(x0, t0)

        self.t_values = []
        self.x_values = []
        self.tau_values = []

        while (self.t <= tf):
            if controller is not None:
                tau = controller.get_control_output(self.x, self.t)
            else:
                tau = 0
            self.step(tau, dt, integrator=integrator)

        return self.t_values, self.x_values, self.tau_values


def plot_timeseries(T, X, U, torque_show=False):
    plt.plot(T, np.asarray(X).T[0], label="theta 1", alpha=0.8)
    plt.plot(T, np.asarray(X).T[2], label="theta dot 1", linestyle="--", alpha=0.6)
    plt.plot(T, np.asarray(X).T[1], label="theta 2", alpha=0.8)
    plt.plot(T, np.asarray(X).T[3], label="theta dot 2", linestyle="--", alpha=0.6)
    if torque_show:
        plt.plot(T, np.asarray(U).T[0], label="tau 1", linestyle="-.", alpha=0.6)
        plt.plot(T, np.asarray(U).T[1], label="tau 2", linestyle="-.", alpha=0.6)
    plt.title("Double Pendulum State")
    plt.xlabel("Time (s)")
    plt.ylabel("State")
    plt.legend(loc="best")
    plt.show()


class GravityCompController():
    def __init__(self, m1, m2, l1, l2, r1, r2, gravity):
        self.g = gravity
        self.m1 = m1
        self.m2 = m2
        self.l1 = l1
        self.l2 = l2
        self.r1 = r1
        self.r2 = r2
        self.dof = 2

    def get_control_output(self, x, t):
        # compensate gravity with input torque
        m1 = self.m1
        m2 = self.m2
        l1 = self.l1
        # l2 = self.l2
        r1 = self.r1
        r2 = self.r2
        g = self.g

        # Angle theta is available to you as a part of the state vector
        q1 = x[0]
        q2 = x[1]

        G = np.array([g*(m1*r1 + m2*l1)*np.sin(q1) + g*m2*r2*np.sin(q1+q2),
                     g*m2*r2*np.sin(q1+q2)])

        tau = G
        ##
        return tau


class PFLEnergyShapingController():
    def __init__(self, m1, m2, l1, l2, r1, r2, I1, I2, gravity,
                 k_pfl=1.0, k_es=1.0, Kp=20, Kd=4, torque_limit=np.inf):
        self.g = gravity
        self.m1 = m1
        self.m2 = m2
        self.l1 = l1
        self.l2 = l2
        self.r1 = r1
        self.r2 = r2
        self.I1 = I1
        self.I2 = I2
        self.dof = 2
        self.k_pfl = k_pfl
        self.k_es = k_es
        self.Kp = Kp
        self.Kd = Kd
        self.torque_limit = torque_limit

        self.energy_log = []
        self.torque_log = []
        self.time_log = []

    def set_goal(self, goal):
        self.goal = goal

    def PFL(self, th, th_dot, qddot_desired):
        """
        PFL controller
        :param th: Current angles of the pendulum
        :param th_dot: Current angular velocities of the pendulum
        :param qddot_desired: Desired angular acceleration
        :return: Control torque
        """
        q1, q2 = th
        dq1, dq2 = th_dot
        m1, m2 = self.m1, self.m2
        r1, r2 = self.r1, self.r2
        l1 = self.l1
        I1, I2 = self.I1, self.I2
        g = self.g

        # Compute the equations of motion
        M = np.array([[I1 + I2 + m2*l1**2 + 2*l1*m2*r2*np.cos(q2) + m1*r1**2 + m2*r2**2,
                       I2 + m2*l1*r2*np.cos(q2) + m2*r2**2],
                      [I2 + m2*l1*r2*np.cos(q2) + m2*r2**2,
                       m2*r2**2 + I2]])

        C = np.array([[0, -l1*m2*r2*np.sin(q2)*(2*dq1 + dq2)],
                     [l1*m2*r2*np.sin(q2)*dq1, 0]])

        G = np.array([g*(m1*r1 + m2*l1)*np.sin(q1) + g*m2*r2*np.sin(q1+q2),
                     g*m2*r2*np.sin(q1+q2)])

        # Compute the control torque
        dq_desired_2 = - (M[0, 0] * qddot_desired + C[0, 1] * dq2 + G[0]) / M[0, 1]
        tau = M[1, 0] * qddot_desired + M[1, 1] * dq_desired_2 + C[1, 0] * dq1 + G[1]
        tau = self.k_pfl * tau
        return tau

    def get_control_output(self, x, t):
        """
        Compute the control output using the PFL controller.
        :param x: State vector [theta1, theta2, omega1, omega2]
        :return: Control torque
        """
        q1, q2, dq1, dq2 = x

        TE_d = self.m1 * self.g * self.r1 + self.m2 * self.g * (self.l1 + self.r2)
        PE = self.m1 * self.g * (-self.r1 * np.cos(q1)) + \
            self.m2 * self.g * (-self.l1 * np.cos(q1) - self.r2 * np.cos(q1 + q2))

        KE1 = 1/2 * self.m1 * self.r1**2 * dq1**2 + 1/2 * self.I1 * dq1**2
        v2x = self.l1 * np.cos(q1) * dq1 + self.r2 * np.cos(q1 + q2) * (dq1 + dq2)
        v2y = self.l1 * np.sin(q1) * dq1 + self.r2 * np.sin(q1 + q2) * (dq1 + dq2)
        v2_sq = v2x**2 + v2y**2

        KE2 = 0.5 * self.m2 * v2_sq + 0.5 * self.I2 * (dq1 + dq2)**2
        KE = KE1 + KE2
        TE_c = PE + KE

        # PD Controller on passive joint (joint 1)
        q1 = q1 % (2 * np.pi)
        q1_desired = np.pi
        dq1_desired = 0
        Kp = self.Kp
        Kd = self.Kd
        error = q1_desired - q1
        d_error = dq1_desired - dq1
        qddot_desired = Kp * error + Kd * d_error

        if t < 2:
            alpha = 0.0
        elif 2 <= t < 2 + 1.0:
            alpha = (t - 2) / 1.0
        else:
            alpha = 1.0

        tau_es = -self.k_es * alpha * dq1 * (TE_d - TE_c)

        # tau_es = -self.k_es * dq1 * (TE_d - TE_c)
        tau_pfl = self.PFL(x[:self.dof], x[self.dof:], qddot_desired)
        print(f"tau_pfl: {tau_pfl}\n")
        print(f"tau_es: {tau_es}\n")
        tau_2 = tau_pfl + tau_es
        # Clip the torque to the specified limit
        tau_2 = np.clip(tau_2, -self.torque_limit, self.torque_limit)

        tau = np.array([0, tau_2])

        self.energy_log.append((t, TE_c, TE_d))
        self.torque_log.append((t, tau_2))
        self.time_log.append(t)
        return tau

    def plot_energy(self):

        times, TE_c, TE_d = zip(*self.energy_log)
        _, torques = zip(*self.torque_log)

        plt.figure(figsize=(10, 4))
        plt.plot(times, TE_c, label="Total Energy (TE_c)")
        plt.plot(times, TE_d, label="Desired Energy (TE_d)", linestyle="--")
        plt.xlabel("Time [s]")
        plt.ylabel("Energy [J]")
        plt.title("Energy Shaping")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.show()

        # Plot torque
        plt.figure(figsize=(10, 4))
        plt.plot(times, torques, label="Control Torque τ₂")
        plt.xlabel("Time [s]")
        plt.ylabel("Torque [Nm]")
        plt.title("Control Torque Over Time")
        plt.grid(True)
        plt.tight_layout()
        plt.show()

