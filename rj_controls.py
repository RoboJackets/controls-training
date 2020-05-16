import numpy as np
import matplotlib.pyplot as plt

def simulate_system(
        x0,
        f,
        dt,
        T,
        discrete=False,
        output_fn=None,
        control_fn=None,
        hide_states=[],
        hide_inputs=[],
        hide_outputs=[],
        hide_goals=[],
        plot_phase=None,
        goal=None):
    """
    Simulate a system and plot the results.
    """

    x0 = np.array(x0, dtype=np.float32).reshape(-1)

    # Parameter preprocessing
    if plot_phase == True:
        plot_phase = (0, 1)

    if type(goal) == float:
        goal = np.array(goal, dtype=np.float32).reshape(-1)

    if type(goal) == type(x0):
        goal_value = goal
        goal = lambda _: goal_value

    if type(control_fn) == float:
        control_fn = np.array(control_fn, dtype=np.float32).reshape(-1)

    if type(control_fn) == type(x0):
        control_value = control_fn
        if goal:
            control_fn = lambda _1, _2, _3: control_value
        else:
            control_fn = lambda _1, _2: control_value

    if type(output_fn) == type(x0):
        output_value = output_fn
        output_fn = lambda _1, _2: output_value

    if plot_phase == True:
        plot_phase = (0, 1)

    # Calculate dimensions
    state_dim = x0.shape[0]
    g0 = (goal(0),) if goal else ()
    control_dim = 0
    output_dim = 0
    goal_dim = 0

    try:
        control_dim = np.array(control_fn(x0, 0, *g0)).reshape(-1).shape[0]
    except:
        pass

    try:
        goal_dim = np.array(goal(0)).reshape(-1).shape[0]
    except:
        pass

    try:
        output_dim = np.array(output_fn(x0, 0)).reshape(-1).shape[0]
    except:
        pass

    # Simulate the system
    x = x0
    xs = []
    us = []
    ys = []
    gs = []

    ts = np.linspace(0, T * dt, T + 1)

    # We h
    xs.append(x0)

    # Skip simulating the last timestep.
    for t in ts[:-1]:
        # First, we calculate and record outputs
        # (based off of the previous state)
        y = np.array([0.0])
        if output_fn:
            y = np.array(output_fn(x, t)).reshape(-1)
        ys.append(y)

        g = (np.array(goal(t)).reshape(-1),) if goal else ()
        gs.append(g)

        # Then, do the same for inputs
        u = np.array([0.0])
        if control_fn:
            u = np.array(control_fn(x, t, *g)).reshape(-1)
        us.append(u)

        # Finally, save the new state
        if discrete:
            x = f(x, u, t)
        else:
            x = x + dt * f(x, u, t)
        xs.append(x)

    xs = np.array(xs).reshape(T + 1, -1)
    us = np.array(us).reshape(T, -1) if control_dim != 0 else None
    ys = np.array(ys).reshape(T, -1) if output_dim != 0 else None
    gs = np.array(gs).reshape(T, -1) if goal_dim != 0 else None

    fig = None
    time_ax = None
    phase_ax = None
    if plot_phase:
        fig, axs = plt.subplots(1, 2, figsize=(14, 7))
        time_ax = axs[0]
        phase_ax = axs[1]
    else:
        fig, axs = plt.subplots(1, 1, figsize=(7, 7))
        time_ax = axs

    for i in range(state_dim):
        if i not in hide_states:
            time_ax.plot(ts, xs[:, i], label=f'x[{i}]')

    if gs is not None:
        for i in range(goal_dim):
            if i not in hide_states:
                time_ax.plot(ts[:-1], gs[:, i], label=f'g[{i}]')

    for i in range(control_dim):
        if i not in hide_inputs:
            time_ax.plot(ts[:-1], us[:, i], label=f'u[{i}]')

    for i in range(output_dim):
        if i not in hide_outputs:
            time_ax.plot(ts[1:], ys[:, i], label=f'y[{i}]')

    time_ax.legend()

    if plot_phase:
        x0i, x1i = plot_phase

        x0s = xs[:, x0i]
        x1s = xs[:, x1i]

        min_x0, max_x0 = np.min(x0s), np.max(x0s)
        min_x1, max_x1 = np.min(x1s), np.max(x1s)
        dx0 = (max_x0 - min_x0) * 0.1
        dx1 = (max_x1 - min_x1) * 0.1

        X0, X1 = np.meshgrid(np.linspace(min_x0 - dx0, max_x0 + dx0, 10),
                             np.linspace(min_x1 - dx1, max_x1 + dx1, 10))

        dX = np.zeros((2, X0.shape[0], X0.shape[1]))
        for i in range(X0.shape[0]):
            for j in range(X0.shape[1]):
                X = np.array([X0[i, j], X1[i, j]])
                g = (goal(0),) if goal else ()
                u = control_fn(X, 0, *g) if control_fn else 0.0
                u = np.array(u).reshape(-1)
                dX[:, i, j] = f(X, u if control_fn else np.array([0.]), 0)

        phase_ax.quiver(X0, X1, dX[x0i], dX[x1i])
        phase_ax.plot(x0s, x1s)
        phase_ax.set_aspect('equal', 'box')
        phase_ax.set_xlabel(f'x[{x0i}]')
        phase_ax.set_ylabel(f'x[{x1i}]')

    plt.show()

class PID:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt

        self.last_e = 0
        self.integral = 0

    def __call__(self, x, t, g):
        error = g - x[0]
        self.integral += error * self.dt
        derivative = (error - self.last_e) / self.dt
        self.last_e = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

def limit_control(control_fn, u_min, u_max):
    def limited_fn(x, t, g=()):
        return np.minimum(u_max, np.maximum(u_min, control_fn(x, t, *g)))
    return limited_fn

def mass_spring_damper(m, k, c):
    def dynamics(x, u, t):
        return np.array([x[1], (u[0] - k * x[0] - c * x[1]) / m])
    return dynamics

if __name__ == '__main__':
    # Run a test
    x0 = np.array([0, 1])
    dynamics = lambda x, u, t: np.array([x[1], u[0] - x[0] - 0.3 * x[1]])
    dt = 0.1
    T = 350
    simulate_system(x0, dynamics, dt, T, discrete=False,
                    # control_fn=lambda x, t: np.array([np.sin(t)]),
                    plot_phase=(0, 1))
