from stopping_simulator import StoppingSimulator
from quintic_polynomials_planner import plot_arrow

import matplotlib.pyplot as plt
import numpy as np

show_animation = True


def main():
    print(__file__ + " start!!")

    sx = 10.0  # start x position [m]
    sy = 10.0  # start y position [m]
    sv = 1.0  # start speed [m/s]
    sa = 0.1  # start accel [m/ss]

    gx = 30.0  # goal x position [m]
    gy = -10.0  # goal y position [m]

    yaw = np.arctan2(gy - sy, gx - sx)

    stop_sim = StoppingSimulator()
    time, s, s_d, s_dd, s_ddd = stop_sim.run(
        ego_speed=sv, ego_accel=sa,
        obstacle_distance=np.hypot(gx - sx, gy - sy)
    )

    if show_animation:  # pragma: no cover
        x = [sx + si*np.cos(yaw) for si in s]
        y = [sy + si*np.sin(yaw) for si in s]
        for i, _ in enumerate(time):
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])
            plt.grid(True)
            plt.axis("equal")
            plt.plot(sx, sy, 'ro')
            plt.plot(gx, gy, 'ro')
            plot_arrow(x[i], y[i], yaw)
            plt.title("Time[s]:" + str(time[i])[0:4] +
                      " v[m/s]:" + str(s_d[i])[0:4] +
                      " a[m/ss]:" + str(s_dd[i])[0:4] +
                      " jerk[m/sss]:" + str(s_ddd[i])[0:4],
                      )
            plt.pause(stop_sim.DT)

        plt.plot(x, y, "-r")
        plt.show()


if __name__ == '__main__':
    main()
