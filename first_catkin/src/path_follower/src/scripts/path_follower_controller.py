#!/usr/bin/env python

# """
# Path tracking simulation with pure pursuit steering control and PID speed control.
# author: Atsushi Sakai (@Atsushi_twi)
# """
import numpy as np
import math
import matplotlib.pyplot as plt

# ROS imports
import rospy
from path_follower.msg import path_msg
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32


k = 0.1  # look forward gain
Lfc = 2.0  # look-ahead distance
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s]
L = 2.9  # [m] wheel base of vehicle


old_nearest_point_index = None
show_animation = False


path_recieved = False
pos_found = False


px, py = [], []


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((L / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((L / 2) * math.sin(self.yaw))


def update(state, a, delta):

    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * math.tan(delta) * dt
    state.v = state.v + a * dt
    state.rear_x = state.x - ((L / 2) * math.cos(state.yaw))
    state.rear_y = state.y - ((L / 2) * math.sin(state.yaw))

    return state


def PIDControl(target, current):
    a = Kp * (target - current)

    return a


def pure_pursuit_control(state, cx, cy, pind):

    ind = calc_target_index(state, cx, cy)

    if pind >= ind:
        ind = pind

    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw

    Lf = k * state.v + Lfc

    delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)

    return delta, ind


def calc_distance(state, point_x, point_y):

    dx = state.rear_x - point_x
    dy = state.rear_y - point_y
    return math.sqrt(dx ** 2 + dy ** 2)


def calc_target_index(state, cx, cy):

    global old_nearest_point_index

    if old_nearest_point_index is None:
        # search nearest point index
        dx = [state.rear_x - icx for icx in cx]
        dy = [state.rear_y - icy for icy in cy]
        d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
        ind = d.index(min(d))
        old_nearest_point_index = ind
    else:
        ind = old_nearest_point_index
        distance_this_index = calc_distance(state, cx[ind], cy[ind])
        while True:
            ind = ind + 1 if (ind + 1) < len(cx) else ind
            distance_next_index = calc_distance(state, cx[ind], cy[ind])
            if distance_this_index < distance_next_index:
                break
            distance_this_index = distance_next_index
        old_nearest_point_index = ind

    L = 0.0

    Lf = k * state.v + Lfc

    # search look ahead target point index
    while Lf > L and (ind + 1) < len(cx):
        dx = cx[ind] - state.rear_x
        dy = cy[ind] - state.rear_y
        L = math.sqrt(dx ** 2 + dy ** 2)
        ind += 1

    return ind


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    """
    Plot arrow
    """

    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)


def pathCallback(points):
    global path_recieved, px, py
    path_recieved = True
    # rospy.loginfo(str(points.points))
    for p in points.points:
        px.append(p.x)
        py.append(p.y)


def gpsCallback(data):
    pos_found = True



def main():
    #  target course
    # cx = np.arange(0, 50, 0.1)
    # cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]

    rospy.init_node('path_follower_controller_py', anonymous=False)

    rospy.Subscriber('path_msg', path_msg, pathCallback)
    rospy.Subscriber('veh_gps', Float32MultiArray, gpsCallback)

    steer_pub = rospy.Publisher('steer_control', Float32, queue_size=1000)
    throttle_pub = rospy.Publisher(
        'throttle_control', Float32, queue_size=1000)
    braking_pub = rospy.Publisher('braking_control', Float32, queue_size=1000)

    target_speed = 10.0 / 3.6  # [m/s]

    T = 100.0  # max simulation time

    count = 1
    while not path_recieved:
        if rospy.is_shutdown():
            return

    # initial state
    state = State(x=px[0], y=py[0], yaw=0.0, v=0.0)

    lastIndex = len(px) - 1
    time = 0.0
    # x = [state.x]
    # y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_ind = calc_target_index(state, px, py)

    while not rospy.is_shutdown() and T >= time and lastIndex > target_ind:
        if not pos_found and not path_recieved:
            continue

        ai = PIDControl(target_speed, state.v)
        di, target_ind = pure_pursuit_control(state, px, py, target_ind)
        # state = update(state, ai, di)

        steer_pub.publish(di)

        dv = x[-1] - state.v
        throttle_pub.publish(dv if dv > 0 else 0.0)
        braking_pub.publish(dv if dv < 0 else 0.0)

        print(dv)

        time = time + dt

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)

        if show_animation:  # pragma: no cover
            plt.cla()
            plot_arrow(state.x, state.y, state.yaw)
            plt.plot(px, py, "-r", label="course")
            plt.plot(x, y, "-b", label="trajectory")
            plt.plot(px[target_ind], py[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.pause(0.001)

    # Test
    assert lastIndex >= target_ind, "Cannot goal"

    if show_animation:  # pragma: no cover
        plt.cla()
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(x, y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)

        plt.subplots(1)
        plt.plot(t, [iv * 3.6 for iv in v], "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.grid(True)
        plt.show()


if __name__ == '__main__':
    # print("Pure pursuit path tracking simulation start")
    try:
        main()
    except rospy.ROSInterruptException:
        pass
