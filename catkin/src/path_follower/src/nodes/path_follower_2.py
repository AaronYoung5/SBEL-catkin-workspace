#!/usr/bin/env python

# Python imports
import math

# ROS imports
import rospy
from path_follower.msg import path_msg
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import String

path_recieved, pos_found = False, False;

path = Path()

class Path:
    x,y = [],[]
    len = 0
    def __init(self):
        pass

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((L / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((L / 2) * math.sin(self.yaw))

def PIDControl(target, current, Kp=1.0):
    a = Kp * (target - current)

    return a


def pure_pursuit_control(state, path, pind):

    ind = calc_target_index(state, path)

    if ind >= last_index:
        ind = last_index

    if ind < path.len:
        tx = path.x[ind]
        ty = path.y[ind]
    else:
        tx = path.x[-1]
        ty = path.y[-1]
        ind = path.len - 1

    alpha = math.atan2(ty - state.x, tx - state.y) - state.yaw

    Lf = k * state.v + Lfc

    delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)

    return delta, ind


def calc_target_index(state, path):
    global last_index

    if last_index is None:
        dx = [state.x - ipx for ipx in path.x]
        dy = [state.y - ipy for ipy in path.y]
        d = [abs(math.sqrt(idx**2 + idy**2) for (idx, idy) in zip(dx, dy)]
        ind=d.index(min(d))
        last_index=ind
    else:
        ind=last_index
        distance_this_index=calc_distance(path.x[ind], path.y[ind])
        while True:
            ind=ind + 1 if (ind + 1) < path.len else ind
            distance_next_index=calc_distance(path.x[ind], path.y[ind])
            if distance_this_index < distance_next_index:
                break
            distance_this_index=distance_next_index
        last_index=ind

def calc_distance(point_x, point_y):
    global x, y
    dx=x - point_x
    dx=y - point_y
    return math.sqrt(dx**2 + dy**2)

def pathCallback(points):
    global path_recieved
    path_recieved=True
    for p in points.points:
        path.x.append(p.x)
        path.y.append(p.y)


def gpsCallback(data):
    global pos_found, x, y
    pos_found=True;
    x=data.data[0]
    y=data.data[1]
    z=data.data[2]


def main():

    rospy.init_node('path_follower', anonymous=True)

    rospy.Subscriber('path_msg', path_msg, pathCallback)
    rospy.Subscriber('veh_gps', Float32MultiArray, gpsCallback)

    qs=1000
    throttle_pub=rospy.Publisher(
        'throttle_control', Float32, queue_size=qs)
    steering_pub=rospy.Publisher(
        'steering_control', Float32, queue_size=qs)
    braking_pub=rospy.Publisher(
        'braking_control', Float32, queue_size=qs)

    while not rospy.is_shutdown():
        if not path_recieved or not pos_found:
            continue



        throttle_pub.publish(.5)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
