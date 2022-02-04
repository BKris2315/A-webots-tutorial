"""RomiTrajRepr controller."""

"""
This controller aims to move a robot to a given valid coordinate in the plane.
To do this follows a very simple iteration rule with a variation. The rule is it moves 
forward until the other coordinate and the corresponding part of its position will be
identical, then it turns 90 degrees and goes forwrd to reach the target.

The variation consists from the fact the arena may contain obstacles. To not to crash, the
robot has a safety system, aka obstacle avoidance. If any object is in the way the robot turns
and tries to avoid it.

Note: here the final orientation of the robot is irrelevant, but if it is important for any
reason then the controller can be completed with an extra rotation at the end of the trajectory
to get the desired position of the robot.

The origin of the world frame is always set to be identical with the origin of the robot's
frame in t0.  
"""

from controller import Robot
import math
import numpy as np 

timestep = 32
max_speed = 10

romi = Robot()

wheel_radius = 0.0346
dist_betw_wheel = 0.1415

wheel_circ = 2 * 3.1415 * wheel_radius
enc_un = wheel_circ / 6.283

lin_vel = wheel_radius * max_speed
    
angle_of_rot =  3.1415 / 3
rot_rate = (2 * lin_vel) / dist_betw_wheel
rot_time = np.array([3.1415 / 6 / rot_rate, 3.1415 / 4 / rot_rate, 3.1415 / 3 / rot_rate, 3.1415 / rot_rate])
rot_time_back = 3.1415 / rot_rate

prev_error = 0
kp = 1.25 
ki = 0.01
kd = 0.5
Integral = 0
max_speed = 8

#motor instances
wheels = []
#rmr is right motor
wheelsNames = ['lmr', 'rmr']
for i in range(2):
    wheels.append(romi.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

# front sensors
ds = []
dsNames = ['dsf1', 'dsf2', 'dsf3', 'dsf4', 'dsf5']
for i in range(5):
    ds.append(romi.getDevice(dsNames[i]))
    ds[i].enable(timestep)

reading = np.zeros(len(dsNames))

ps = []
psNames = ['psl', 'psr']
for i in range(2):
    ps.append(romi.getDevice(psNames[i]))
    ps[i].enable(timestep)

def delay(ms):
    initTime = romi.getTime()
    while romi.step(timestep) != -1:
        if (romi.getTime() - initTime) * 1000 > ms:
            break

def setSpeed(leftSpeed, rightSpeed):
    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)

def getReading():
    for i in range(3):
        if ds[i].getValue() < 950: 
            reading[i] = 1
        else:
            reading[i] = 0
    for i in range(3,5):
        if ds[i].getValue() < 450: 
            reading[i] = 1
        else:
            reading[i] = 0

def run_robot(robot):
    global timestep
    global dist_betw_wheel, enc_un
    global x, y
    global kpx, kpy
    global max_speed
    pr = True
    turn1 = 1

    # the origin for the robot's pose will be the starting one:
    robot_pose = np.zeros(3) #x, y, theta
    ps_values = [0, 0]
    dist_values = [0, 0]
    last_ps_values = [0, 0]

    while robot.step(timestep) != -1:
        for i in range(2):
            ps_values[i] = ps[i].getValue()

        for ind in range(2):
            diff = ps_values[ind] - last_ps_values[ind]
            if diff < 1e-3:
                diff = 0
                ps_values[ind] = last_ps_values[ind]
            dist_values[ind] = diff * enc_un
       

        #composing the robot's pose
        v = (dist_values[0] + dist_values[1]) / 2.0
        w = (dist_values[0] - dist_values[1]) / dist_betw_wheel
        dt = 1

        robot_pose[2] += w * dt
        vx = v * math.cos(robot_pose[2])
        vy = v * math.sin(robot_pose[2])

        robot_pose[0] += vx*dt
        robot_pose[1] += vy*dt

        for i in range(2):
            last_ps_values[i] = ps_values[i]

        if pr:
            # print readings and values
            print("--------------------------")
            print("position sensor values: {} {}".format(ps_values[0], ps_values[1]))
            print("f1: {} f2: {} f3: {} f4: {} f5: {}".format(ds[0].getValue(), ds[1].getValue(), ds[2].getValue(), ds[3].getValue(), ds[4].getValue())) 
            print("robot pose: {} {} {}".format(robot_pose[0], robot_pose[1], robot_pose[2]))
        speed = max_speed/2
        if not(x < robot_pose[0] < x + 0.01):
            if x < 0 and turn1 == 1:
                setSpeed(max_speed, -max_speed)
                delay(rot_time[3] * 1000)
                turn1 -= 1

            setSpeed(speed, speed)
        else:
            setSpeed(0,0)
        """  
        error = 0

        errorx = (x - robot_pose[0]) 
        errory = (y - robot_pose[1])

        P = kpx * errorx + kpy * errory
        # I = Integral + (ki * error)
        # if error + errorD > prev_error:
        #     D = kd * (error + errorD - prev_error)
        # else:
        #     D = 0
        if not(x < robot_pose[0] < x + 0.01 and y < robot_pose[1] < y + 0.01):
            corr = P * (abs(y/x) - abs(np.tan(robot_pose[2])))

            
            print(corr)
            getReading()
            if np.sum(reading[0:3]) == 0:
                vl = speed*(1+corr)
                vr = speed*(1-corr)

                if vl < -max_speed: vl = -max_speed
                if vl > max_speed: vl = max_speed
                if vr < -max_speed: vr = -max_speed
                if vr > max_speed: vr = max_speed
                setSpeed(vl, vr)
            else:
                setSpeed(max_speed, -max_speed)
                delay(rot_time[2]/3 * 1000 - 2 * timestep)
        else:
            setSpeed(0,0) """

        

if __name__ == "__main__":

    # create the Robot instance.
    # n = 15
    x = -0.5 #float(input("Please give coordinate X "))
    y = 0.75 #float(input("Please give coordinate Y "))
    kpx = 1
    kpy = 1.1
    print(x, y)
    run_robot(romi)
