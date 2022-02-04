"""RomiTrajRepr controller."""

"""
Here will be reproduced a prerecorded movement of the robot. 
The movement data is stored in a 3xN matrix, where the first
 column is time, the second is encoder values ert to the left
 wheel, and the third column contains encoder values wrt. the 
 right wheel.

"""

from controller import Robot
import math
import numpy as np 

timestep = 32
max_speed = 20

ObstCounter = 0
reading = [0, 0, 0]

romi = Robot()

wheel_radius = 0.0346
dist_betw_wheel = 0.1415

wheel_circ = 2 * 3.1415 * wheel_radius
enc_un = wheel_circ / 6.283

lin_vel = wheel_radius * max_speed
    
angle_of_rot =  3.1415 / 3
rot_rate = (2 * lin_vel) / dist_betw_wheel
rot_time = np.array([3.1415 / 6 / rot_rate, 3.1415 / 4 / rot_rate, 3.1415 / 3 / rot_rate, 3.1415 / 2 / rot_rate])
rot_time_back = 3.1415 / rot_rate

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
dsNames = ['dsf1', 'dsf2', 'dsf3']
for i in range(3):
    ds.append(romi.getDevice(dsNames[i]))
    ds[i].enable(timestep)

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

def avoid_coll(vl = None, vr = None):
    global reading
    global max_speed
    global rot_time
    if vl != None and vl != None:
        if vl < -max_speed: vl = -max_speed
        if vl > max_speed: vl = max_speed
        if vr < -max_speed: vr = -max_speed
        if vr > max_speed: vr = max_speed

        if np.sum(reading) != 0:
            setSpeed(max_speed, -max_speed)
            delay(rot_time[2] * 1000 - 2 * timestep)
        else:
            setSpeed(vl, vr)
    else:
        if np.sum(reading) != 0:
            setSpeed(max_speed, -max_speed)
            delay(rot_time[2] * 1000 - 2 * timestep)
        else:
            setSpeed(max_speed, max_speed)

def getReading():
    global ObstCounter
        
    if romi.getDevice('dsf1').getValue() < 950: 
        reading[0] = 1
    else:
        reading[0] = 0
    if romi.getDevice('dsf2').getValue() < 950: 
        reading[1] = 1
    else:
        reading[1] = 0
    if romi.getDevice('dsf3').getValue() < 950: 
        reading[2] = 1
    else:
        reading[2] = 0

def get_vels():
    global wheel_radius
    global dist_betw_wheel
    encs = np.loadtxt("trajectory_velocity.txt", unpack = True)
    dt = [encs[0, n]-encs[0, n-1] for n in range(1,len(encs[0]))]
    adt = np.average(dt)
    pose = np.loadtxt("out.txt", unpack = True)
    o = pose[0,:]
    x = pose[1,:]/100
    y = pose[2,:]/100
    n = len(x)
    do = [o[n]-o[n-1] for n in range(1,len(x))]/adt
    dx = [x[n]-x[n-1] for n in range(1,len(x))]/adt
    dy = [y[n]-y[n-1] for n in range(1,len(y))]/adt
    vrs = []
    vls = []
    for i in range(n-1):
        vrs.append(dx[i]/np.cos(o[i])+dist_betw_wheel*0.5*do[i])
        vls.append(dx[i]/np.cos(o[i])-dist_betw_wheel*0.5*do[i])

    return np.array(vrs)/wheel_radius, np.array(vls)/wheel_radius, n
    
def run_robot(robot):
    global timestep
    global n
    global vl
    global vr

    stepNr = 0
    pr = False
    
    # the origin for the robot's pose will be the starting one:
    robot_pose = [0, 0, 0]  #x, y, theta
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
        if pr:
            # print readings and values
            print("--------------------------")
            print("position sensor values: {} {}".format(ps_values[0], ps_values[1]))
            print("f1: {} f2: {} f3: {}".format(ds[0].getValue(), ds[1].getValue(), ds[2].getValue())) 
            print("distance values: {} {}".format(dist_values[0], dist_values[1]))
            print("robot pose: {} {} {}".format(robot_pose[0], robot_pose[1], robot_pose[2]))
        if stepNr < n-1:
            getReading()
            avoid_coll(vl[stepNr], vr[stepNr])
            stepNr += 1
        else:
            print(robot.getTime())
            setSpeed(0,0)

if __name__ == "__main__":

    # create the Robot instance.
    # n = 15
    vl, vr, n = get_vels()
    print(vl[np.argmax(vl)], np.argmax(vl))
    run_robot(romi)
