"""Wheelie_PID_controller_for_Maze."""

from controller import Robot
import math 

# Some global variables
max_speed = 6.28
timestep = 32 # in ms; timestep for reading values

#define the robot
wheelie = Robot()

# PID parameters: determined by trial and fail :p
prev_error = 0
kp = .25 #megmutatni pl 5.5-re; 0.1 derekszogu
ki = 0.0
kD = 0.1
kd = .75 # 0.25 derekszogu
Integral = 0
error = 0
#Turn parameters
wheel_radius = 0.025
dist_betw_wheel = 0.09

wheel_circ = 2 * 3.1415 * wheel_radius
enc_un = wheel_circ / 6.283

lin_vel = wheel_radius * max_speed
    
angle_of_rot =  3.1415 / 3
rot_rate = (2 * lin_vel) / dist_betw_wheel
rot_time = 1.57 / rot_rate
rot_time_back = 3.1415 / rot_rate

right = 0
left = 0
forw = 0
back = 0

stop = 0

# Sensor readings and ObstAvoid
reading = [0, 0, 0, 0, 0, 0, 0, 0, 0]
ObstCounter = 0

# enable IR for linefollowing
dsd = []
dsdNames = ['dsr2', 'dsr', 'dsm', 'dsl', 'dsl2', 'dsm2', 'dsm3', 'dsr1', 'dsl1']
for i in range(9):
    dsd.append(wheelie.getDevice(dsdNames[i]))
    dsd[i].enable(timestep)

#motor instances
wheels = []

#  motor1 right motor
wheelsNames = ['motor1', 'motor2']
for i in range(2):
    wheels.append(wheelie.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf')) # wbots has position control, after given rotation it stops
    wheels[i].setVelocity(0.0)

"""
This controller will be able to navigate a robot through a maze that consists
of lines. For this we will use a PID controller and sensors. The sensors will
 be IR ones, and they are able to distinguish different colors and

%%%%%%%%%%
Game rules
%%%%%%%%%%

For solving the maze first of all we need some rules and a map that is solvable 
based on the rules. The

The rules are the following:
----------------------------

If you can go Right
If you can't go Right go Straight
If you can't go Right and Straight go left
If you can't go Right, Straight or Left, then go Back

------

On the bottom of the Robot we have five sensors called dsr, dsr2, dsm, dsl, dsl2.
To turn Right: dsr and dsr2 have reading
To turn Left: dsl and dsl2 have reading
To go Straight: dsm has reading and if dsr or dsl have reading too adjusting will commence
To go Back: neither of the sensors have reading

After each turn we will move the robot forward a bit to not to confuse the sensors.
"""
def delay(ms):
    initTime = wheelie.getTime()
    while wheelie.step(timestep) != -1:
        if (wheelie.getTime() - initTime) * 1000 > ms:
            break

def setSpeed(leftSpeed, rightSpeed):
    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)

def avoid_coll():
    global ObstCounter
    
    leftSpeed = 3.0
    rightSpeed = 3.0
    if ObstCounter > 0:
        ObstCounter -= 1
        leftSpeed = 2.0
        rightSpeed = -2.0

    return leftSpeed, rightSpeed

def getReading():
    global ObstCounter
    global right, left, forw, back, stop

    if 180 < wheelie.getDevice('dsl2').getValue() < 210: 
        reading[0] = 1
    else:
        reading[0] = 0
    if 180 < wheelie.getDevice('dsl').getValue() < 210: 
        reading[1] = 1
    else:
        reading[1] = 0
    if 180 < wheelie.getDevice('dsm').getValue() < 210: 
        reading[2] = 1
    else:
        reading[2] = 0
    if 180 < wheelie.getDevice('dsr').getValue() < 210: 
        reading[3] = 1
    else:
        reading[3] = 0
    if 180 < wheelie.getDevice('dsr2').getValue() < 210: 
        reading[4] = 1
    else:
        reading[4] = 0
    if 180 < wheelie.getDevice('dsm2').getValue() < 210: 
        reading[5] = 1
    else:
        reading[5] = 0
    if 180 < wheelie.getDevice('dsm3').getValue() < 210: 
        reading[6] = 1
    else:
        reading[6] = 0
    if 180 < wheelie.getDevice('dsr1').getValue() < 210: 
        reading[7] = 1
    else:
        reading[7] = 0
    if 180 < wheelie.getDevice('dsl1').getValue() < 210: 
        reading[8] = 1
    else:
        reading[8] = 0
    if (75 < wheelie.getDevice('dsm').getValue() < 95) or (75 < wheelie.getDevice('dsm2').getValue() < 95) or (75 < wheelie.getDevice('dsm3').getValue() < 95): 
        stop = 1
    else:
        stop = 0
    
    #turn right
    if (reading[3] == 1 and reading[4] == 1) and stop == 0:
        right = 1

    # go forward
    if right == 0 and (reading[2] == 1 or reading[5] == 1 or reading[6] == 1) and stop == 0: 
        forw = 1
    else:
        forw = 0

    #turn left
    if right == 0 and reading[0] == 1 and reading[1] == 1 and forw == 0 and stop == 0: 
        left = 1

    if reading[0] == 0 and reading[1] == 0 and reading[2] == 0 and reading[3] == 0 and reading[4] == 0 and stop == 0:
        back = 1

    if reading[0] == 0 and reading[1] == 0 and reading[2] == 0 and reading[3] == 0 and reading[4] == 0:
        back = 1

"""
The controller part can be broken down int o different parts: the turning and the straight
movement. The straight movement reqires to stay on track. for that we can make two different approaches.
One is to continously use a low velocity for forward motion and whenever the sensor picks a 
signal a constant velocity to turn. The other method is a more adaptive PID method. Here, 
for turning,  the velocity continously changes in function of an error term.

In the turn part we will give an approximate time to turn either 90 (left or right) or 180 degrees (back)
and after that we will make the necessary adjustments (simply put, no PID control here).
"""

def run_robot(robot):
    global timestep
    global kd, ki, kp, Integral, prev_error
    global right, left, forw, back, stop
    global rot_time, rot_time_back
    
    """ #Enable IR for wall following
    ds = []
    dsNames = ['dsf1', 'dsf2', 'dsf3', 'dsf4','dsf5', 'dsf6', 'dsf7', 'dsf8', 'dsf9']
    for i in range(9):
        ds.append(robot.getDevice(dsNames[i]))
        ds[i].enable(timestep) """

    # enable position sensors

    ps = []
    psNames = ['ps1', 'ps2']
    for i in range(2):
        ps.append(robot.getDevice(psNames[i]))
        ps[i].enable(timestep)

    # the origin for the robot's pose will be the starting one:
    robot_pose = [0, 0, 0]  #x, y, theta
    ps_values = [0, 0]
    dist_values = [0, 0]
    last_ps_values = [0, 0]

    #Main loop: performs the simulation until Webots is stopping the controller

    while robot.step(timestep) != -1:
        
        for i in range(2):
            ps_values[i] = ps[i].getValue()
        
        # printing out readings
        print("--------------------------")
        print("position sensor values: {} {}".format(ps_values[0], ps_values[1]))
        print("left1: {} left: {} middle: {} right: {} right: {}".format(dsd[0].getValue(), dsd[1].getValue(),
                                                                 dsd[2].getValue(), dsd[3].getValue(), dsd[4].getValue()))
        """ print("f1: {} f2: {} f3: {} f2: {}".format(ds[0].getValue(), ds[1].getValue(), ds[2].getValue(), 
                                            ds[3].getValue())) """
        for ind in range(2):
            diff = ps_values[ind] - last_ps_values[ind]
            if diff < 1e-3:
                diff = 0
                ps_values[ind] = last_ps_values[ind]
            dist_values[ind] = diff * enc_un
            
        print("distance values: {} {}".format(dist_values[0], dist_values[1]))

        #composing the robot's pose
        v = (dist_values[0] + dist_values[1]) / 2.0
        w = (dist_values[0] - dist_values[1]) / dist_betw_wheel
        dt = 1
        robot_pose[2] += w * dt
        vx = v * math.cos(robot_pose[2])
        vy = v * math.sin(robot_pose[2])
        robot_pose[0] += vx*dt
        robot_pose[1] += vy*dt

        getReading()
        print(reading)
        speed = 3
        error = 0
        coeff = [1, 2, -1, -2]
        # global error
        error = coeff[0] * reading[1] + coeff[0] * reading[3]
        errorD = coeff[1] * reading[7] + coeff[1] * reading[8]
        P = kp * error + kD * errorD
        I = Integral + (ki * error)
        if error + errorD > prev_error:
            D = kd * (error + errorD - prev_error)
        else:
            D = 0

        corr = (P + I + D)
        print('error', error+errorD, corr, 'D', D)

        #stop
        if stop:
            ls = 0
            rs = 0
            print('stop')
            setSpeed(ls, rs)
        # right decision
        elif right:
            ls = max_speed
            rs = -max_speed
            if ls > max_speed: ls = max_speed
            if ls < -max_speed: ls = -max_speed
            if rs > max_speed: rs = max_speed
            if rs < -max_speed: rs = -max_speed

            setSpeed(ls, rs)
            delay(rot_time * 1000 - 2 *  timestep)
            setSpeed(speed, speed)
            delay(5 * timestep)
            print("right")
            right -= 1
        # forward decision
        elif forw:
            #adjust left
            print("corr", corr)
            if reading[1] != 0:
                ls = speed - abs(corr)
                rs = speed + abs(corr)
                if ls > max_speed: ls = max_speed
                if ls < -max_speed: ls = -max_speed
                if rs > max_speed: rs = max_speed
                if rs < -max_speed: rs = -max_speed

                setSpeed(ls, rs)
                print("al")
            #adjust right
            elif reading[3] != 0:
                ls = speed + abs(corr)
                rs = speed - abs(corr)
                if ls > max_speed: ls = max_speed
                if ls < -max_speed: ls = -max_speed
                if rs > max_speed: rs = max_speed
                if rs < -max_speed: rs = -max_speed

                setSpeed(ls, rs)
                print("ar")
            #go forward
            else:
                l, r = avoid_coll()
                ls = l
                rs = r
                if ls > max_speed: ls = max_speed
                if ls < -max_speed: ls = -max_speed
                if rs > max_speed: rs = max_speed
                if rs < -max_speed: rs = -max_speed
                # error = 0
                setSpeed(ls, rs)
                print("forw")

        elif left:
            ls = -max_speed
            rs = max_speed
            if ls > max_speed: ls = max_speed
            if ls < -max_speed: ls = -max_speed
            if rs > max_speed: rs = max_speed
            if rs < -max_speed: rs = -max_speed

            setSpeed(ls, rs)
            delay(rot_time * 1000 - 2 * timestep)
            setSpeed(speed, speed)
            delay(5 * timestep)
            print("left")
            left -= 1

        elif back:
            ls = max_speed
            rs = -max_speed
            if ls > max_speed: ls = max_speed
            if ls < -max_speed: ls = -max_speed
            if rs > max_speed: rs = max_speed
            if rs < -max_speed: rs = -max_speed

            setSpeed(ls, rs)
            delay(rot_time_back * 1000 - 2 * timestep)
            """ setSpeed(speed, speed)
            delay(5 * timestep) """
            print("back")
            back -= 1

        prev_error = error + errorD
        Integral += I

if __name__ == '__main__':
    
    run_robot(wheelie)
