"""Wheelie_driver_with_encoder controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor


#Here will be used Odometry calculations
from controller import Robot
import math

# PID parameters
prev_error = 0
kp = 5.5
ki = 0.0
kd = 0.5
Integral = 0
timestep = 1
max_speed = 8

reading = [0, 0, 0, 0, 0]
ObstCounter = 0

wheel_radius = 0.025
dist_betw_wheel = 0.09

wheel_circ = 2 * 3.1415 * wheel_radius
enc_un = wheel_circ / 6.283

lin_vel = wheel_radius * max_speed
    
angle_of_rot =  3.1415 / 3
rot_rate = (2 * lin_vel) / dist_betw_wheel
rot_time = angle_of_rot / rot_rate
rot_time_back = 3.1415 / rot_rate

def avoid_coll():
    global Timestep
    global wheelie
    global ObstCounter
    
    leftSpeed = 5
    rightSpeed = 5
    if ObstCounter > 0:
        ObstCounter -= 1
        leftSpeed = 2.0
        rightSpeed = -2.0

    return leftSpeed, rightSpeed
    


def getReading():
    global ObstCounter

    if wheelie.getDevice('dsl').getValue() == 1000: 
        reading[0] = 1
    else:
        reading[0] = 0
    if wheelie.getDevice('dsr').getValue() == 1000: 
        reading[1] = 1
    else:
        reading[1] = 0
    if 650 < wheelie.getDevice('dsm').getValue() < 700: 
        reading[2] = 58
    if 500 < wheelie.getDevice('dsm').getValue() < 575 and reading[0] != 0:
        reading[3] = 29
    if 500 < wheelie.getDevice('dsm').getValue() < 575 and reading[1] != 0:
        reading[4] = 29
        
    if wheelie.getDevice('dsf1').getValue() < 950: ObstCounter = 50
    if wheelie.getDevice('dsf2').getValue() < 950: ObstCounter = 50
    if wheelie.getDevice('dsf3').getValue() < 950: ObstCounter = 50
    if wheelie.getDevice('dsf4').getValue() < 950: ObstCounter = 50

def turn_it(r, l, b ):
    if r:
        ls = 3.0
        rs = -3.0
    if l:
        ls = -3.0
        rs = 3.0
    if b:
        print("back")
        reading[2] -= 1
        ls = 3.0
        rs = -3.0
   
    return ls, rs

def PID():
    speed = 5.5
    global max_speed
    error = 0
    coeff = [1, 1]
    for i in range(2):
        error += coeff[i] * reading[i]

    P = kp * error
    I = Integral + (ki * error)
    D = kd * (error - prev_error)

    corr = (P + I + D)
    if reading[2] != 0:
        l, r = turn_it(0, 0, 1)
        ls = l
        rs = r 
        print(ls,rs)

    """ if reading[4] != 0 and reading[0] == 0 and reading[1] == 0:
        l, r = turn_it(1, 0, 0)
        ls = l
        rs = r 
        print(ls,rs)

    if reading[2] != 0:
        l, r = turn_it(0, 1, 0)
        ls = l
        rs = r 
        print(ls,rs) """

    # if reading[0] != 0 and reading[2] == 0  and reading[3] == 0  and reading[4] == 0:
    if reading[0] != 0 and reading[2] == 0:
        ls = speed + corr
        rs = speed - corr
        print("r0")
        
    # if reading[1] != 0 and reading[2] == 0  and reading[3] == 0  and reading[4] == 0:
    if reading[1] != 0 and reading[2] == 0:
        ls = speed - corr
        rs = speed + corr
        print("r1")
            
    if (reading[0] == 0) and (reading[1] == 0) and (reading[2] == 0):
    # if (reading[0] == 0) and (reading[1] == 0) and (reading[2] == 0):
        ls, rs = avoid_coll()
        print("av")
        # ls = rs = speed


    if ls > max_speed: ls = max_speed
    if ls < -max_speed: ls = -max_speed
    if rs > max_speed: rs = max_speed
    if rs < -max_speed: rs = -max_speed
    
    print(ls, rs)
    return ls, rs, I, error

def run_robot(robot):
    #timestep of the current world
    global timestep 
    ps_values = [0, 0]
    dist_values = [0, 0]

    global enc_un 
    
    ds = []
    dsNames = ['dsf1', 'dsf2', 'dsf3', 'dsf4']
    for i in range(4):
        ds.append(wheelie.getDevice(dsNames[i]))
        ds[i].enable(timestep)
    #motor instances
    right_motor = robot.getDevice('motor1')
    left_motor = robot.getDevice('motor2')

    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)   
    
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)
    
    # instances for encoders
    left_ps = robot.getDevice('ps1')
    left_ps.enable(timestep)
    
    right_ps = robot.getDevice('ps2')
    right_ps.enable(timestep)
    
    wheel_circ = 2 * 3.1415 * wheel_radius
    enc_un = wheel_circ / 6.283
    
    #Robot pose
    
    robot_pose = [0, 0, 0]  #x, y, theta
    last_ps_values = [0, 0]
    
    #Enable IR
    
    left_ir = robot.getDevice('dsl')
    left_ir.enable(timestep)
    
    mid_ir = robot.getDevice('dsm')
    mid_ir.enable(timestep)
    
    right_ir = robot.getDevice('dsr')
    right_ir.enable(timestep)
    
    #Main loop
    #Performs the simulation until Webots is stopping the controller

    while robot.step(timestep) != -1:
    
        #reading values of position sensors, these are in radians
        
        """
        With encoders we get encoders ticks. These being summed we will find how much our wheel 
        has been rotated and from that we will be able to tell the distance it travelled.
        
        In Webots the encoder and encoder computation has already been done
        with the Position Sensor.
        """
        
        ps_values[0] = left_ps.getValue()
        ps_values[1] = right_ps.getValue()
        
        #Reading IR values
        
        l_ir_val = left_ir.getValue()
        m_ir_val = mid_ir.getValue()
        r_ir_val = right_ir.getValue() 
        
        #if (l_ir_val > r_ir_val
               
        print("--------------------------")
        print("position sensor values: {} {}".format(ps_values[0], ps_values[1]))
        print("left: {} middle: {} right: {}".format(l_ir_val, m_ir_val, r_ir_val))
        print("f1: {} f2: {} f3: {} f2: {}".format(ds[0].getValue(), ds[1].getValue(), ds[2].getValue(), 
                                            ds[3].getValue()))
        for ind in range(2):
            diff = ps_values[ind] - last_ps_values[ind]
            if diff < 1e-3:
                diff = 0
                ps_values[ind] = last_ps_values[ind]
            dist_values[ind] = diff * enc_un
            
        print("distance values: {} {}".format(dist_values[0], dist_values[1]))
        
        #linear velocity
        v = (dist_values[0] + dist_values[1]) / 2.0
        w = (dist_values[0] - dist_values[1]) / dist_betw_wheel
        
        dt = 1
        
        robot_pose[2] += w * dt
        
        vx = v * math.cos(robot_pose[2])
        vy = v * math.sin(robot_pose[2])
        
        robot_pose[0] += vx*dt
        robot_pose[1] += vy*dt
        
        print("robot pose: {} {} {}".format(robot_pose[0], robot_pose[1], robot_pose[2]))

        #with rotation we can show that the encoder values difference will grow significantly
        getReading()
        ls, rs, Integral, prev_error = PID()
        left_motor.setVelocity(ls)
        right_motor.setVelocity(rs)

        for i in range(2):
            last_ps_values[i] = ps_values[i]
            
        
        
if __name__ == "__main__":

    # create the Robot instance.
    wheelie = Robot()
    run_robot(wheelie)