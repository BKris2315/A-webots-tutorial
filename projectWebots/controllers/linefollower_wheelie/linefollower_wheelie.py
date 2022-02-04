"""Wheelie_driver_with_encoder controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor


#Here will be used Odometry calculations
from controller import Robot
import math

# PID parameters
prev_error = 0
kp = 2.5
ki = 0
kd = 0.5
Integral = 0

last_error = differ = intg = prop =0
reading = [0, 0]

def getReading():
    if wheelie.getDevice('dsl').getValue() == 1000: 
        reading[0] = 1
    else:
        reading[0] = 0
    if wheelie.getDevice('dsr').getValue() == 1000: 
        reading[1] = 1
    else:
        reading[1] = 0

def PID():
    error = 0
    coeff = [-1000, 1000]
    for i in range(2):
        error += coeff[i] * reading[i]

    P = kp * error
    I = Integral + (ki * error)
    D = kd * (error - prev_error)

    corr = (P + I + D) / 1000

    ls = 6.28 + corr
    rs = 6.28 + corr

    if ls > 6.28: ls = 6.28
    if ls < 0: ls = 0
    if rs > 6.28: rs = 6.28
    if rs < 0: rs = 0

def run_robot(robot):
    #timestep of the current world
    timestep = 10
    ps_values = [0, 0]
    dist_values = [0, 0]
    max_speed = 6.28
    wheel_radius = 0.025
    dist_betw_wheel = 0.09
    avoidObstacleCounter = 0
    
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
    turn = 0
    oturn = 0
    counter = 10
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
        print("f1: {} f2: {} f3: {} f4: {}".format(ds[0].getValue(), ds[1].getValue(), ds[2].getValue(), 
                                            ds[3].getValue()))
        for ind in range(2):
            diff = ps_values[ind] - last_ps_values[ind]
            if diff < 1e-3:
                diff = 0
                ps_values[ind] = last_ps_values[ind]
            dist_values[ind] += diff * enc_un
            
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
        if ((m_ir_val == 1000) and (r_ir_val < 800) and (980 < l_ir_val < 1020)):# or ((980 < m_ir_val < 1020) and (r_ir_val < 800) and (980 < l_ir_val < 1020)):
            #turn left
            print("l")
            left_motor.setVelocity(0.25 * max_speed)
            right_motor.setVelocity(-0.25 * max_speed)
        elif ((m_ir_val == 1000) and (980 < r_ir_val < 1020) and (l_ir_val < 800)):# or (((980 < m_ir_val < 1000) and (980 < r_ir_val < 1020) and (l_ir_val < 800))):
            #turn right
            print("r")
            left_motor.setVelocity(-0.25 * max_speed)
            right_motor.setVelocity(0.25 * max_speed)
        elif ((m_ir_val != 1000) and (l_ir_val != 1000) and (r_ir_val != 1000) and (turn < 5)):
            #while ((m_ir_val != 1000) and (l_ir_val != 1000) and (r_ir_val != 1000)):
            print("here") 
            leftSpeed = 0.5 * max_speed
            rightSpeed = 0.5 * max_speed
            if avoidObstacleCounter > 0:
                avoidObstacleCounter -= 1
                leftSpeed = 2.0
                rightSpeed = -2.0
            else:  # read sensors
                for i in range(4):
                    if ds[i].getValue() < 950.0:
                        avoidObstacleCounter = 50
            left_motor.setVelocity(leftSpeed)
            right_motor.setVelocity(rightSpeed)
            turn += 1
        
        elif ((l_ir_val <= 800) and (r_ir_val < 800) and (980 < m_ir_val < 1020)):
            print("s")
            left_motor.setVelocity(0.25 * max_speed)
            right_motor.setVelocity(0.25 * max_speed)
        """ else:
            print("rot")
            left_motor.setVelocity(2)
            right_motor.setVelocity(-2) """
        if (turn > 5) and (turn-oturn == 0):
            
            print("av")
            counter -= 1
            leftSpeed = 2.0
            rightSpeed = -2.0
            left_motor.setVelocity(leftSpeed)
            right_motor.setVelocity(rightSpeed)
            if counter == 0:
                turn = oturn = 0 
                counter = 10

        for i in range(2):
            last_ps_values[i] = ps_values[i]
            
        oturn = turn
        
if __name__ == "__main__":

    # create the Robot instance.
    wheelie = Robot()
    run_robot(wheelie)