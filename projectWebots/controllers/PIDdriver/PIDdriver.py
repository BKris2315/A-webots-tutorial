"""Wheelie_driver controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# PID parameters
kp = 0.03
ki = 1e-4
kd = 0.5
last_error = differ = intg = prop =0

KP = 1.7
KI = 0.2
KD = 0.

theoretical_position = [0, 0]
P_error = [0, 0]
I_error = [0, 0]
D_error = [0, 0]

timestep = 64

def integrate_trajectory_Euler(set_speed_lr,dt_local):
	global theoretical_position
	theoretical_position[0]+=dt_local*set_speed_lr[0]
	theoretical_position[1]+=dt_local*set_speed_lr[1]

def pid_control_pos(velocity):
    global dt
    global theoretical_position
    global I_error
    global P_error
    global KI
    global PI
    global PD
    global current_time, prev_time, rot_start, rot_end, rot_time, side_time, timesteps
    global wheelie
    
    measured_encoder_value = [wheelie.getPositionSensor('ps1').getValue(), wheelie.getPositionSensor('ps2').getValue()]
    for i in range(2):
    
        P_error[i] = (theoretical_position[i] - measured_encoder_value[i])*KP
        I_error[i] += (theoretical_position[i] - measured_encoder_value[i])*KI
    vx = (velocity[0]+P_error[0]+I_error[0])/17
    vy = (velocity[1]+P_error[1]+I_error[1])/17

    integrate_trajectory_Euler(velocity, timestep/1000)
    
    return vx, vy
    #print(theoretical_position-measured_encoder_value)
	
	
def pid(error):
    global kp, ki, kd, intg, differ, last_error, prop
    prop = error
    intg = error + intg
    diff = error -last_error
    balance = kp * prop +  ki * intg + kd * diff
    last_error = error
    return balance

def setspeed(max_speed, balance):
    left_motor.setVelocity(max_speed + balance)
    right_motor.setVelocity(max_speed - balance)

"""Wheelie_driver_with_encoder controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor


#Here will be used Odometry calculations
from controller import Robot
import math

def run_robot(robot):
    #timestep of the current world
    global timestep
    ps_values = [0, 0]
    dist_values = [0, 0]
    max_speed = 6.28
    wheel_radius = 0.025
    dist_betw_wheel = 0.09
    
    #motor instances
    right_motor = robot.getDevice('motor1')
    left_motor = robot.getDevice('motor2')

    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)   
    
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)
    
    # instances for encoders
    left_ps = robot.getPositionSensor('ps1')
    left_ps.enable(timestep)
    
    right_ps = robot.getPositionSensor('ps2')
    right_ps.enable(timestep)
    
    wheel_circ = 2 * 3.1415 * wheel_radius
    enc_un = wheel_circ / 6.283
    
    #Robot pose
    
    robot_pose = [0, 0, 0]  #x, y, theta
    last_ps_values = [0, 0]
    
    num_side = 4
    side_length = 0.25
    
    lin_vel = wheel_radius * max_speed
    side_time = side_length / lin_vel
    
    start_time = robot.getTime()
    
    angle_of_rot = (num_side-2) * 3.1415 / num_side
    rot_rate = (2 * lin_vel) / dist_betw_wheel
    rot_time = angle_of_rot / rot_rate
    
    rot_start = start_time + side_time
    rot_end = rot_start + rot_time
    
    vx = max_speed
    vy = max_speed
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
        
        current_time = robot.getTime()
                
        ps_values[0] = left_ps.getValue()
        ps_values[1] = right_ps.getValue()
        
        
        print("--------------------------")
        print("position sensor values: {} {}".format(ps_values[0], ps_values[1]))
        
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
        
        if rot_start < current_time < rot_end:
            vx = - max_speed
            vy = max_speed
        elif current_time > rot_end:
            rot_start = current_time + side_time
            rot_end = rot_start + rot_time
        else:  
            vx, vy = pid_control_pos([vx, vy])
        print("here {} {}".format(vx, vy))
        #vy = v * math.sin(robot_pose[2])
        
        robot_pose[0] += vx*dt
        robot_pose[1] += vy*dt
        
        print("robot pose: {} {} {}".format(robot_pose[0], robot_pose[1], robot_pose[2]))
        print("time: {} ".format(current_time))
        #with rotation we can show that the encoder values difference will grow significantly

        

        
        left_motor.setVelocity(vx)
        right_motor.setVelocity(vy)
        
        for i in range(2):
            last_ps_values[i] = ps_values[i]
        
if __name__ == "__main__":

    # create the Robot instance.
    wheelie = Robot()
    run_robot(wheelie)