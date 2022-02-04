"""Wheelie_driver_with_encoder controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor


#Here will be used Odometry calculations
from controller import Robot
import math

def run_robot(robot):
    #timestep of the current world
    timestep = 64
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
        
        vx = v * math.cos(robot_pose[2])
        vy = v * math.sin(robot_pose[2])
        
        robot_pose[0] += vx*dt
        robot_pose[1] += vy*dt
        
        print("robot pose: {} {} {}".format(robot_pose[0], robot_pose[1], robot_pose[2]))
        
        #with rotation we can show that the encoder values difference will grow significantly
        left_speed =  max_speed
        right_speed = max_speed
        
        if rot_start < current_time < rot_end:
            left_speed = - max_speed
            right_speed = max_speed
        elif current_time > rot_end:
            rot_start = current_time + side_time
            rot_end = rot_start + rot_time
        
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        
        for i in range(2):
            last_ps_values[i] = ps_values[i]
        
if __name__ == "__main__":

    # create the Robot instance.
    wheelie = Robot()
    run_robot(wheelie)
    