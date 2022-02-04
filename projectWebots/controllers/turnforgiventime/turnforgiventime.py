"""Wheelie_driver controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot


if __name__ == "__main__":

    # create the Robot instance.
    robot = Robot()
    
    # get the time step of the current world.
    timestep = 1
    max_speed = 6.28
    wheel_radius = 0.025
    dist_betw_wheel = 0.09
    prev_time = 0
    # You should insert a getDevice-like function in order to get the
    # instance of a device of the robot. Something like:
    #  motor = robot.getDevice('motorname')
    #  ds = robot.getDevice('dsname')
    #  ds.enable(timestep)
    
    
    #We have created motor instances
    right_motor = robot.getDevice('motor1')
    left_motor = robot.getDevice('motor2')
    
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)   
    
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)
    
    num_side = 4
    side_length = 0.25
    
    lin_vel = wheel_radius * max_speed /4
    side_time = side_length / lin_vel
    
    start_time = robot.getTime()
    
    angle_of_rot = (num_side-2) * 3.1415 / num_side
    rot_rate = (2 * lin_vel) / dist_betw_wheel
    rot_time = 1.5754 / rot_rate
    
    rot_start = start_time + side_time
    rot_end = rot_start + rot_time
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    turnstart = 0
    while robot.step(timestep) != -1:
    
        current_time = robot.getTime()
        print(current_time)

        if current_time%6.4 == 0:
            turnstart = robot.getTime()
        
        # Read the sensors:
        # Enter here functions to read sensor data, like:
        #  val = ds.getValue()
    
        # Process sensor data here.
    
        # Enter here functions to send actuator commands, like:
        #  motor.setPosition(10.0)

        if turnstart != 0 and current_time< turnstart+rot_time: turn = 1
        else: turn = 0

        if turn:
            left_speed = - max_speed/4
            right_speed = max_speed/4
        else:
            left_speed = 0
            right_speed = 0
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        
        prev_time = current_time
    
    # Enter here exit cleanup code.
