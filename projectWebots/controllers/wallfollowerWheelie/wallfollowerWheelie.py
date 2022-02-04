"""wallfollowerWheelie controller."""

from controller import Robot
timestep = 64
max_speed = 6.28

def run_robot(robot):
    "Wallfollower robot controller"
    global timestep
    global max_speed
    wheels = []
    #  motor1 left motor
    wheelsNames = ['motor1', 'motor2']
    for i in range(2):
        wheels.append(robot.getDevice(wheelsNames[i]))
        wheels[i].setPosition(float('inf'))
        wheels[i].setVelocity(0.0)
        
    ds = []
    dsNames = ['dsf1', 'dsf2', 'dsf3', 'dsf4','dsf5', 'dsf6', 'dsf7', 'dsf8']
    for i in range(8):
        ds.append(robot.getDevice(dsNames[i]))
        ds[i].enable(timestep)
        
    speed = [0, 0]

    while robot.step(timestep) != -1:
        for i in range(8):
            print("ind: {}, val: {}".format(i, ds[i].getValue()))
        #  processing data from sensors
        l_w = ds[4].getValue() < 450
        l_w_t = ds[4].getValue() > 650
        l_c = ds[3].getValue() < 450
        f_w = (ds[1].getValue() < 650) or (ds[2].getValue() < 650)
        #f_w_t = (450 < ds[1].getValue() < 650) or (450 < ds[2].getValue() < 650)
        
        print(l_w)
        if f_w:
            print("Turn right in place")
            speed[0] = max_speed
            speed[1] = -max_speed
        else:
            if l_w:
                print("Go forward")
                speed[0] = max_speed
                speed[1] = max_speed
            else:
                print("Turn left")
                speed[0] = max_speed/8
                speed[1] = max_speed
            if l_c:
                print("Too close: turn right")
                speed[0] = max_speed
                speed[1] = max_speed/8
            """"
            if f_w_t:
                print("Too close: turn right")
                speed[0] = max_speed
                speed[1] = max_speed/8
                """
            if l_w_t:
                print("Too far: turn left")
                speed[0] = max_speed/8
                speed[1] = max_speed

        for i in range(2):
            wheels[i].setVelocity(speed[i])

# Enter here exit cleanup code.

if __name__ == "__main__":
    wheelie = Robot()
    run_robot(wheelie)