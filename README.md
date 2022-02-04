# A-webots-tutorial
A repository for a Webots tutorial series. The series contains a a few **world files**, **controllers** and **protos** as well.

The controllers are in some case far from the optimized version, since the aim was to create them as detailed as possible to understand what’s going on.

The list of the worlds and what they do:

0. **empty** - An empty world. How to create a project and an arena.

1. **SimpleShape** - Creating a world with a sphere in it. Adding physics and bounding objects.
2. **Wheelie** - the first differential drive robot, built by hand.
3. **ControlWheelie** - A simple controller for Wheelie. It can now go along a polygon. No PID.
4. **WheelieEncoders** - Adding some position sensors, computing pose. Movement along a polygon. no PID.
5. **LineFollower** - Wheelie follows now a line/finds the track if initially is set off-track. Obstacle avoidance included. With PID.
6. **turnForGivenTime** - A simple trick to get a turn with specific angle. Making a *delay* function to *sleep* our simulation**.**
7. **wallfollower** - Wheelie now follows a wall and solves a maze based on left wall following rule. With sensors and PID.
8. **mazeV1** - Wheelie follows a circular maze track. Makes complex decisions where to turn and stop. With sensors and PID.
9. **ReproducingSquare** - We create a new type of robot: the [Pololu Romi](https://www.pololu.com/product/3500/pictures). It will reproduce a square shaped trajectory. The corresponding encoder values for the trajectory were prerecorded and based on them the positions for each step computed. Now we do the job backward: knowing the positions we determine the needed velocities and reproduce the track.
10. **FeneralIKRomi** - a more general Inverse Kinematics for Romi. Given a (x,y) point in the plane Romi goes there avoiding possible obstacles.

## NOTE

It is important to note the whole project was created in, for the time being, current  Webots 2022a version. Not using this specific version may cause errors, it is recommended to download this, so you may want to look it up in the [Webots’ site archives](https://www.cyberbotics.com/doc/blog/menu).
