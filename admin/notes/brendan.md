# 20221017

- cartesian impedance control law is given on slide 16 of FRI powerpoint.
- control law requires setting a set point in joint space for the desired origin of the spring
  - does this mean that the end point can't be located outside the workspace of the arm (ie at infinity, with zero stiffness)?
  - do we need to solve the inverse kinematics of the robot?
- is it better to put the set point on the surface of the white board, and send a wrench, or is it better to allow the force to be generated through the stiffness in Cartesian space?
  - preliminary idea is to use the set point with a low stiffness spring normal to the whiteboard (where we need a constant force with an uncertain position) and then use the wrench to control the erasing action. 
