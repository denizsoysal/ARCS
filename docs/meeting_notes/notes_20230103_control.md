# Todo:
1. Update the spring position with in the cartesian impedance control interface to remove effect of spring constant.
2. Generate a motion specification instead of using a constant velocity.

# 20220103 Notes:

## Debugging implementation of (1)
- I am first attempting to remove the influence of the spring term of the FRI by setting the cmd jnt velocity of the robot to be the meas jnt velocity. However, I am concerned about this approach for several reasons:
  - We don't actually know where $q_{meas}$ is in the FRI. Ideally, you would want to set this value to the actual end effector position at each timestamp. However, we don't have access to this parameter. Therefore, since it is set indirectly by integrating a velocity, the error on that position will build up. 

## Questions on implementation of (1)
1. Where is the frame that we are commanding a wrench in?
2. How do we actually command an orientation of the robot?
  - when we tell it to move vertically up, I don't get a good vertical up motion, and the end effector is changing orientation. Do I need to compensate for this in the wrench I command?

## Next Steps
1. Take some videos and make some notes of the issues we are facing.
2. Reach out to Federico to see if we have direct access to $q_{meas}$. 
3. Discuss with Herman on how to control the orientation of the robot as well. Is it better to use joint torque control mode?
