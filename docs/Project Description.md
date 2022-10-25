## Goal
When a whiteboard (on wheels) is detected within range of the robot, the two arms will grab it and move it in a suitable position to erase specific parts of it, meaning only parts with writing/drawings.
The erasing will be done with one arm while holding the whiteboard with the other. 
Additional skills that can be added are to:
- only erase specific letters/words,
- turn the whiteboard 180° and also erase the other side.
Finally, everything should be done while avoiding obstacles. The robot arms should avoid obstacles themselves but should also make sure the whiteboard does not collide with anything while moving it.

## Actions
1. Start task: Erasing process starts with a given cue, e.g. the whiteboard is rolled into the robots range.
2. Orientation of whiteboard: Grab the board with both arms and bring it to a position in which the area to be erased is reachable, two arms are necessary to be able to rotate the board more easily and move it sideways in a controllable way.
3. Keep holding the board with one arm, grab the eraser with the other and start erasing. This behaviour is inspired by how a human would erase a whiteboard on wheels to prevent it rolling away.
4. If other parts of the whiteboard have writing on it which is not reachable after this, grab the board again and move it over to erase the other parts. Repeat untill whiteboard is clean.
5. Additional skill: If one side of the whiteboard is clean, rotate the board to clean the other side.

This sequence can be split into two tasks: orientation and erasing, which the robot must alternate between to complete the overarching goal of cleaning the whiteboard. Both tasks are described in more detail below. 

### 1. Orientation of Whiteboard
This aspect of the project deals with moving the whiteboard to a suitable location. A user should be able to place the whiteboard in a skewed orientation in front of the robot, and have the robot move the board to an easier location to erase the characters. The robot should also be able to move the whiteboard in the case that it cannot reach the entire surface during erasing.

#### Control Elements
1.  Understand when the task can start (ie, when can the robot start interacting with the whiteboard?)
2.  Once the desired grasp location is known, how do we move the arms to that location and orient it with respect to the edge?
3.  How do we initiate a grasp sequence? What grasping hardware do we need? How do we know that a grasp has been successful?
4.  When we know that we are holding the whiteboard, control of the multi arm system to move the whiteboard to the desired location.

#### Perception Elements
1.  Understand when the whiteboard is in the frame?
2.  Extract the desired grasp locations, or "graspable" features from the whiteboard images.
3.  Transform those features to a desired location and pose for the gripper.
4.  Understand when the whiteboard is in the desired final location.

### 2. Erasing of Whiteboard

#### Control Elements
1.  Grasping the sponge to erase the whiteboard (how to grasp, and how to know we are successful?
2.  Applying the correct force to the normal surface of the whiteboard to ensure that the sponge is always in contact, but is still able to move to the left and right.
3.  Monitors so that at any time a human can regain control of the system. This could be implemented by the human applying a force to the robot, or visually, when the camera detects a human in the frame. Robot should signal that it is ready to hand off control to the human.
4.  While erasing with one arm, using the other arm to stabilize the whiteboard.

#### Perception Elements
1.  Identifying dirty areas on the whiteboard, and prioritizing the ones to erase first.
2.  Confirming that a dirty area has been erased.
3.  Distinguishing characters from each other, if we only want to erase parts of words or spelling mistakes.
4.  Understand spacing between letters and be able to separate words.


## Other Decisions

### Camera Placement and Type
1. Attached to one of the arms
	1. Normal camera (structure from motion) -> No feedback on position of whiteboard when grabbed with two arms
3. Stationary
	1. Normal camera -> no depth information, unable to grab the board without first bumping into it
	2. Camera + some depth information
		1. In principle only depth information needed from 1 point on the whiteboard but this requires movement of laser pointer. A 2D Lidar would suffice but since RGBD camera is available this is chosen above a separate camera + Lidar.
		2. The robot arms can block the view of the camera

### Control mode: 
Cartesian impedance. The idea is to put the desired position behind the board and have a relatively soft impedance in the direction perpendicular to the whiteboard surface to make sure the eraser stays in contact with it and at the same not push it backwards too much. The impedance for the two directions in the plane of the whiteboard should be higher to be able to overcome the friction and control the position of the eraser on the whiteboard with relatively high accuracy.

## What makes this project advanced?
- Coordination between two separate robot arms
- Decision making: where to move the whiteboard, what to erase

## What makes this project relevant?
- Manipulating larger objects by coordination of multiple robot arms
- Smart interaction with the environment by using perception
