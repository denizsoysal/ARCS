# ARCS

## Project Goal

See docs/project_description.md for more details.

The goal is to erase characters off of a whiteboard using a bimanual robot arm system. This involves the following "skills":

1. Classify a whiteboard as clean or dirty.
2. Identify dirty areas of the whiteboard that require cleaning.
3. Grasp the whiteboard with a free arm so that it will not move while erasing.
4. Move a cleaning tool (eraser) along the surface of the whiteboard to clean dirty regions with the other arm.
5. Check whether the attempted erasing action was successful.

The followoing is a description of what has been implemented thus far.

# Architecture of Activities

<img src="docs/figs/activity_architecture.svg">

This figure is a layout of the **currently implemented** activity architecture in the iiwase project. The headers of the objects provide the activity name, as well as the thread time for the activity. The rows below the header provide important variables which are present in each activity. The arrows indicate a "modifies" relation, where the value of a variable in one activity is copied to the destination variable in a different activity. Provided below is a high level description of the activities used, and their respective responsibilities. 

## perception
TODO...

## iiwa_activity
- real time communication with the KUKA FRI and Sunrise workbench.
- determines the cmd mode (Position, JntTorque, or Wrench).
- Provides reading of the sensor data and communication of actuation commands back to the iiwa. 

## iiwa_estimation
- filtering of noisy joint position encoder readings to estimate the state of the robot (real joint positions, joint velocities)
- differentiates the joint positions to obtain joint velocities. This involves measuring it's own cycle time.
- Performs forward kinematics to determine the cartesian positions and velocities of the robot. 

## navigation
- responsible for specifying the type motion that the arm should conduct.
- The motion is specified by a heading vector (in the base frame) which is a direction that the arm must move in, and a velocity magnitude which it should be moving.
- the navigation activity is the interface with the perception activity which sends positions where the arm should be heading in order to accomplish this task. 

## iiwa_controller
- Sends wrench commands to the iiwa activity in order to achieve the desired heading and velocity magnitude. 
- Implements a thresholded adaptive controller (ABAG) to ensure that force applied at end effector is always "human safe" and only applies force in the necessary direction for the task. 

# Perception
TODO

# Control 

## Implementation Details

### Force Based Cartesian Velocity Control (Free Space Motion)

- Our task is to erase a whiteboard, which involves moving an erasing tool over a flat surface, while exerting some normal force and remaining in contact with that surface. Therefore, we decided that force control was a requirement for our task. 
- KUKA FRI does not allow "switching" control modes (ie between position and force control) online, so we decided to implement a cartesian velocity controller to **move in free space** with force control mode. 
- We could have used the available "impedance control" mode available through the KUKA FRI (equation below). This would involve setting a stiffness k and just moving the point $x_{msr}$ around in free space. This would be a far easier approach. However, it has the following disadvantages:
  - The 'error' term is position based, so the joint torques are proportional to the deviation from a desired trajectory, or time sequences of positions in free space. We did not feel like this was a good fit for our application, since we don't care how the arm approaches the whiteboard, therefore, we did not want to specify an explicit trajectory.
  - There could be safety issues depending on how far we deviate from our desired trajectory, since the control input increases proportionally. Therefore, unforeseen deviations, such as an obstacle, would result in large forces applied to the end effector. 
  - There is no control on the velocity apart from the trajectory specification. Therefore, if the arm is perturbed from the desired trajectory, the control law will exert large joint torques causing it to accelerate until $(x_{FRI} - x_{msr})=0$. Therefore, the resulting velocity of the arm could be much higher than the desired velocity of the trajectory, leading to more safety issues.
  - Overall, the above issues fall under the category of "adding energy to the system proportional to the error", which we did not feel was appropriate for our application. 

$$\tau_{cmd} = J^T[k_c(x_{FRI}-x_{msr}) + W_{FRI}] + D(d_c) + f_{dynamics}(q, \dot{q}, \ddot{q})$$

- We decided instead to eliminate the stiffness term by setting $x_{msr}$ to $x_{FRI}$ and implement a controller that would set $W_{FRI}$ based on the velocity measurement. This controller has the following advantages:
  - It can be made inherently safe by saturating the $W$ term ensuring that the force applied to the end effector is never greater than a maximum value.
  - Any control law can be chosen for $W$, not one that is position dependent, meaning that there is no penalization (or extra control effort) exerted for large deviations from the trajectory, in the case of hitting unforseen obstacles for example. 
  - Instead of **implicitly** controlling the force through a stiffness and a displacement, we are **explicitly** controlling a force to the end effector to move. 

### ABAG Velocity Control

- For velocity control, we use the control law outlined in [1]. We use it for the following reasons:
  - the output is inherently saturated, meaning that we have a bounded force that can be applied at the end effector
  - the gain (analogous to a proportional term) and the bias (analogous to the integral term) are adaptive, and 'tuned' by the controller itself depending on the current state of the system to provide the desired motion specification. 

```c++
void abag(abag_params_t *params, abag_state_t *state, double setpoint, double val){
	state->ek_bar = params->alpha * state->ek_bar + (1 - params->alpha) * sgn(setpoint - val);
	state->bias = saturate(state->bias + 
	    params->delta_bias * hside(fabs(state->ek_bar)-params->bias_thresh) * sgn(state->ek_bar-params->bias_thresh), 
		params->sat_low, params->sat_high);
	state->gain = saturate(state->gain + 
	    params->delta_gain * sgn(fabs(state->ek_bar) - params->gain_thresh), 
		params->sat_low, params->sat_high);
	state->control = saturate(state->bias + state->gain * sgn(setpoint - val), 
	    params->sat_low, params->sat_high);

	return;
}
```
(Above) shows the implementation of the controller outlined in [1]. (Below) shows the values set for the 'hyperparameters' of the controller which determines how the bias and gain terms adapt. Both code blocks from from `iiwa_controller.cpp`.

```cpp
params->max_torque = 2.0;
params->max_force = 5.0;

// Configure ABAG Controller
params->abag_params_cartesian.sat_high = 1;
params->abag_params_cartesian.sat_low = -1;

    // parameters from paper
// TODO we should set alpha to remove the moving avg filter here eventually
// TODO we should do all filtering in the estimation
params->abag_params_cartesian.alpha = 0.75;
params->abag_params_cartesian.bias_thresh = 0.75;
params->abag_params_cartesian.delta_bias = 0.001;
params->abag_params_cartesian.gain_thresh = 0.5;
params->abag_params_cartesian.delta_gain = 0.001;
```

- The 'error' term in the ABAG controller is the difference between the desired velocity and the actual velocity. Below is a code block were we call the abag controller implementation. As can be seen, the abag controller has a state, which is the current set of gains it is using. 

```cpp
abag(&params->abag_params_cartesian, &cts_state->abag_state_x, cts_state->local_velocity_magnitude * cts_state->local_heading[0], local_vel[0]);
```

[1] A. Franchi and A. Mallet, “Adaptive closed-loop speed control of BLDC motors with applications to multi-rotor aerial vehicles,” in 2017 IEEE International Conference on Robotics and Automation (ICRA), May 2017, pp. 5203–5208. doi: 10.1109/ICRA.2017.7989610.

### Two Approaches for Velocity Specification and Control

For the following section, the following symbols apply:
- $\overrightarrow{v_{ee}}$ is the end effector velocity vector, measured from the iiwa.
- $\overrightarrow{h}$ is the heading vector, generated from the state estimation (current position) and the perception (position we want to go towards). $\overrightarrow{h} = x_{des} - x_{ee}$. 
- $v_{des}$ (along with h) is the other part of the motion specification. It is a scalar magnitude of a velocity that we want to have in free space, travelling in the direction of $\overrightarrow{h}$. 
- $\overrightarrow{F}$ is the force applied, previously called $W$, to the KUKA.
- $u_{ABAG}$ is the output of the ABAG controllers mentioned above.


#### 1. Use a single ABAG controller and only control the velocity components along the heading vector $\overrightarrow{h}$ that you want to be travelling in. 

$$\Delta v = \|\frac{\overrightarrow{v_{ee}}\cdot\overrightarrow{h}}{\|\overrightarrow{v_{ee}}\|^2}\overrightarrow{h}\| - v_{des}$$
$$\overrightarrow{F} = F_{max} * u_{ABAG}(\Delta v) * \overrightarrow{h}$$

#### 2. Use an ABAG controller for each component (xyz) of the velocity. 

$$\Delta v_x = \overrightarrow{v_{ee}}[x] - v_{des}*\overrightarrow{h}[x]$$
$$\Delta v_y = \overrightarrow{v_{ee}}[y] - v_{des}*\overrightarrow{h}[y]$$
$$\Delta v_z = \overrightarrow{v_{ee}}[z] - v_{des}*\overrightarrow{h}[z]$$

$$\overrightarrow{F}[x] = F_{max} * u_{ABAG}(\Delta v_x) * \overrightarrow{h}[x]$$
$$\overrightarrow{F}[y] = F_{max} * u_{ABAG}(\Delta v_y) * \overrightarrow{h}[y]$$
$$\overrightarrow{F}[z] = F_{max} * u_{ABAG}(\Delta v_z) * \overrightarrow{h}[z]$$

### Velocity Estimation (Filtering)



  - what type of filter do we choose and why?

<img src="docs/figs/jnt_vel_signal.svg">

Some comments about this graph here...

<img src="docs/figs/vel_signal_hist.svg">

Some comments about this other graph...**DO NOT USE A KALMAN FILTER FOR VELOCITY ESTIMATION**

## Future Implementation (TODO)
- orientation control with another ABAG
- orientation of the entire robot arm to avoid singularities and position better to sense forces
