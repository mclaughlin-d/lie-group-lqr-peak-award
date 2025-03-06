# Intuitive Robotic Arm Control Using Lie Groups - PEAK Summit Award 2025
This repo has code for our 2025 PEAK Summit Award, "Intuitive Robotic Arm Control Using Lie Groups". It will be updated as the project progresses.

#### System Description
This projects specifically focuses on the differential wrist in the 6-DOF robot arm that is a part of Watney, Mark VI, a rover-style robot created by the Northeastern University Mars Rover Team (if you'd like to learn more about the team, see this video [here](https://www.youtube.com/watch?v=9GoSA4WFsdQ)). The differential wrist in the robotic arm is controlled using two Dynamixel "smart actuators". Images of this wrist are shown below.

end effector | gear mechanism close-up
:------------:|:-------------:|
![diff_wrist](https://github.com/user-attachments/assets/53604854-ed34-4363-9c90-5ffdaef01f83) | ![diff_wrist_closeup](https://github.com/user-attachments/assets/6d514f65-7d9a-4005-b486-da0540930cb8)



## Lie Group Differential Wrist Control
The first part of this project was to implement a control scheme for a differential wrist such that infinite roll rotations (and pitch rotations) are not allowed. This can be done with standard angle representation of positions, however, the code can get a little ugly when checking for these conditions. We wanted a simpler and more mathematically elegant solution, which is why we chose to convert our pitch/roll representations to SO(2) and do math on them that way.

### Theory
SO(2) groups are 2x2 orthogonal matrices that represent rotations in 2D Euclidian space. For systems which allow infinite rotations, SO(2) representation of angles is beneficial because "equivalent" angles (such as 20° and 380°) are represented by the same group element.

The differential wrist is controlled by converting goal pitch ($\theta_{gp}$) and goal roll ($\theta_{gr}$) angles to goal angles for each of the Dynamixels in the wrist. In our implementation, we convert these angles, as well as current pitch ($\theta_{cp}$) and current roll ($\theta_{cr}$) to their corresponding SO(2) group object. We then determine which SO(2) group objects gets from the current pitch/roll (in SO(2)) to the goal pitch/roll (in SO(2)). We extract the angle increment/decrement from this SO(2) group object and then use it to update the actual goal pitch/roll before moving the Dynamixel actuators. 

An example using pitch is as follows: 
First, calculate the SO(2) group object corresponding to the change in pitch (\theta_{dp}):
```math
D = \begin{bmatrix} \cos(\theta_{dp}) & -\sin(\theta_{dp}) 
\\ \sin(\theta_{dp}) & \cos(\theta_{dp}) \end{bmatrix} = 

\begin{bmatrix} \cos(\theta_{cp}) & -\sin(\theta_{cp}) 
\\ \sin(\theta_{cp}) & \cos(\theta_{cp}) \end{bmatrix}^{-1}
\begin{bmatrix} \cos(\theta_{gp}) & -\sin(\theta_{gp}) 
\\ \sin(\theta_{gp}) & \cos(\theta_{gp}) \end{bmatrix}

```

Then, calculate the absolute value and the direction of the angle delta:

```math
\theta_{dp} = \arccos(D_{00}) * (D_{10} / |D_{10}|)
```

Once the same is done for roll (as in, you have both $\theta_{dp}$ and $\theta_{dr}$), calculate the new goal pitch and roll. Then, calculate the diff angles ($\theta_{d1}$ and $\theta_{d2}$):

```math
\theta_{gp} = \theta_{cp} + \theta_{dp}
```

```math
\theta_{gr} = \theta_{cr} + \theta_{dr}
```
 
```math
\theta_{d1} = \theta_{gr} - \theta_{gp}
```

```math
\theta_{d2} = \theta_{gp} + \theta_{gr}
```

### Implementation
We use `NumPy` and standard Python `math` module trig functions to convert to/from SO(2) and perform matrix operations (multiplication, inverse). The `lie_group_pitch_roll` function found in `dynamixel_control.py` handles this conversion and performs the math operations explained in the previous section to get pich/roll increments. These are then added to the goal pitch and roll so they are updated before the Dynamixel angles are calculated and written. The rest of the code in that file handles actually talking to and controlling the dynamixels. 

### Results
_Cool photos/videos will be added shortly!_

## LQR Control
_This will be implemented in next month or so! Anticipating completion at the end of March 2025_
