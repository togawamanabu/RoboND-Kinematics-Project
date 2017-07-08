## Project: Kinematics Pick & Place

togawa manabu

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code.


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc2.png
[image3]: ./misc_images/misc3.png

[image4]: ./misc_images/robbot-image.png
[image5]: ./misc_images/diaglam1.jpg

[image6]: ./misc_images/theta3-2.png

[image7]: ./misc_images/run1.png
[image8]: ./misc_images/run2.png



## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README





### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Start forwad_kinematics demo with  ```$ roslaunch kuka_arm forward_kinematics.launch```

Each joints controllable from ```from joint_state_publisher``` panel. also each link can visible / hide from ```Links``` checkbox in left window.

![Forward kinematics][image4]

Joint rotations figure
![Joint rotation figure][image5]


joints code are written in ```kuka_arm/urdf/kr210.urdf.xacro```

from line 316 is joints information
```
<!-- joints -->
<joint name="fixed_base_joint" type="fixed">
  <parent link="base_footprint"/>
  <child link="base_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
<joint name="joint_1" type="revolute">
  <origin xyz="0 0 0.33" rpy="0 0 0"/>
  <parent link="base_link"/>
  <child link="link_1"/>
  <axis xyz="0 0 1"/>
  <limit lower="${-185*deg}" upper="${185*deg}" effort="300" velocity="${123*deg}"/>
</joint>
<joint name="joint_2" type="revolute">
  <origin xyz="0.35 0 0.42" rpy="0 0 0"/>
  <parent link="link_1"/>
  <child link="link_2"/>
  <axis xyz="0 1 0"/>
  <limit lower="${-45*deg}" upper="${85*deg}" effort="300" velocity="${115*deg}"/>
</joint>
<joint name="joint_3" type="revolute">
  <origin xyz="0 0 1.25" rpy="0 0 0"/>
  <parent link="link_2"/>
  <child link="link_3"/>
  <axis xyz="0 1 0"/>
  <limit lower="${-210*deg}" upper="${(155-90)*deg}" effort="300" velocity="${112*deg}"/>
</joint>
<joint name="joint_4" type="revolute">
  <origin xyz="0.96 0 -0.054" rpy="0 0 0"/>
  <parent link="link_3"/>
  <child link="link_4"/>
  <axis xyz="1 0 0"/>
  <limit lower="${-350*deg}" upper="${350*deg}" effort="300" velocity="${179*deg}"/>
</joint>
<joint name="joint_5" type="revolute">
  <origin xyz="0.54 0 0" rpy="0 0 0"/>
  <parent link="link_4"/>
  <child link="link_5"/>
  <axis xyz="0 1 0"/>
  <limit lower="${-125*deg}" upper="${125*deg}" effort="300" velocity="${172*deg}"/>
</joint>
<joint name="joint_6" type="revolute">
  <origin xyz="0.193 0 0" rpy="0 0 0"/>
  <parent link="link_5"/>
  <child link="link_6"/>
  <axis xyz="1 0 0"/>
  <limit lower="${-350*deg}" upper="${350*deg}" effort="300" velocity="${219*deg}"/>
</joint>
```

gripper joint information from line 202
```
<joint name="gripper_joint" type="fixed">
  <parent link="link_6"/>
  <child link="gripper_link"/>
  <origin xyz="0.0375 0 0" rpy="0 0 0"/>
</joint>
```


DH parameter table

from both diaglam and urdf file create dh parameter table

alpha = twist angle between z-1 and z (right hand)
a = distance from z-1 to z, measured along x
d = offset distance from x-1 to x, measured along z
q(theta) = angle between x-1 and x

i   | alpha | a     | d           | q (theta)
--- | ---   | ---   | ---         | ---
1   | 0     |   0   | 0.75        | q1
2   | -pi/2 |  0.35 | 0           | q2-pi/2
3   | 0     | 1.25  | 0           | q3
4   | -pi/2 | -0.054| 1.50        | q4
5   | pi/2  |   0   | 0           | q5
6   | -pi/2 |   0   | 0           | a6
7   | 0     |   0   | 0.303       | 0


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

```
    # Define DH param symbols
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

    # Joint angle symbols
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

    #DH parameters
    s = {alpha0: 0,     a0: 0,      d1: 0.75,
         alpha: -pi/2, a1: 0.35,   d2: 0,     q2: q2-pi/2,
         alpha2: 0,     a2: 1.25,   d3: 0,
         alpha3: -pi/2, a3: -0.054, d4: 1.50,
         alpha4: pi/2,  a4: 0,      d5: 0,
         alpha5: -pi/2, a5: 0,      d6: 0,
         alpha6: 0,     a6: 0,      d7: 0.303, q7: 0}

    #Homogeneous Transforms
    T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
                   [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
                   [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
                   [                   0,                   0,            0,               1]])
    T0_1 = T0_1.subs(s)

    T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
                   [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
                   [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
                   [                   0,                   0,            0,               1]])
    T1_2 = T1_2.subs(s)

    T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
                   [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                   [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
                   [                   0,                   0,            0,               1]])
    T2_3 = T2_3.subs(s)

    T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
                   [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
                   [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
                   [                   0,                   0,            0,               1]])
    T3_4 = T3_4.subs(s)

    T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
                   [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
                   [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
                   [                   0,                   0,            0,               1]])
    T4_5 = T4_5.subs(s)

    T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
                   [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
                   [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
                   [                   0,                   0,            0,               1]])
    T5_6 = T5_6.subs(s)

    T6_G = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
                   [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
                   [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
                   [                   0,                   0,            0,               1]])

    T6_G = T6_G.subs(s)

    # Transform from base link to end effector
    T0_G = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G)

    #end-effector position & rotation
    # r= end-effector's roll, p= pitch, y = yaw

    #rotation matrices
    R_roll = Matrix([[ 1,         0,          0],
                     [ 0, cos(r), -sin(r)],
                     [ 0, sin(r), cos(r)]])

    R_pitch = Matrix([[ cos(p),  0, sin(p)],
                      [          0,  1,          0],
                      [-sin(p),  0, cos(p)]])

    R_yaw = Matrix([[ cos(y), -sin(y), 0],
                    [ sin(y),  cos(y), 0],
                    [        0,         0, 1]])

    R0_6 = R_roll * R_pitch * R_yaw

```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.


Step 1: complete the DH parameter
  above

Step 2: find location of WC relative to the base

$$
WC =
  \left[
   \begin{array}{r}
     px  \\
     py  \\
     pz
   \end{array}
 \right] - dy \cdot r06
 \left[
  \begin{array}{r}
    0  \\
    0  \\
    1
  \end{array}
\right]

$$

Step 3: find theta1
$$
\theta1 = atan2(y_{wc} , x_{wc})
$$

Step 4: find theta3

image from slack channel
![from slack chunnel][image6]

$$
\theta = arccos(\frac{b^2 + c^2 - a^2}{2 b c})
$$

Step 5: find theta2

$$
\theta = atan2(y_c,xc) + atan2(\sqrt{1-cos^2(q_21), cos(q_21)})
$$

Step 6: find theta4, theta5, theta6

theta4, 5, 6 calculate with tf.transformations.euler_from_matrix

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.

It was very hard to understood but good study.

Implimented these concept in ```scpipts/IK_server.py```.

Calclating inverse kinematics takes time, maybe still have space to improve performance.

-> Native Ubuntu embironment is much much faster compare to Ubuntu on VM.

![test running][image7]
![test running][image8]
