
## Project: Kinematics Pick & Place

The goal of this project is to write code to perform Forward and Inverse Kinematic for the Kuka KR210:

* Setup environment properly
* Perform Kinematic Analysis of the robot and derive equations for individual joint angles.
* Implement Kinematic code in `IK_server.py`

[//]: # (Image References)

[image1]: ./misc_images/fk_1.png
[image2]: ./misc_images/fk_2.png
[image3]: ./misc_images/fk_3.png
[image4]: ./misc_images/fk_4.png
[image5]: ./misc_images/fk_5.png
[image6]: ./misc_images/fk_6.png
[image7]: ./misc_images/fk_7.png
[image8]: ./misc_images/dh_transform.png
[image9]: ./misc_images/dh_transform_matrix.png
[image10]: ./misc_images/dh_transform_0_ee.png
[image11]: ./misc_images/dh_parameters.png
[image12]: ./misc_images/homogeneous_trans.png
[image13]: ./misc_images/r_corr.png
[image14]: ./misc_images/wc.png
[image15]: ./misc_images/theta1.gif
[image16]: ./misc_images/theta2_theta3.png
[image17]: ./misc_images/r3_6.png
[image18]: ./misc_images/extrinsic_rotation.png
[image19]: ./misc_images/alpha.png
[image20]: ./misc_images/beta.png
[image21]: ./misc_images/gamma.png
[image22]: ./misc_images/kuka.png
[image23]: ./misc_images/misc2.png
[image24]: ./misc_images/test1.png
[image25]: ./misc_images/test2.png
[image26]: ./misc_images/test3.png
[image27]: ./misc_images/ki1.jpg
[image28]: ./misc_images/ki2.jpg
[image29]: ./misc_images/ki3.jpg

---

![alt text][image23]


### One time Gazebo setup step:
Check the version of gazebo installed on your system using a terminal:
```sh
$ gazebo --version
```
To run projects from this repository you need version 7.7.0+
If your gazebo version is not 7.7.0+, perform the update as follows:
```sh
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo7
```

Once again check if the correct version was installed:
```sh
$ gazebo --version
```
### For the rest of this setup, catkin_ws is the name of active ROS Workspace, if your workspace name is different, change the commands accordingly

If you do not have an active ROS workspace, you can create one by:
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now that you have a workspace, clone or download this repo into the **src** directory of your workspace:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/udacity/RoboND-Kinematics-Project.git
```

Now from a terminal window:

```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ sudo chmod +x target_spawn.py
$ sudo chmod +x IK_server.py
$ sudo chmod +x safe_spawner.sh
```
Build the project:
```sh
$ cd ~/catkin_ws
$ catkin_make
```

Add following to your .bashrc file
```
export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/models

source ~/catkin_ws/devel/setup.bash
```

For demo mode make sure the **demo** flag is set to _"true"_ in `inverse_kinematics.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch

In addition, you can also control the spawn location of the target object in the shelf. To do this, modify the **spawn_location** argument in `target_description.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch. 0-9 are valid values for spawn_location with 0 being random mode.

You can launch the project by
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ ./safe_spawner.sh
```

If you are running in demo mode, this is all you need. To run your own Inverse Kinematics code change the **demo** flag described above to _"false"_ and run your code (once the project has successfully loaded) by:
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ rosrun kuka_arm IK_server.py
```
Once Gazebo and rviz are up and running, make sure you see following in the gazebo world:

	- Robot
	
	- Shelf
	
	- Blue cylindrical target in one of the shelves
	
	- Dropbox right next to the robot
	

If any of these items are missing, report as an issue.

Once all these items are confirmed, open rviz window, hit Next button.

To view the complete demo keep hitting Next after previous action is completed successfully. 

Since debugging is enabled, you should be able to see diagnostic output on various terminals that have popped up.

The demo ends when the robot arm reaches at the top of the drop location. 

There is no loopback implemented yet, so you need to close all the terminal windows in order to restart.

In case the demo fails, close all three terminal windows and rerun the script.


### Kinematic Analysis

#### Step 1. Sketch the robot arm in its zero configuration

1. Label joints from 1 to n
![alt text][image1]


2. Define joint axes
![alt text][image2]


3. Label links from 0 to n
![alt text][image3]


4. Define common normals and reference frame origins
![alt text][image4]


5. Add gripper frame: it's an extra frame, represents the point on the end effector that we actually care about. It differs from frame 6 only by a translation in z6 direction.
![alt text][image5]

#### Step 2. Derive DH parameters

![alt text][image11]

Definitions of the four DH parameters:

* **α** (twist angle): the angle between **Zi-1** and **Zi** measured about **Xi-1** in a right hand sense.
* **a** (link length): the distance from **Zi-1** to **Zi** measred along **Xi-1** where **Xi-1** is perpendicular to both.
* **d** (link offset): the signed distance from **Xi-1** to **Xi** measured along **Zi**.
* **θ** (joint angle): the angle between **Xi-1** and **Xi** measured about **Zi** in a right hand sense.

I first identified the location of each non-zero link lengths **a** and the link offsets **d**:

![alt text][image6]

I obtained the values for **a** and **d** from the URDF file at `/kuka_arm/urdf/kr210.urdf.xacro`. It wasn't a straight forward task because:

1. The joint origin in the URDF file are not consistent with the frame origins created in accordance with the DH parameter convention, nor do they have the same orientation:
![alt text][image7]
2. Each joint is defined relative to its parent.

Then I obtained th twist angles **alpha** from observing the relationship of each **Zi-1** and **Zi** pair:

Z(i), Z(i+1) | α(i) 
--- | ---
Z(0) ll Z(1) | 0
Z(1) ⟂ Z(2)| -pi/2
Z(2) ll Z(3) | 0 
Z(3) ⟂ Z(4) | -pi/2
Z(4) ⟂ Z(5) | pi/2
Z(5) ⟂ Z(6) | -pi/2
Z(6) ll Z(G) | 0


Here is the DH parameter table:

Links | α(i-1) | a(i-1) | d(i-1) | θ(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | -pi/2 | 0.35 | 0 | q2 - pi/2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  -pi/2 | -0.054 | 1.50 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | q7

#### 2. Using the DH parameter table to create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

For each link there are four individual transforms, 2 rotations and 2 translations:

![alt text][image8]

In matrix form this transform is:

![alt text][image9]

In addition to individual transforms, I determined the transfromation between `base_link` and the `end_effector` by pre-multiplying(intrinsic transformations) all individual transforms together:

![alt text][image10]

Then I determined a complete homogeneous transfrom between `base_link` and the `gripper_link` using the end-effector pose (position + rotation):

![alt text][image12]

where Px, Py, Pz represent the position of end-effector w.r.t. base_link and RT represent the rotation part. I constructed RT using the Roll-Pitch-Yaw angles of the end-effector (given by the simulator).

The URDF model does not follow the DH convention, I used additional rotations - a rotation about z-axis of 180° followed by a rotation about the y-axis of -90° - in the calculation to compensate for the difference:

![alt text][image13]


### Inverse Kinematics

Inverse kinematics is used to determine the joint angles necessary to get the end-effector in the desired position and orientation.

In the Kuka 210 Robotic arm, the last three joints are revolute joints such that the last three neighboring joint axes intersect at a single point. Such a design is called a **spherical wrist** and the common point of intersection is called the **wrist center**. 

The advantage of such a design is that it kinematically decouples the position and orientation of the end effector. Hence, it is possible to independently solve two simpler problems: first, the Cartesian coordinates of the wrist center, and then the composition of rotations to orient the end effector. The first three joints to control the position of the wrist center while the last three joints would orient the end effector as needed.

#### 1. Solution of Inverse Position

**Derivation of Wrist Center**

![Derivation of Wrist Center](images/wrist_center_derivation.jpg)

 - In the above diagram, dG is obtained from the DH parameter table (i.e. URDF specification). 
 - The wrist center is located at the center of joint 5.
 - Px, Py, Pz are the target end effector (gripper) positions.
 - Rrpy is a unit vector along the link from Wc to Gripper.
 - Rrpy is obtained from the target roll, pitch & yaw angles. 
 ```Rrpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll) * R_corr.inv()```.
 - However, R_corr inverse is same as R_corr as its symmetric. Thus 
 ```Rrpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll) * R_corr```

**Derivation of first joint angle** :

![alt text][image27]

**Theta1**: 

To find 𝜃1, we need to project the wrist center point, (WCx, WCy, WCz) onto the ground plane by setting Wcz = 0. Hence, ```theta1 = atan2(WCy, WCx)```

**Derivation of 2nd and 3rd joint angle** :

In the following diagram, the side view of the robot arm is depicted. The circled numbers represent the joints.

![alt text][image28]

**Theta2**: 

From the above diagram, at joint 2, we can see that : ```theta2 = pi/2 - A - beta```

From the triangle ABC, we can calculate angle ABC and Angle beta.


**Theta3**:

From the above diagram, at joint 3, we can see that: ```theta3 + B + alpha = pi/2 ```

The same way, we can calculate Angle B and Angle alpha.


#### 2. Solution of Inverse Orientation

![alt text][image29]

**Note** : The above formulae for theta 4,5 and 6 are in terms of the elements of R3_6, where the indices are 1-based. In python the indices will be 0-based.

Thus,
```
theta4 = mpmath.atan2(R3_6[2,2] , -1*R3_6[0,2])
theta5 = mpmath.atan2(mpmath.sqrt(R3_6[1,0]**2 + R3_6[1,1]**2) , R3_6[1,2])
theta6 = mpmath.atan2(-1 * R3_6[1,1] , R3_6[1,0])
```

### Project Implementation

I implemented Forward Kinematics in lines 60 to 91 in `IK_server.py`, and Inverse Kinematics in lines 121 to 160.

I've limited the angles to be under the limits indicated in the URDF file. 

Before doing the real implementation in `IK_server.py`, I tested my implementation with `IK_debug.py`. Here are the test results:

##### Test Case 1

![alt text][image24]

##### Test Case 2

![alt text][image25]

##### Test Case 3

![alt text][image26]


### There is a video named robotvideo.mp4 in the folder shows how the robot work.
