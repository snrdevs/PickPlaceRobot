## Project: Kinematics Pick & Place
### First Project for Udacity Robotics Software Engineering ND 2019

---

**Brief Information**

![](images/intro_pic.png)

This project uses KUKA KR210 - 6 DOF serial manipulator to pick up an object from shelves, and drop it to a bin standing next to the manipulator. 

## 1) Setting Up The Environment

To set up the environment succesfully, please have a look at this guide. 

https://github.com/udacity/RoboND-Kinematics-Project/blob/master/README.md

## 2) Kinematics Problem

### 2.1. Setup DH Parameter Table 

Step 1 is the completing DH Parameter table while solving inverse kinematics problem. 

 ![kuka_DH_diagram](images/kuka_DH_diagram.png)

To calculate DH table, upper image was taken as reference, because all joints' and links' axes and the distances between joints can be seen clearly. 

Additionally while creating this table, URDF robot KUKA KR210 is taken into consideration. 

The URDF file is located at "*kuka_arm/urdf/kr210.urdf.xacro*" ***xacro*** stands for XML Macros. More information can be found at:  *http://wiki.ros.org/xacro*

From URDF file, all joint's coordinates & axes are taken: 

|   Joint   |   x   |  y   |   z    |
| :-------: | :---: | :--: | :----: |
| base_link |   0   |  0   |   0    |
|  joint_1  |   0   |  0   |  0.33  |
|  joint_2  | 0.35  |  0   |  0.42  |
|  joint_3  |   0   |  0   |  1.25  |
|  joint_4  | 0.96  |  0   | -0.054 |
|  joint_5  | 0.54  |  0   |   0    |
|  joint_6  | 0.193 |  0   |   0    |

From upper image & table our DH Table becomes: 

```python
    # Define DH Transformation Matrix
	DH_Table = {alpha0: 0, 	a0: 0, 		    d1: 0.75, 	q1: q1,
		alpha1: -pi/2.,     a1: 0.35,	    d2: 0, 		q2: -pi/2. + q2,
		alpha2: 0, 	        a2: 1.25, 	    d3: 0, 		q3: q3,
		alpha3: -pi/2.,     a3: -0.054, 	d4: 1.5, 	q4: q4,
		alpha4: pi/2, 	    a4: 0, 		    d5: 0, 		q5: q5,
		alpha5: -pi/2.,     a5: 0, 		    d6: 0, 		q6: q6,
		alpha6: 0, 	        a6: 0, 		    d7: 0.303, 	q7: 0}
```

*Note: q -> Θ in the code*  

## 2.2. Joint Based Transformation Matrices

The transformation matrix can be calculated by substituting the DH parameters from the table above into this matrix by applying mathematical expressions: 



![joint_based_tf_m](images/joint_based_tf_m.png)



```python
        # Define transformation matrix with respect to DH Parameters
    	def TF_Matrix(alpha, a, d, q):
            TF = Matrix([
			[cos(q), 		-sin(q), 		0, 		a],
	     	[sin(q)*cos(alpha), 	cos(q)*cos(alpha), 	-sin(alpha), 	-sin(alpha)*d],
	     	[sin(q)* sin(alpha), 	cos(q)*sin(alpha), 	cos(alpha), 	cos(alpha)*d],
	     	[0,			0,			0,		1]
		   ])
            return TF
```

A function is used for ease of calculation.

```python
        T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)
        T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)
        T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)
        T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(DH_Table)
        T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(DH_Table)
        T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(DH_Table)
        T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)
```

Then, transformation matrix from base_link to end_effector is calculated: 

```python
# Transformation matrix from the base_link to end_effector 
    	T0_EE = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE)
```

## 2.3. Inverse Kinematics

Wrist center calculated with the formula below:

​	![1558091719577](images/1558091719577.png)

But to calculate WC, firstly rotation of end effector matrix depending on roll, pitch and yaw must be created:

```python
# Smybolize roll, pitch and yaw as r,p,y
      r, p , y = symbols('r p y')
      # Create rotation matrix with respect to roll
      ROT_x = Matrix([[1, 0 , 0],
					[0, cos(r), -sin(r)],
      				[0, sin(r), cos(r)]])
      
      # Create rotation matrix with respect to pitch
      ROT_y = Matrix([[cos(p),   	0 ,    sin(p)],
      				[0,       		1,     	0],
      				[-sin(p),  		0,     cos(p)]]) 
     
     # Create rotation matrix with respect to yaw
      ROT_z = Matrix([[cos(y), -sin(y), 0],
      				[sin(y), cos(y), 0],
      				[0, 0, 1]]) 
             
     # Calculate End Effector's Rotation Matrix
        ROT_EE = simplify(ROT_z * ROT_y * ROT_x)
     # Correction for KUKA KR210 
     # This correction is made for difference between gripper, 
        Rot_Error = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))
        ROT_EE = simplify(ROT_EE * Rot_Error)
```

*`Rot_Error`* comes from the difference of frames between the gripper reference frame as defined in URDF vs. the DH parameters. To overcome this issue, it's suggested about to rotate Z axis then X axis. 

![images/diff_urdf_dh.png](images/diff_urdf_dh.png)

Since we have `ROT_EE` we can calculate, Wrist Center: 

```python
ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})
# End effector position matrix 
EE = Matrix([[px], [py], [pz]])
# Wrist center calculation
WC = EE - (0.303) * ROT_EE[:,2]
```

 Next step after WC calculation is to find joint angles (theta1,2,3): 

*Calculating theta1:*

```python
theta1 = atan2(WC[1], WC[0])
```

![images/theta1.png](images/theta1.png)

> Image: Udacity 

*Calculating theta2:*

![images/theta2.png](images/theta2.png)

> Image: Udacity 

![images/theta3.png](images/theta3.png)

> Image: Udacity 

All the remaining code for inverse kinematics solution & algorithm can be seen below:

```python
# Calculating joint angles using geometric IK method 
side_a = 1.501 #from link dimension
side_b = sqrt(pow(sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35, 2)+ pow((WC[2] - 0.75), 2))
side_c = 1.25 # from link dimension

angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c ) / (2 * side_a * side_b))

theta1 = atan2(WC[1], WC[0])
theta2 = pi / 2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)  # (CHANGED!!!)
theta3 = pi / 2 - (angle_b + 0.036)  # 0.036 accounts for sag in link4 of -0.054m

# Once we have theta1,2,3 we can build the rotation matrix 
R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
R0_3 = R0_3.evalf(subs={q1: theta1, q2:theta2, q3: theta3})
R3_6 = R0_3.transpose()* ROT_EE

# Calculate Euler angles from rotation matrices 
theta4 = atan2(R3_6[2,2], -R3_6[0,2])
theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
theta6 = atan2(-R3_6[1,1], R3_6[1,0])
```

## 3. Successful Jobs done by KUKA KR210

In this section there will be several images of successful jobs: 

These images can be found in `images` folder

![images/s1.jpg](images/s1.jpg)

![images/s2.jpg](images/s2.jpg)

