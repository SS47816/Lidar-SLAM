# Lidar SLAM

> by: SS47816

## Homework 2: Odometry Calibration

### Task 1: Linear Method

After implementing the code, the results are shown here: 

![linear-calib](media/linear-calib.png)

<span style='color:DeepSkyBlue'> Blue Trajectory: </span> Recorded by Odom

<span style='color:Crimson'> Red Trajectory: </span> Recorded by Lidar

<span style='color:MediumSpringGreen'> Green Trajectory: </span> After Calibration



---

### Task 2: Model Based Method

The result I got was: 

![model-based-calib](media/model-based-calib.png)



---

### Task 3: Solving Ax = b

Generally there are three types of methods for solving for the least square solution of a overdetermined system Ax = b. 

Depending on the trade-off between **speed** vs. **accuracy**, different methods can be selected for different purposes:

#### SVD Decomposition

* Most accurate
* Longer time

#### QR Decomposition

* Average accuracy
* Average speed

#### Normal Equations

* Least accurate
* Very fast

There are some methods that can only be use in some special cases such as the LLT, which requires the matrix A to be **positive definite**. 

The full list of supported solvers in Eigen is listed [here](http://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html):

![eigen-solver](media/eigen-solver.png)



---

### Task 4: Designing Calibration Method for Odometry & Lidar

#### Description of Method

1. Retrive the odometry readings from both wheel encoders (**Predicted Value**)
2. Retrive the Lidar odometry from a selected Lidar-Odom method (such as LOAM)
3. The Lidar odometry will be passed to a **Kalman Filter** (e.g. an EKF), with it's standard deviation as the covariance matrix to produce a filtered Lidar odometry data as the ground truth (**Observed Value**)
4. The calibration node subscribes to both topics and record the data for a certain period of time (e.g. 20s), or a certain distance (e.g. 10m).
5. Once it has collected enough amount of data, the calibration process will be triggered to solve for the correction matrix using the **Model-based** method (differential drive, two wheels only). 

#### Assumptions

1. The Lidar is mounted at the centre of the robot platform
2. There's no slipping between wheels (tires) and the ground
3. The ground is flat
4. The Lidar plane is always horizontal to the ground plane (map plane)
5. There's no deformation on the wheels (or the deformation is always evenly spread over the wheel circumference)
6. The Lidar Odometry is accurate

#### Construction of Ax = b

1. The wheel encoder odometry data and the will be use to contruct matrix A
   $$
   \begin{bmatrix} 
   \omega_{L0} \triangle T_0 \\
   \omega_{L1} \triangle T_1 \\
   
   \end{bmatrix}
   $$
   

2. The Lidar Odom data after the Kalman Filter (in Step 3) will be used as the ground truth 

3. 