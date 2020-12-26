# Lidar SLAM

> by: SS47816

## Homework 3: Laser-scan Undistortion

### Task 1: Laser-scan Undistortion

Procedure 

The result on the rosbag is shown here:

<video src="./media/undistortion.mp4"></video>

---

### Task 2: ICP with Known Associated Points

#### Problem:

Given 2 sets of Lidar points as $\{p_i\}$ and $\{p\prime _i\}$, with $i = 1,2,\dots,N$, we have:
$$
p\prime_i = Rp_i + T + N_i \\
$$

where $R$ is a $3\times3$ rotation matrix, $T$ is a $3\times1$ vector representing the translation, and $N$ is the noise vector. 

We wish to find the $R$ and $T$ that minimise this:
$$
{\sum}^2 = \sum^N_{i=1} || p\prime_i - (Rp_i + T) || \\
$$

#### Proof:


$$
{\sum}^2 = \sum^N_{i=1} || p\prime_i - (Rp_i + T) || \\
$$


$$
{\sum}^2 = \sum^N_{i=1} || p\prime_i - (Rp_i + T) || \\
$$



---

### Task 3: Principles about Lidar



---

### Task 4: Lidar Point Undistortion using IMU

1. 


