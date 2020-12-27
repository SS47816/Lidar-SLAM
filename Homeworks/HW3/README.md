# Lidar SLAM

> by: SS47816

## Homework 3: Laser-scan Undistortion

### Task 1: Laser-scan Undistortion

Procedure 

The result on the rosbag is shown here:

<video src="./media/undistortion.mp4"></video>

---

### Task 2: ICP with Known Associated Points

#### 1. Problem:

Given 2 sets of Lidar points as $\{p_i\}$ and $\{p\prime _i\}$, with $i = 1,2,\dots,N$, we have:
$$
p\prime_i = Rp_i + T + N_i
$$

where $R$ is a $3\times3$ rotation matrix, $T$ is a $3\times1$ vector representing the translation, and $N$ is the noise vector. 

We wish to find the $R$ and $T$ that minimise this:
$$
{\sum}^2 = \sum^N_{i=1} {|| p\prime_i - (Rp_i + T) ||}^2
$$

#### 2. Decoupling Translation and Rotation:

If the least-square solution to (1) is $\widehat R$ and $\widehat T$, then $\{p\prime)$ and $\{p\prime\prime \triangleq \widehat R = \widehat T\}$ have the same centroid, i.e.,
$$
p\prime = p\prime\prime
$$
where
$$
p \triangleq {1 \over N}\sum^N_{i=i}p_i
$$

$$
p\prime \triangleq {1 \over N}\sum^N_{i=i}p\prime_i
$$

$$
p\prime\prime \triangleq {1 \over N}\sum^N_{i=i}p\prime\prime_i = \widehat Rp + \widehat T
$$

Let
$$
q_i \triangleq p_i - p
$$

$$
q\prime_i \triangleq p\prime_i - p\prime
$$

We have
$$
{\sum}^2 = \sum^N_{i=1} {|| q\prime_i - Rq_i ||}^2
$$
Therefore, the original least-aquare problem can be reduced to two parts:

1. Find $\widehat R$ to minimise ${\sum}^2$ in (9).
2. Then, the translation is found by

$$
\widehat T = p\prime - \widehat Rp \\
$$

#### 3. SVD for Finding $\widehat R$

1. From $\{p_i\}$ and $\{p\prime _i\}$, calculate $p$, $p\prime$, and then $\{q_i\}$ and $\{q\prime _i\}$.

2. Construct a $3\times3$ matrix H:
   $$
   p\prime \triangleq {1 \over N}\sum^N_{i=i}p\prime_i
   $$
   

$$
{\sum}^2 = \sum^N_{i=1} || p\prime_i - (Rp_i + T) || \\
$$



---

### Task 3: Principles about Lidar



---

### Task 4: Lidar Point Undistortion using IMU

1. 


