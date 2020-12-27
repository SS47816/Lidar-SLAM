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
   H \triangleq \sum^N_{i=i}q_iq\prime^T_i
   $$
   
3. SVD of H:
   $$
   H = U\Lambda V^T
   $$

4. Construct X:
   $$
   X = VU^T
   $$
   
5. Determinant of X:
   $$
   \widehat R = \left\{%
   \begin{array}{lc@{\kern2pt}c@{\kern2pt}r}
   X & det(X) & = & +1 \\
   \textrm{fail} & det(X) & = & -1
   \end{array}\right.
   $$
   
6. Expanding equation (9):
   $$
   \begin{align*}
   {\sum}^2 &= \sum^N_{i=1}(q\prime_i - Rq_i)^T(q\prime_i - Rq_i) \\
   &= \sum^N_{i=1}(q\prime_i^T q\prime_i + q_i^T R^T Rq_i - q\prime_i^T Rq_i - q_i^T R^T q\prime_i) \\
   &= \sum^N_{i=1}(q\prime_i^T q\prime_i + q_i^T q_i - 2q\prime_i^T Rq_i)
   \end{align*}
   $$

7. Therefore, minimising $\sum^2$ is equivalent to maximising:
   $$
   \begin{align*}
   F &= \sum^N_{i=1} q\prime_i^T Rq_i \\
   &= Trace\Big(\sum^N_{i=1}Rq_iq\prime_i^T \Big) \\
   &= Trace(RH)
   \end{align*}
   $$
   

8. _Lemma:_ for any positive definite matrix $AA^T$ , and any orthonormal matrix $B$:
   $$
   Trace(AA^T) \geq Trace(BAA^T)
   $$
   _Proof of Lemma:_ Let $a_i$ be the $i$th column of matrix A, then:
   $$
   \begin{align*}
   Trace(BAA^T) &= Trace(A^TBA) \\
   &= \sum_i a^T_i(Ba_i)
   \end{align*}
   $$
   By Suchwarz inequality, 
   $$
   a^T_i(Ba_i) \le \sqrt{(a^T_ia_i)(a^T_iB^TBa_i)} = a^T_ia_i
   $$
   Hence,
   $$
   Trace(AA^T) \geq Trace(BAA^T)
   $$

9. From (12) and (13), we have:
   $$
   \begin{align*}
   XH &= VU^TU\Lambda V^T \\
   &= V\Lambda V^T
   \end{align*}
   $$
   which is a symmetrical and positive definate matrix.

10. Therefore, for any $3\times3$ orthonomal matrix $B$,
    $$
    Trace(XH^T) \geq Trace(BXH^T)
    $$
    Thus, among all $3\times3$ orthonomal matrices, $X$ maximise $F$. When $det(X) = +1$, $X$ is a rotation matrix. 

11. Hence, the solution is:
    $$
    \begin{align*}
    R &= X = VU^T \\
    T &= p - Rp\prime
    \end{align*}
    $$
    

---

### Task 3: Principles about Lidar

1. Lidars measures distance by measureing the ToF (Time of Flight) of each laser beam it emitted. 

   Distance = The travel time (measured through the phase difference of the emitted and the recieved beam) of beam $\times$ the speed of light $/$ 2.

2. The image represents the 4 typical patterns on the distribution of measured range from the recieved laser beams. 

   <span style='color:MediumSpringGreen'>Green Line: </span> Exponential distribution, more beams tend to be reflected by objects that are closer to the Lidar

   <span style='color:DeepSkyBlue'> Blue Line: </span> Gaussian distribution, the beams reflected by various objects naturally distributed in the scene

   <span style='color:Crimson'>Red Line: </span> Uniform distribution, the beams reflected by various objects that ranging uniformly in the scene

   <span style='color:pink'>Red Line: </span> Uniform distribution, the beams reached/exceeded the max range

   

---

### Task 4: Lidar Point Undistortion using IMU

1. By using IMU alone, the accuarcy and reliability of the IMU will become the primary source affecting the performance of the undistrotion, especially when in a magnetically noisy environment. And the accuarcy will suffer from the inaccuracy in IMU drift.
2. When applying undistrotion on pointcloud with only an IMU, the high frequency IMU reading could be used to estimate the linear and angular velocity (which tends to be more accuarte than the odom reading), and uses these estimated motion parameters to initialise a VICP algorithm, to iteratively optimise the motion estimation and the pointcloud undistorion. 


