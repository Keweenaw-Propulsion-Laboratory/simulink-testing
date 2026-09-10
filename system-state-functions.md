$$
\displaylines{
\vec{u}: \text{Control vector} \\
n: \text{RPS of propeller} \\
\phi: \text{Gimbal pitch} \\
\psi: \text{Gimbal yaw} \\
\vec{x}: \text{State vector} \\
\vec{s}: \text{Vehicle position with respect to fixed Earth axis} \\
\dot{\vec{s}}: \text{Vehicle velocity with repsect to fixed Earth axis} \\
q: \text{Vehicle body quaternion} \\
\vec{\omega}: \text{Vehicle angular velocity with respect to the vehicle body axis} \\
\vec{T}_B: \text{Propeller thrust vector relative to the vehicle body axis} \\
\vec{T}_E: \text{Propeller thrust vector relative to the fixed Earth axis} \\
m_B: \text{Vehicle body mass} \\
\ddot{\vec{s}}: \text{Vehicle acceleration with repsect to fixed Earth axis} \\
g: \text{Earth's gravitational constant} \\
\mathbf{I}: \text{Body inertia tensor} \\
\mathbf{R}_x(\theta): \text{Rotation matrix about the x axis by } \theta \text{ radians} \\
\mathbf{R}_y(\theta): \text{Rotation matrix about the y axis by } \theta \text{ radians}
\\ \\

\vec{u}=\begin{bmatrix}
n\\\phi\\\psi
\end{bmatrix} \\

\vec{x}=\begin{bmatrix}
\vec{s}\\
\dot{\vec{s}}\\
q\\
\vec{\omega}
\end{bmatrix} \\ \\

|\vec{T}_B|=|\vec{T_E}|=k_pn^2 \\
\hat{\vec{T}_B}= \mathbf{R}_x(\phi)\mathbf{R}_y(\psi)\hat{k}\\
\vec{T}_B = |\vec{T}_B|\hat{\vec{T}_B}\\
\vec{T}_E=q\otimes \vec{T}_B\otimes q^{-1} \\
\ddot{\vec{s}}=\frac{1}{m_B}\vec{T}_E-g\hat{k} \\ \\

\dot{\vec{x}} = \begin{bmatrix}
\dot{\vec{s}} \\
\frac{1}{m_B}\vec{T}_E-g\hat{k} \\
\frac{1}{2}(q\otimes \vec{\omega}) \\
\mathbf{I}^{-1}(\vec{R}\times\vec{T}_B - \vec{\omega}\times\mathbf{I}\vec{\omega})
\end{bmatrix} \\ \\
}
$$$$
\displaylines{

\text{Custom Cost Function Constraints (based on data from the spec sheet, as of 4/12/26)} \\ \\


n=u_1\leq167 \quad\rightarrow\quad u_1-167\le0\\
u_1\ge0\quad\rightarrow\quad-u_1\leq0 \\ \\
\phi=u_2\le20\degree\quad\rightarrow\quad u_2-20\degree\le0 \\
u_2\ge-20\degree\quad\rightarrow\quad -u_2-20\degree\le0 \\
\text{Same applies to } \psi \text{/}u_3 \\ \\

\text{so, in terms of the MATLAB custom inequality cost function matrix:} \\
\begin{bmatrix}
u_1-167\\
-u_1\\
u_2-20\degree\\
-u_2-20\degree\\
u_3-20\degree\\
-u_3-20\degree
\end{bmatrix}
}
$$