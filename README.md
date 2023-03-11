# kalman-filter
You can predicting your future data value and making sensor fusion by using kalman filter. So kalman filter used in 
robotics, engineering, a.i. developing,marketing ... .

## Extended Kalman Filter
Normal kalman filter using in lineer project. And you can't get efficient result for nonlineer project. Extended kalman filters solved this problems.
First you need the state model for your system to using EKF(Extended Kalman Filter).
For this project our state variables are
```math
x = 
\begin{bmatrix}
x_{pos} \\
x_{vel} \\
x_{alt} 
\end{bmatrix}
```
  and equation are 

```math
\begin{align}
x_{pos}' &= x_{pos} +x_{vel}*dt + w_{pos} \\
x_{vel}' &= x_{vel} +  w_{vel} \\
x_{alt}' &= x_{alt} +  w_{alt}
\end{align}
```
w is a sensor noises.So our input matrix A is 
```math
A = 
\begin{bmatrix}
1 & 1 & 0 \\
0 & 1 & 0 \\
0 & 0 & 1
\end{bmatrix}
```
and our state math model 

```math
x' = Ax + Bu_k
y =  Cx + Q
```
Q is sensor noise matrix . In this project this is used 0.001 for every w. For now this math model is nonlineer so we need the linnerization before using kalman filter.
At this point we using Jacobian matrices and Taylor series. Using Taylor series for input matrix A
```math
A' = A*dt + \frac{(A*dt)^2}{2!} + ...
```

and using Jacobian matrix at output matrix.

So we use the kalman filter now.

Kalman filter math model is 
```math
\begin{align}
&\text{Predict}\\
x_k'&= Ax_{k-1} + Bu_k\\
P_k &= AP_{k-1}A^T + Q \\\
&\text{Update} \\
K_k &= P_kH^T(HP_kH^T+R)^{-1} \\
x_k &= x_k + K_k(z_k - Hx_k)\\
P_k &= (I - K_kH)P_k
\end{align}

```

## References
- [Havelsan Youtube Videosu](https://www.youtube.com/watch?v=P2B8h3SQz7U)
- [Understand & Code a Kalman Filter [Part 1 Design]](https://www.youtube.com/watch?v=TEKPcyBwEH8&list=PLvKAPIGzFEr8n7WRx8RptZmC1rXeTzYtA)
- [Extended Kalman Filter and Unscented Kalman Filter](https://www.youtube.com/watch?v=LkHBR7efKQw)
- [Understanding Kalman Filters](https://youtube.com/playlist?list=PLn8PRpmsu08pzi6EMiYnR-076Mh-q3tWr)
- [Kalman Filter Example](https://cocalc.com/share/public_paths/7557a5ac1c870f1ec8f01271959b16b49df9d087/11-Extended-Kalman-Filters.ipynb)
- [Detaylı Kalman Filtresi](https://nbviewer.org/github/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/table_of_contents.ipynb)

