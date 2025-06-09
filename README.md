# Fluid Simulation 
Fluids prove to be incredibly difficult to simulate accurately due to the complexity of the equations involved and overall computational challenges [change this]. This project [sees] to simulate the motion of particles dictated by the incompressible Navier Stokes' Equations. 

$$ \rho\frac{D\textbf{v}}{Dt}=-\nabla p+\mu\nabla^{2}\textbf{v}+\textbf{F}_\mathrm{ext} $$

This project initally started out as a simple particle simualtion with basic elastic collision logic, this document details the progression to a workable fluid simulation. 

# Extras
## Color Mapping
One minor portion of the project that I wanted to implement was some basic color mapping based on the speed of the particle or any other scalar parameter.
### Vector Mapping Algorithm
The inital idea for mapping the color was to take a weighted sum of $n$ number of basis colors and take the resultant sum to be the new color. This amounts to a linear combination of $n$ vectors with coefficients to be determined. 

$$ \textbf{C}=\sum_{i=1}^{n}v_i\textbf{c}_i=v_1\textbf{c}_1+v_2\textbf{c}_2+\dots+v_n\textbf{c}_n $$

And since each vector has three components, this just amounts to matrix multiplication between an $n$ dimensional vector and a $3\times n$ matrix.

$$ \textbf{C}=A\textbf{v}=\begin{bmatrix}
c_{1,R} & c_{2,R} & \cdots & c_{n,R} \\
c_{1,G} & c_{2,G} & \cdots & c_{n,B} \\
c_{1,B} & c_{2,B} & \cdots & c_{n,G}
\end{bmatrix} \begin{bmatrix}
v_{1} \\
v_{2} \\
\vdots \\
v_{n}
\end{bmatrix}
$$

I chose 7 colors, and for now we can write up the algorithm to map each color.
```C++
Color vectorMapper(float t) {
    // Defining our basis Colors in a matrix
    float col_mat[7][3] = {{0.0f, 0.4f, 0.9f}, {0.0f, 0.75f, 0.9f}, {0.0f, 1.0f, 0.8f}, {0.5f, 1.0f, 0.4f},
                           {0.9f, 1.0f, 0.4f}, {1.0f, 0.9f, 0.4f},  {1.0f, 0.4f, 0.4f}};
    std::vector<float> w_vec = weightFunction(t);
    float col_vec[3] = {0.0, 0.0, 0.0};

    // Performing the matrix transformation
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 7; j++) {
            col_vec[i] += col_mat[j][i] * w_vec[j];
        }
    }

    return {col_vec[0], col_vec[1], col_vec[2]};
}
```
The main difficultly arises in determining the weights in $\textbf{v}$ as a function of $t\in[0,1]$. To do this we...
```C++
// write up the code here
```
Now we need to map the particle's speed to $t\in[0,1]$.
```C++
Color mapColors(Vec2 obj_velocity, float max_speed, Color (*Mapper)(float)) {
    float t;
    float obj_speed = DotProduct(obj_velocity, obj_velocity);
    t = (obj_speed > 4 * max_speed * max_speed) ? 1.0f : obj_speed / (max_speed * max_speed);
    return Mapper(t);
}
```
The way I ended up doing so is defining a maximum speed, if the particle's speed was considerablely larger than the maximum speed then we automatically map $t=1.0$, otherwise we take $t$ to be the ratio between the speed of the particle to the maximum speed. 
### Cosine Mapping Algorithm
While the above method works as intended, an alternative——arguably more flexible method——is to generate the color gradient based on a sinusoidal function. This method was largely inspired by [inigo quilez](https://iquilezles.org/articles/palettes/)'s procedural color palette. 

$$ R(t) = R_0 +  A_R\cos(2\pi(f_R t+\phi_R)) $$
$$ G(t) = G_0 +  A_G\cos(2\pi(f_G t+\phi_G)) $$
$$ B(t) = B_0 +  A_B\cos(2\pi(f_B t+\phi_B)) $$

The functions I ended up defining seperate functions and used a [graph](https://www.desmos.com/calculator/gzxhbd6dwt) to help visualize the gradient.

$$ R(t)=\frac{1}{2}(1+A_{R}\cos(v_{R}\pi t-\phi_{R}) $$
$$ G(t)=\frac{1}{2}(1+A_{G}\cos(v_{G}\pi t-\phi_{G}) $$
$$ B(t)=\frac{1}{2}(1+A_{B}\cos(v_{B}\pi t-\phi_{B}) $$

From here writing up the code for the color mapping was largely simple. 

```C++
Color mapCosine(float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    float phase_R = M_PI;
    float phase_G = M_PI_2;
    float phase_B = M_PI_4;

    float ang_R = 1.0f;
    float ang_G = -1.5f;
    float ang_B = 1.0f;

    float amp_R = 1.0f;
    float amp_G = 1.0f;
    float amp_B = 1.0f;

    return {
        0.5f * (1.0f + amp_R * std::cosf(ang_R * M_PI * t - phase_R)),
        0.5f * (1.0f + amp_G * std::cosf(ang_G * M_PI * t - phase_G)),
        0.5f * (1.0f + amp_B * std::cosf(ang_B * M_PI * t - phase_B)),
    };
}
```
