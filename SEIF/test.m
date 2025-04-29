% Analytic Jacobian for the nonlinear motion model
% omega(x) = 0.1 + x^2, v = 1, dt = 1

clc;
clear;

% Define symbolic variables
syms x y theta real
v = 1;
dt = 1;
omega = 0.1 + x^2;

% Define motion model
g1 = x + (v/omega) * (sin(theta + omega*dt) - sin(theta));
g2 = y + (v/omega) * (-cos(theta + omega*dt) + cos(theta));
g3 = theta + omega*dt;

% Stack into a vector
g = [g1; g2; g3];

% Compute Jacobian wrt state [x; y; theta]
F = simplify(jacobian(g, [x; y; theta]));

% Display result
disp('Analytic Jacobian F(x, y, theta):');
F
