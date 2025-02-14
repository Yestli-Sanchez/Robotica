clear all
close all
clc

syms x y z

% Definimos las funciones dadas
F1 = sin(5*x^3 + 3*y - 4*y*x*z^2);
F2 = -10*x^5 - 4*y*x*z + 15*x*z^4;
F3 = cos(-x*y*z^5 - 6*x*y^5*z - 7*y*x*z^2);

% Definimos el vector de funciones
F = [F1; F2; F3];

% Calculamos la matriz Jacobiana
J = jacobian(F, [x, y, z]);

% Evaluamos en el punto (-5, -4, 1)
J_eval = subs(J, [x, y, z], [-5, -4, 1]);

disp('Matriz Jacobiana evaluada en (-5, -4, 1):');
pretty(J_eval)