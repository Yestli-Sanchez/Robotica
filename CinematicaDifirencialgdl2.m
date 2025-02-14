clear all
close all
clc

syms th1(t) th2(t) l1 l2

RP = [0; 0];

Q = [th1; th2];
disp('Coordenadas generalizadas');
pretty(Q);

% Vector de velocidades generalizadas
Qp = diff(Q, t);
disp('Velocidades generalizadas');
pretty(Qp);

% Número de grados de libertad del robot
GDL = size(RP, 1);

% Vector de traslación de la articulación 1 respecto a 0
P(:,:,1) = [l1*cos(th1);
            l1*sin(th1);
                     0];

% Matriz de rotación de la junta 1 respecto a 0
R(:,:,1) = [cos(th1) -sin(th1) 0;
            sin(th1)  cos(th1) 0;
            0         0        1];

% Vector de traslación de la articulación 2 respecto a 0
P(:,:,2) = [P(1,1,1) + l2*cos(th1 + th2);
            P(2,1,1) + l2*sin(th1 + th2);
                                0];

% Matriz de rotación de la junta 2 respecto a 0
R(:,:,2) = [cos(th1 + th2) -sin(th1 + th2) 0;
            sin(th1 + th2)  cos(th1 + th2) 0;
            0               0              1];

% Derivadas parciales de x respecto a th1 y th2
Jv11 = functionalDerivative(P(1,1,GDL), th1);
Jv12 = functionalDerivative(P(1,1,GDL), th2);

% Derivadas parciales de y respecto a th1 y th2
Jv21 = functionalDerivative(P(2,1,GDL), th1);
Jv22 = functionalDerivative(P(2,1,GDL), th2);

% Derivadas parciales de z respecto a th1 y th2
Jv31 = functionalDerivative(P(3,1,GDL), th1);
Jv32 = functionalDerivative(P(3,1,GDL), th2);

disp('Cinemática diferencial de la posición del robot de 2 GDL');

% Obtención de la cinemática diferencial del robot a partir de la cinemática directa
Jv_d = simplify([Jv11 Jv12;
                 Jv21 Jv22;
                 Jv31 Jv32]);
pretty(Jv_d)