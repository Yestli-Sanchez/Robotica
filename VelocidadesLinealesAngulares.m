clear all
close all
clc

% Declaración de variables simbólicas
syms th1(t) th2(t) l1 l2 t

% Configuración del robot (0: rotacional, 1: prismático)
RP = [0 0];

% Vector de coordenadas articulares
Q = [th1; th2];
disp('Coordenadas articulares');
pretty(Q);

% Vector de velocidades articulares
Qp = diff(Q, t);
disp('Velocidades articulares');
pretty(Qp);

% Número de grados de libertad del robot
GDL = size(RP, 2);

% Posiciones de las juntas
P(:,:,1) = [l1*cos(th1); l1*sin(th1); 0];
P(:,:,2) = P(:,:,1) + [l2*cos(th1 + th2); l2*sin(th1 + th2); 0];

% Matrices de rotación
R(:,:,1) = [cos(th1) -sin(th1) 0;
            sin(th1)  cos(th1) 0;
            0         0        1];
R(:,:,2) = R(:,:,1) * [cos(th2) -sin(th2) 0;
                        sin(th2)  cos(th2) 0;
                        0         0        1];

% Vector de ceros
Vector_Zeros = zeros(1, 3);

% Inicialización de matrices de transformación homogénea
for i = 1:GDL
    A(:,:,i) = simplify([R(:,:,i) P(:,:,i); Vector_Zeros 1]);
    try
        T(:,:,i) = T(:,:,i-1) * A(:,:,i);
    catch
        T(:,:,i) = A(:,:,i);
    end
    PO(:,:,i) = T(1:3,4,i);
    RO(:,:,i) = T(1:3,1:3,i);
end

% Cálculo del Jacobiano
Jv_a = sym(zeros(3, GDL));
Jw_a = sym(zeros(3, GDL));

for k = 1:GDL
    if RP(k) == 0
        try
            Jv_a(:,k) = cross(RO(:,3,k-1), PO(:,:,GDL) - PO(:,:,k-1));
            Jw_a(:,k) = RO(:,3,k-1);
        catch
            Jv_a(:,k) = cross([0;0;1], PO(:,:,GDL));
            Jw_a(:,k) = [0;0;1];
        end
    else
        try
            Jv_a(:,k) = RO(:,3,k-1);
        catch
            Jv_a(:,k) = [0;0;1];
        end
        Jw_a(:,k) = [0;0;0];
    end
end

% Cálculo de velocidades
V = simplify(Jv_a * Qp);
W = simplify(Jw_a * Qp);

disp('Velocidad lineal obtenida mediante el Jacobiano:');
pretty(V);
disp('Velocidad angular obtenida mediante el Jacobiano:');
pretty(W);