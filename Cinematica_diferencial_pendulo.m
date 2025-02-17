clear all 
clc

% Declaración de variables simbólicas
syms l1 l2 t  
th1 = str2sym('th1(t)'); % variables simbólicas que son string
th2 = str2sym('th2(t)'); % variables simbólicas que son string

% Configuración del robot, 0 para junta rotacional, 1 para junta prismática
RP = [0 0];

% Creamos el vector de coordenadas generalizadas 
Q = [th1 th2];
disp('Coordenadas generalizadas')
pretty(Q);

% Número de grado de libertad del robot
GDL = size(RP, 2);

% Vector de posición de las articulaciones
P(:,:,1) = [l1*cos(th1); 
            l1*sin(th1);
            0]; % Posición de la primera articulación respecto al sistema de referencia 0

P(:,:,2) = [l2*cos(th2+th1); 
            l2*sin(th2+th1);
            0]; % Posición de la segunda articulación respecto al sistema de referencia 1

% Matriz de rotación de la junta 1 respecto a 0
R(:,:,1) = [cos(th1) -sin(th1)  0;
            sin(th1)  cos(th1)  0;
            0         0         1];

% Matriz de rotación de la junta 2 respecto a 1
R(:,:,2) = [cos(th2) -sin(th2)  0;
            sin(th2)  cos(th2)  0;
            0         0         1];

% Posición del extremo del robot (punto final)
% Aplicamos las transformaciones geométricas
P_final = P(:,:,1)  +P(:,:,2);

disp('Posición del extremo del robot')
pretty(P_final);

% Cálculo del Jacobiano usando diff
Jv = sym(zeros(3, 2)); % Inicializamos el Jacobiano

% Derivadas parciales de x respecto a th1 y th2
Jv(1, 1) = diff(P_final(1), th1); 
Jv(1, 2) = diff(P_final(1), th2); 

% Derivadas parciales de y respecto a th1 y th2
Jv(2, 1) = diff(P_final(2), th1); 
Jv(2, 2) = diff(P_final(2), th2);

% Derivadas parciales de z respecto a th1 y th2
Jv(3, 1) = diff(P_final(3), th1); 
Jv(3, 2) = diff(P_final(3), th2); 
disp('Jacobiano de la posición del extremo del robot usando diff')
pretty(Jv);