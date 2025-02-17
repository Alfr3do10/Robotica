clear all 
clc

% Declaración de variables simbólicas
syms x y z

RP=[0 0 0];

Q = [x y z];
disp('Coordenadas generalizadas');
pretty(Q);

% Número de grado de libertad del robot
% Definir la función vectorial F(x, y, z)
F = [sin(5*x^3 + 3*y - 4*y*x*z^2); 
     -10*x^5 - 4*y*x*z + 15*x*z^4; 
     cos(-x*y*z^5 - 6*x*y^5*z - 7*y*x*z^2)];
pretty(F);

% Cálculo del Jacobiano usando diff porque no es posible con
% functionalDerive debido al segundo argumento

% %Jv = sym(zeros(3, 3)); % Inicializamos el Jacobiano
% %Jv(1, 1) = diff(F(1,1), x); 
% %Jv(2,1)=diff(F(2,1),x);
% Jv(3,1)=diff(F(3,1),x);
% Jv(1, 2) = diff(F(1,1), y); 
% Jv(2,2)=diff(F(2,1),y);
% Jv(3,2)=diff(F(3,1),y);
% Jv(1, 3) = diff(F(1,1), z); 
% Jv(2,3)=diff(F(2,1),z);
% Jv(3,3)=diff(F(3,1),z);
% pretty(Jv)


%Usamos jacobian para ahorrar lineas

JacobianF = jacobian(F, [x, y, z]); % Calcula la matriz Jacobiana automáticamente

disp('Matriz Jacobiana:');
pretty(JacobianF);

% Evaluamos la matriz jacobiana en el punto (-5, -4, 1)
x_val = -5;
y_val = -4;
z_val = 1;

JacobianF_eval = subs(JacobianF, {x, y, z}, {x_val, y_val, z_val});

% Mostrar la matriz jacobiana evaluada
disp('Matriz jacobiana evaluada en (-5, -4, 1):')
pretty(JacobianF_eval);