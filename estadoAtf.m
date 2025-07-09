% Parámetros del Sistema Viga y Carrito
% --- Definición de las constantes físicas
m = 0.020;      % Masa del carrito en kg (20 gramos)
b = 0.45;       % Coeficiente de fricción viscosa en Ns/m
g = 9.81;       % Aceleración de la gravedad en m/s^2

% --- Asumir un valor para el momento de inercia de la viga
% Para una viga de laboratorio pequeña (ej. 50 cm de largo, 200g de masa), 
% un valor de ~0.02 kg*m^2 es razonable.
Iv = 0.02;      % Momento de inercia de la viga en kg*m^2

% Representación en Espacio de Estados (Matrices A, B, C, D)
% Basado en el modelo linealizado: z_punto = Az + Bu, y = Cz + Du
% Donde el vector de estado es z = [x; x_punto; alpha; alpha_punto]
% La entrada u es el torque (tau) y la salida y es la posición (x)

A = [0,   1,    0,    0;
     0, -b/m,   g,    0;
     0,   0,    0,    1;
   -m*g/Iv, 0,  0,    0];

B = [0; 0; 0; 1/Iv];

C = [1, 0, 0, 0]; % La salida es la posición del carrito 'x'

D = 0;

% Conversión de Espacio de Estados a Función de Transferencia
% Se utiliza la función de MATLAB ss2tf
% G(s) = Y(s)/U(s) = num(s)/den(s)

[num, den] = ss2tf(A, B, C, D);

% --- Muestra la función de transferencia de forma simbólica (más legible)
G = tf(num, den);
disp('La función de transferencia es:');
roots(den)
G

numS = [490.5];
denS = [1 22.5 0 0 96.24];
Gs = tf(numS, denS);
step(Gs)
roots(denS)