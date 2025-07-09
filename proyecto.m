% -------------------------------------------------------------------------
% Script de MATLAB para Control PID con Setpoint Manual
% -------------------------------------------------------------------------
% Este script controla la posición de un carro en un riel usando un
% sensor ultrasónico. El setpoint (distancia objetivo) se introduce
% manualmente por el teclado al inicio de la ejecución.
%
% Requisitos de hardware:
% 1. Placa Arduino (Uno, Nano, etc.)
% 2. Un sensor ultrasónico HC-SR04
% 3. Un servomotor
%
% Conexiones (puedes cambiarlas si es necesario):
% - Servo: Pin digital 9
% - Sensor 1 (Carro): Trig en D7, Echo en D6
%
% Cómo usar:
% 1. Conecta tu Arduino a la computadora.
% 2. Abre este script en MATLAB.
% 3. Ejecuta el script.
% 4. Se te pedirá en la Ventana de Comandos que introduzcas el setpoint.
% 5. Para detener el script, presiona Ctrl+C en la Ventana de Comandos.
% -------------------------------------------------------------------------

% --- Limpieza e Inicialización ---
clear; clc; close all;

disp('Inicializando conexión con Arduino...');
% Establece la conexión con la placa Arduino.
% Asegúrate de que el puerto ('COMx') sea el correcto.
a = arduino('COM4', 'Uno');

disp('Configurando pines y servo...');
% --- Definición de Pines ---
trigPin1 = 'D7';
echoPin1 = 'D6';
servoPin = 'D9';

% Configura el sensor ultrasónico para medir la posición del carro
sensor1 = ultrasonic(a, trigPin1, echoPin1);

% Configura el objeto servo
s = servo(a, servoPin);

% --- Constantes del Controlador y del Sistema ---
Umax = 30;      % Grados
Umin = -30;     % Grados
Umax_rad = 1.151; % Radianes
Umin_rad = -1.151;% Radianes
T = 0.09;       % Período de muestreo (segundos)

time_log = [];      % Vector para guardar el tiempo
distance_log = [];  % Vector para guardar las distancias medidas


% --- Ganancias del Controlador PID ---
Kp = 2.5;
Ki = 2;
Kd = 0.75;

% --- Inicialización de Variables ---
I_prec = 0;
y_prec = 0;
D_prec = 0;
Saturation = false;

% %% --- CAMBIO: INTRODUCIR SETPOINT POR TECLADO --- %%
% Pide al usuario el setpoint en centímetros y lo convierte a metros.
setpoint_cm = input('Introduce el setpoint deseado en centímetros (ej: 20): ');
setpoint = setpoint_cm / 100; % Convertir a metros
disp(['Setpoint fijado en: ', num2str(setpoint), ' metros.']);
move_servo(s, 0);
disp('servo en cero grados');
pause(5);

disp('Iniciando la fase de configuración (Setup)...');
% --- Fase de Configuración ---
writePosition(s, 0.5); % Mueve el servo a la posición central (90 grados)
pause(2); % Espera a que el servo se estabilice

% Toma la primera medición del carro para inicializar el filtro
y_prec = measure_1(sensor1);
disp(['Posición inicial del carro: ', num2str(y_prec), ' metros.']);
disp(y_prec);
%pause(0.05);

disp('Configuración completada. Iniciando bucle de control...');
disp('Presiona Ctrl+C para detener.');

% --- Bucle de Control Principal ---
figure; % Crea una figura para graficar
hold on;
grid on;
title('Control de Posición del Carro con Setpoint Manual');
xlabel('Tiempo (s)');
ylabel('Distancia (m)');
h_y = animatedline('Color', 'b', 'LineWidth', 2);
h_sp = animatedline('Color', 'r', 'LineStyle', '--');
% Dibuja una línea horizontal constante para el setpoint
yline(setpoint, 'r--', ['Setpoint = ' num2str(setpoint) ' m']);
legend('Posición del Carro (y)');
startTime = tic; % Inicia un temporizador

while true
    % --- Lectura y Filtrado del Sensor ---
    % Lee la posición del carro y aplica un filtro digital
    y = measure_1(sensor1);
    y = 0.53 * y + 0.47 * y_prec;
    
    % --- Cálculo del Error ---
    % El setpoint es el valor fijo introducido por el usuario
    error = y - setpoint;
    %error = setpoint - y;
    
    % --- Controlador PID ---
    P = Kp * error;
    
    if ~Saturation
        I = I_prec + T * Ki * error;
    else
        I = I_prec;
    end
    
    D = (Kd / T) * (y - y_prec);
    D = 0.56 * D + 0.44 * D_prec;
    
    U = P + I + D;
    
    % --- Saturación de la Acción de Control ---
    if U < Umin_rad
        U = Umin_rad;
        Saturation = true;
    elseif U > Umax_rad
        U = Umax_rad;
        Saturation = true;
    else
        Saturation = false;
    end
    
    % --- Conversión y Mapeo de la Señal de Control ---
    U_deg = U * (180 / pi);
    U_mapped = mapfun(U_deg, Umin, Umax, 24/180, 156/180);
    
    % --- Mover el Servo ---
    if U_mapped < (83/180) || U_mapped > (95/180) || abs(error) > 0.02
        move_servo(s, U_mapped);
    end
    
    % --- Actualización de Gráficas y Variables ---
    currentTime = toc(startTime);
    time_log(end+1) = currentTime;
    distance_log(end+1) = y;
    addpoints(h_y, currentTime, y);
    drawnow limitrate; % Actualiza la gráfica
    
    % Actualiza las variables para la siguiente iteración
    I_prec = I;
    y_prec = y;
    D_prec = D;
    
    pause(T);
end

% --- Funciones Auxiliares ---
function distance_m = measure_1(sensor_obj)
    % Lee la distancia del sensor 1 (carro) y la convierte a metros.
    dist_cm = readDistance(sensor_obj) * 100;
    if dist_cm > 42, dist_cm = 43; end
    if dist_cm < 0, dist_cm = 0; end
    distance_m = (dist_cm - 1.5 + 0.5) * 0.01;
end

function move_servo(servo_obj, u_mapped)
    % Mueve el servo a la posición 'u_mapped' [0, 1]
    correction_deg = mapfun(u_mapped, 30/180, 150/180, 14, 3);
    correction_norm = correction_deg / 180;
    final_pos = u_mapped - correction_norm;
    final_pos = max(0, min(1, final_pos));
    writePosition(servo_obj, final_pos);
end

function output = mapfun(x, in_min, in_max, out_min, out_max)
    % Réplica de la función map() de Arduino en MATLAB
    output = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
end
