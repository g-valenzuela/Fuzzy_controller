%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   Escrito por: Gustavo Valenzuela                  %
%                   gustavo.valenzuela.ing@gmail.com                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%{
  En este código se simula el sistema de control PID.
  Las ganancias para cada tramo obtenidas mediante AGs se cargan 
  mediante los archivos PID_(*)_J(**).mat, donde:
  
  *:  1er o 2do tramo de la señal de referencia
  **: Basado en función Objetivo J1 o J2
%}

clear all
close all
clc
tic % Iniciar temporizador para calcular el tiempo de CPU

%Parametros de la planta
a = 1.00151e-4;
b = 8.67973e-3;
g = 40;
Y0 = 25;
Ts = 25; %Tiempo de muestreo
aTs = exp(-a*Ts);
bTs = (b/a)*(1-exp(-a*Ts));

% Cargar ganancias optimización
load('PID_1_J1.mat','x_1','fval_1')
load('PID_2_J1.mat','x_2','fval_2')

Setpoint = [65 80];    % Salida deseada (°C)
hr = 2;                % Horas      
Time = hr*3600;        % Tiempo total de simulacion (s)
n = round(Time/Ts);    % Numero de muestras

% Pre-asignar todas las matrices para optimizar el tiempo de simulacion
t = (0:n-1)'*Ts;
u_pid = zeros(n,1);
y_pid = zeros(n,1);
y_pid(1) = 50;
e = zeros(n,1);
r = zeros(n,1);

% Cambio de setpoint 
for i = 1:2
    if i == 1
        r(1:n/2,1) = Setpoint(i);
    end
    if i == 2
        r(n/2+1:end,1) = Setpoint(i);
    end
   
end

% Bucle de control
for k = 1:n
    % Cambio de ganancia según rango
    if r(k,1) == Setpoint(1)
        x = x_1;
    else
        x = x_2;
    end
    Kp = x(1);
    Ki = x(2);
    Kd = x(3);
    q0 = Kp + Kd/Ts;
    q1 = -Kp + (Ki*Ts) -2*Kd/Ts;
    q2 = Kd/Ts;
    e(k) = r(k) - y_pid(k);
    if k == 1
        u_pid(k) = q0*e(k);
    end
    if k == 2
        u_pid(k) = u_pid(k-1) + q0*e(k) +q1*e(k-1);
    end
    if k > 2
        u_pid(k) = u_pid(k-1) + q0*e(k) +q1*e(k-1) + q2*e(k-2);
    end
   
    if u_pid(k) < 0
        u_pid(k) = 0;
    end
    
    if k < n
        y_pid(k+1) = modelo_planta(y_pid(k),u_pid(k),aTs,bTs,g,Y0); % Salida de la planta
    end
    
end

tsim = toc; % Tiempo de simulacion

% Graficar resultados
figure
hold all
stairs(t,r,'r','LineWidth',1)
grid on
stairs(t,y_pid,'b','LineWidth',1)
xlabel('Tiempo (s)')
ylabel('Temperatura (°C)')
legend('Setpoint','Simulacion')

% Indices de desempeño
ISE = sum(e.^2*Ts);
IAE = sum(abs(e)*Ts);
ITAE = sum(t.*abs(e)*Ts);
ITSE = sum(t.*e.^2*Ts);
ISCO = sum(u_pid.^2*Ts);
w = [1 1];
J_1 = w(1)*ITAE + w(2)*ISCO;
J_2 = w(1)*ITSE + w(2)*ISCO;