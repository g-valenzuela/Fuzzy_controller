%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   Escrito por: Gustavo Valenzuela                  %
%                   gustavo.valenzuela.ing@gmail.com                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%{
  En este código se simula el sistema de control difuso PD+I,
  es necesario cargar el archivo .fis correspondiente:
  fuzzy_l.fis: superficie lineal 
  fuzzy_nl.fis: superficie no-lineal.
  
  Las ganancias para cada tramo obtenidas mediante AGs se cargan 
  mediante los archivos sf_(*)_J(**).mat, donde:
  
  *:  1er o 2do tramo de la señal de referencia
  **: Basado en función Objetivo J1 o J2
%}

clear all
close all
clc
tic % Iniciar temporizador para calcular el tiempo de CPU

% Parametros de la planta
a = 1.00151e-4;
b = 8.67973e-3;
g = 40;
Y0 = 25;
Ts = 25; % Tiempo de muestreo
aTs = exp(-a*Ts);
bTs = (b/a)*(1-exp(-a*Ts));

% Cargar archivo .fis
fuzzy_nl = readfis('fuzzy_nl');

% Construcción de Lookup-Table
Step = 0.1;
E = -1:Step:1;
CE = -1:Step:1;
N = length(E);
LookUpTableData = zeros(N);
for i=1:N
   for j=1:N
      % Evaluación de la salida u para cada combinación de E y CE
      LookUpTableData(i,j) = evalfis(fuzzy_nl,[E(i) CE(j)]);
   end
end
% Guardar Lookup table como archivo .csv
%writematrix(LookUpTableData,'LookUpTableData.csv')

% Cargar ganancias optimización
load('sf_1_J1.mat','x_1','fval_1')
load('sf_2_J1.mat','x_2','fval_2')

Setpoint = [65 80];    % Salida deseada (°C)
hr = 2;                % Horas      
Time = hr*3600;        % Tiempo total de simulacion (s)
n = round(Time/Ts);    % Numero de muestras

% Pre-asignar todas las matrices para optimizar el tiempo de simulacion
t = (0:n-1)'*Ts;
u_fuzz = zeros(n,1);
y_fuzz = zeros(n,1);
y_fuzz(1) = 50;
e = zeros(n,1);
de = zeros(n,1);
efuzz1 = zeros(n,1);
efuzz2 = zeros(n,1);
ie = zeros(n,1);
r = zeros(n,1);

% Cambio de setpoint 
for i = 1:2
    if i == 1
        r(1:n/2,1) = Setpoint(i);
    else
        r(n/2+1:end,1) = Setpoint(i);
    end
   
end

% Bucle de control
for k = 1:n
    % Asignación de ganancias  
    if r(k,1) == Setpoint(1)
        x = x_1;
    else
        x = x_2;
    end
    GE = x(1);
    GU = x(2);
    GIE = x(3);
    GCE = x(4);
    
    % Cálculos controlador
    e(k) = r(k) - y_fuzz(k); % Error actual
    if k == 1
        de(k) = e(k)/Ts;     % Derivada error primera iteración
    else
        de(k) = (e(k) - e(k-1))/Ts; % Derivada error (euler hacia atrás)
        ie(k) = e(k-1)*Ts;          % Integral error (euler hacia adelante)
   
    end
    int = sum(ie);             % Suma integral error
    efuzz1(k) = GE*e(k);      % Multiplicar error por ganancia GE
    efuzz2(k) = GCE*de(k);    % Multiplicar derivada por ganancia GCE
    %u_fuzz(k) = GU2*(evalfis(fuzzy_nl,[efuzz1(k) efuzz2(k)]) + GIE2*int); % Calculo salida sin LUT
    u_fuzz(k) = GU*(interp2(E,CE,LookUpTableData,efuzz1(k),efuzz2(k),'spline') + GIE*int); %Calculo salida con LUT
    if k < n
        y_fuzz(k+1) = modelo_planta(y_fuzz(k),u_fuzz(k),aTs,bTs,g,Y0); % Salida de la planta
    end
    
end
tsim = toc; % Tiempo de simulacion

% Graficar resultados
figure
hold all
grid on
stairs(t,r,'r')
stairs(t,y_fuzz,'b')
xlabel('Tiempo (s)')
ylabel('Temperatura (°C)')
legend('Setpoint','Simulacion')

% Indices de desempeño
ISE = sum(e.^2*Ts);
IAE = sum(abs(e)*Ts);
ITAE = sum(t.*abs(e)*Ts);
ITSE = sum(t.*e.^2*Ts);
ISCO = sum(u_fuzz.^2*Ts);
w = [1 1];
J_1 = w(1)*ITAE + w(2)*ISCO;
J_2 = w(1)*ITSE + w(2)*ISCO;
