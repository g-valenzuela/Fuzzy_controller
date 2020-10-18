%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   Escrito por: Gustavo Valenzuela                  %
%                   gustavo.valenzuela.ing@gmail.com                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function J = fuzzy_test_p2(sf)
%Parametros de la planta
a = 1.00151e-4;
b = 8.67973e-3;
g = 40;
Y0 = 25;
Ts = 25; %Tiempo de muestreo
aTs = exp(-a*Ts);
bTs = (b/a)*(1-exp(-a*Ts));

%Cargar archivo .fis
fuzzy_nl = readfis('fuzzy_nl');

% Cargar ganancias optimización
load('sf_1_J1.mat','x_1')

Setpoint = [65 80];    % Salida deseada (°C)
hr = 1;                % Horas      
Time = hr*3600;        % Tiempo total de simulacion (s)
n = round(Time/Ts);    % Numero de muestra

% Pre-asignar todas las matrices para optimizar el tiempo de simulacion
t = (0:n-1)'*Ts;
u = zeros(n,1);
y = zeros(n,1);
y(1) = 50;
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
    end
    if i == 2
        r(n/2+1:end,1) = Setpoint(i);
    end
   
end

% Bucle de control
for k = 1:n
    % Ganancias NL 
    if r(k,1) == Setpoint(1)
        x = x_1;
    else
        x = [sf(1),sf(2),sf(3),sf(4)];
    end
    GE = x(1);
    GU = x(2);
    GIE = x(3);
    GCE = x(4);
    
    e(k) = r(k) - y(k); % Error actual
    if k == 1
        de(k) = e(k)/Ts;
    else
        de(k) = (e(k) - e(k-1))/Ts;  % Derivada error (euler hacia atrás)
        ie(k) = e(k-1)*Ts;           % Integral error (euler hacia adelante)
    end
    int = sum(ie);          % Suma integral error
    efuzz1(k) = GE*e(k);   % Multiplicar error por ganancia GE
    efuzz2(k) = GCE*de(k); % Multiplicar derivada por ganancia GCE
    u(k) = GU*(evalfis(fuzzy_nl,[efuzz1(k) efuzz2(k)]) + GIE*int); % Calculo salida
    if k < n
        y(k+1) = modelo_planta(y(k),u(k),aTs,bTs,g,Y0); % Salida de la planta
    end
    
end

% Graficar resultados
figure(1)
stairs(t,y,'b')
grid on
xlabel('Tiempo (s)')
ylabel('Temperatura (°C)')

% Indices de desempeño
ISE = sum(e.^2*Ts);
IAE = sum(abs(e)*Ts);
ITAE = sum(t.*abs(e)*Ts);
ITSE = sum(t.*e.^2*Ts);
ISCO = sum(u.^2*Ts);
w = [1 1];
J_1 = w(1)*ITAE + w(2)*ISCO;
J_2 = w(1)*ITSE + w(2)*ISCO;
J = J_1