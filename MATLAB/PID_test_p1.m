%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   Escrito por: Gustavo Valenzuela                  %
%                   gustavo.valenzuela.ing@gmail.com                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function J = PID_test_p1(PID)
%Parametros de la planta
a = 1.00151e-4;
b = 8.67973e-3;
g = 40;
Y0 = 25;
Ts = 25; %Tiempo de muestreo
aTs = exp(-a*Ts);
bTs = (b/a)*(1-exp(-a*Ts));

% Ganancias PID
Kp = PID(1);
Ki = PID(2);
Kd = PID(3);
q0 = Kp + Kd/Ts;
q1 = -Kp + (Ki*Ts) -2*Kd/Ts;
q2 = Kd/Ts;

Setpoint = 65;         % Salida deseada (°C)
hr = 1;                % Horas      
Time = hr*3600;        % Tiempo total de simulacion (s)
n = round(Time/Ts);    % Numero de muestra

% Pre-asignar todas las matrices para optimizar el tiempo de simulacion
t = (0:n-1)'*Ts;
u = zeros(n,1);
y = zeros(n,1);
y(1) = 50;
e = zeros(n,1);
r = Setpoint*ones(n,1);

% Bucle de control
for k = 1:n
    e(k) = r(k) - y(k);
    if k == 1
        u(k) = q0*e(k);
    end
    if k == 2
        u(k) = u(k-1) + q0*e(k) +q1*e(k-1);
    end
    if k > 2
        u(k) = u(k-1) + q0*e(k) +q1*e(k-1) + q2*e(k-2);
    end
    
    if u(k) < 0
        u(k) = 0;
    end
    
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
J = J_2