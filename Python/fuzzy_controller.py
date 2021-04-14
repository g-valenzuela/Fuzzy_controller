######################################################################
#                   Escrito por: Gustavo Valenzuela                  #
#                   gustavo.valenzuela.ing@gmail.com                 #
######################################################################

"""
  En este código se simula el sistema de control difuso PD+I utilizando
  una LookUp-Table. 
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import RectBivariateSpline
from modelo_planta import salida

# Parámetros de la planta
a = 1.00151e-4
b = 8.67973e-3
g = 40
Y0 = 25 # Temperatura ambiente
Ts = 25 # Tiempo de muestreo
aTs = math.exp(-a*Ts)
bTs = (b/a)*(1-math.exp(-a*Ts))

# Importar lookup table
LookUpTableData = np.loadtxt(open("LookUpTableData.csv"), delimiter=",")
E = np.linspace(-1,1,np.size(LookUpTableData,0))  # Discretización de filas
CE = np.linspace(-1,1,np.size(LookUpTableData,1)) # Discretización de columnas
interp = RectBivariateSpline(E,CE,LookUpTableData,kx=3,ky=3) # Crear clase interpolación

# Ganancias controlador
x_1 = np.array([0.065751091003418,4.499999918447061,3.017834563310598e-05,1.251905164602053e-05])
x_2 = np.array([0.065999962502552,3.884999892217342,4.994046780049530e-05,0.009999738029963])

Setpoint = np.array([65,80])  # Salida deseada (°C)
hr = 2                        # Horas
Time = 3600*hr                # Tiempo total de simulacion (s)
n = round(Time/Ts)            # Numero de muestras

# Pre-asignar todas las matrices para optimizar el tiempo de simulacion
t = np.arange(0,Time,Ts)
u = np.zeros((n,1))
y = np.zeros((n,1))
y[0] = 50
e = np.zeros((n,1))
de = np.zeros((n,1))
ie = np.zeros((n,1))
efuzz1 = np.zeros((n,1))
efuzz2 = np.zeros((n,1))
r = np.zeros((n,1))

# Cambio de setpoint
for i in range(2):
    if i == 0:
        r[0:n//2] = Setpoint[i]
    else:
        r[n//2:] = Setpoint[i]

# Bucle de control
for k in range(n-1):
    # Asignación de ganancias 
    if r[k] == Setpoint[0]:
        x = x_1
    else:
        x = x_2
    GE = x[0]
    GU = x[1]
    GIE = x[2]
    GCE = x[3]
    # Cálculos controlador
    e[k] = r[k] - y[k]             # Error actual
    if k == 0:
        de[k] = e[k]/Ts            #Cáculos para primera iteración
        ie[k] = 0
    else: 
        de[k] = (e[k] - e[k-1])/Ts # Derivada error (euler hacia atrás)
        ie[k] = e[k-1]*Ts          # Integral error (euler hacia adelante)
    I = np.sum(ie)                 # Suma integral error
    efuzz1[k] = GE*e[k]            # Multiplicar error por ganancia GE
    efuzz2[k] = GCE*de[k]          # Multiplicar derivada por ganancia GCE
    u[k] = GU*(interp.ev(efuzz1[k],efuzz2[k]) + GIE*I) # Cálculo acción de control
    if k < n:
        y[k+1] = salida(y[k],u[k],aTs,bTs,g,Y0)        # Cálculo de salida de la planta

# Graficar resultados 
plt.step(t,r,"-r",label="Setpoint",where='post')
plt.step(t,y,"-b",label="Simulación",where='post')
plt.legend(loc="lower right")
plt.grid(True)
plt.ylabel('Temperatura (°C)')
plt.xlabel('Tiempo (s)')
plt.axis([0, Time, 50, 85])
plt.show() 
