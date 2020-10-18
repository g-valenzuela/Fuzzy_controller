%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   Escrito por: Gustavo Valenzuela                  %
%                   gustavo.valenzuela.ing@gmail.com                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%{
  En este código se realiza la optimización mediante AGs para el
  controlador PID utilizando la función "ga".         
  
  Los parámetros de salida se guardan en archivos .mat para poder ser   
  cargados porsteriormente en la simulación sim_pid.m
  
  Los archivos PID_test_p(*).m corresponden a las funciones objetivo para
  cada tramo, donde (*): tramo 1 ó 2.

%}

clear all, close all, clc
PopSize = 25;
MaxGenerations = 100;

options = optimoptions(@ga,'PopulationSize',PopSize,'MaxGenerations',MaxGenerations,'UseParallel', false, 'UseVectorized', false);
[x_1,fval_1] = ga(@(PID)PID_test_p1(PID),3,-eye(3),zeros(3,1),[],[],[0 0 0],[0.30145 1 1],[],options);
%save('PID_1_J2_V2.mat','x_1','fval_1')
[x_2,fval_2] = ga(@(PID)PID_test_p2(PID),3,-eye(3),zeros(3,1),[],[],[0 0 0],[0.2385 0.001 1],[],options);
%save('PID_2_J2_V2.mat','x_2','fval_2')
