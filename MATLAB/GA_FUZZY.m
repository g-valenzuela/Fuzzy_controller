%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   Escrito por: Gustavo Valenzuela                  %
%                   gustavo.valenzuela.ing@gmail.com                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%{
  En este código se realiza la optimización mediante AGs para el
  controlador difuso utilizando la función "ga".    
  
  Los parámetros de salida se guardan en archivos .mat para poder ser   
  cargados porsteriormente en la simulación fuzzy_lookup_table.m
  
  Los archivos fuzzy_test_p(*).m corresponden a las funciones objetivo para
  cada tramo, donde (*): tramo 1 ó 2.

%}

clear all, close all, clc
PopSize = 25;
MaxGenerations = 100;
options = optimoptions(@ga,'PopulationSize',PopSize,'MaxGenerations',MaxGenerations,'UseParallel', false, 'UseVectorized', false);
[x_1,fval_1] = ga(@(sf)fuzzy_test_p1(sf),4,[],[],[],[],[0 0 0 0],[0.065 4.497 0.0001 0.04],[],options);
%save('sf_1_J1.mat','x_1','fval_1')
[x_2,fval_2] = ga(@(sf)fuzzy_test_p2(sf),4,[],[],[],[],[0 0 0 0],[0.065 3.436 0.00008 0.04],[],options);
%save('sf_2_J1.mat','x_2','fval_2')
