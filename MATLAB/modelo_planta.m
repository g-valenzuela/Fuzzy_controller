%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   Escrito por: Gustavo Valenzuela                  %
%                   gustavo.valenzuela.ing@gmail.com                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function y_k1 = modelo_planta(y_k,u_k,aTs,bTs,g,Y0)

% Modelo no lineal
y_k1 = aTs*y_k + (bTs/(1+exp(0.5*y_k - g)))*u_k + (1 - aTs)*Y0;