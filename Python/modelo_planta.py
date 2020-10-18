######################################################################
#                   Escrito por: Gustavo Valenzuela                  #
#                   gustavo.valenzuela.ing@gmail.com                 #
######################################################################

import math
def salida(y_k,u_k,aTs,bTs,g,Y0):
    y_k1 = aTs*y_k + (bTs/(1+math.exp(0.5*y_k - g)))*u_k + (1 - aTs)*Y0
    return y_k1


