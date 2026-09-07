import numpy as np


def SmallestSignedAngleBetween(theta1, theta2):
    #### Recordad poner siempre en theta2 la referencia para evitar
    #### problemas de signo
    a = (theta1 - theta2) % (360)
    b = (theta2 - theta1) % (360)
    return -a if a < b else b


print(SmallestSignedAngleBetween(-170, 180))
