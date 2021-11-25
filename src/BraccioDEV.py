import numpy as np
from copy import copy
#import rbdl
cos=np.cos; sin=np.sin; pi=np.pi

def dh(d, theta, a, alpha):
    """
    Matriz de transformacion homogenea asociada a los parametros DH.
    Retorna una matriz 4x4
    """
    sth = np.sin(theta)
    cth = np.cos(theta)
    sa  = np.sin(alpha)
    ca  = np.cos(alpha)
    T = np.array([[cth, -ca*sth,  sa*sth, a*cth],
                   [sth,  ca*cth, -sa*cth, a*sth],
                   [0.0,      sa,      ca,     d],
                   [0.0,     0.0,     0.0,   1.0]])
    return T
    
    
def fkine_BRT(q):
    """
    Calcular la cinematica directa del robot UR5 dados sus valores articulares. 
    q es un vector numpy de la forma [q1, q2, q3, q4, q5, q6]
    """
    # Longitudes (en metros)

    # Matrices DH (completar), emplear la funcion dh con los parametros DH para cada articulacion
    m=100
    l1=(7.5/m)
    l2=(12.5/m)
    l3=(12.5/m)
    l4=(6.5/m)
    l5=(13/m)
    # Matrices DH
    T1=dh(l1 ,-q[0]-pi/2 ,0,-pi/2)#listo
    T2=dh(0,q[1]+pi,l2,0)
    T3=dh(0,q[2]-pi/2,l3,0)
    T4=dh(0,q[3],0 ,pi/2) #listo
    T5=dh(l4+l5,-q[4] ,0 ,0     )#listo
    
    # Efector final con respecto a la base
    T = T1.dot(T2).dot(T3).dot(T4).dot(T5)
    return T

def jacobian_BRT(q, delta=0.0001):
    # Crear una matriz 3x5
    J = np.zeros((3,5))
    # Transformacion homogenea inicial (usando q)
    T = fkine_BRT(q)    
    # Iteracion para la derivada de cada columna
    for i in range(5):
        # Copiar la configuracion articular inicial
        dq = copy(q)
        # Incrementar la articulacion i-esima usando un delta
        dq[i]=dq[i]+delta
        # Transformacion homogenea luego del incremento (q+delta)
        Ti=fkine_BRT(dq)
        # Aproximacion del Jacobiano de posicion usando diferencias finitas
        J[:,i]= 1/delta*(Ti[0:3,3]-T[0:3,3])    
    return J

def ikine_BRT(xdes, q0):
    """
    Calcular la cinematica inversa de UR5 numericamente a partir de la configuracion articular inicial de q0. 
    """    
    epsilon  = 0.0001
    max_iter = 1000
    delta=0.0001
   
    q  = copy(q0)
    for i in range(max_iter):
            # Main loop
            J=jacobian_BRT(q,delta)           
            f=fkine_BRT(q)
            e=xdes-f[0:3,3]
            q=q+np.dot(np.linalg.pinv(J), e)
            #Condicion de termino
            if (np.linalg.norm(e)<epsilon):
                break            
            pass
    return q

def ik_gradient_BRT(xdes, q0):

    """

    Calcular la cinematica inversa de UR5 numericamente a partir de la configuracion articular inicial de q0. 

    Emplear el metodo gradiente

    """

    epsilon  = 0.001
    max_iter = 1000
    delta    = 0.00001
    alpha    = 0.5
    q  = copy(q0)

    for i in range(max_iter):

        # Main loop

        #Matriz Jacobiana

        J=jacobian_BRT(q,delta)

        #Matriz Actual

        Td=fkine_BRT(q)

        #Posicion Actual

        xact=Td[0:3,3]

        # Error entre pos deseada y pos actual

        e=xdes-xact

        # Metodo de Newton

        q=q+alpha*np.dot(J.T,e)

        #Condicion de termino

        if(np.linalg.norm(e)<epsilon):

            break

        pass
    return q
