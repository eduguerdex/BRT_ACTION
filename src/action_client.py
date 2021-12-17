#!/usr/bin/env python
#Importar paquetes necesarios
import rospy
import actionlib
#Importar elementos para uso de servidor y cliente
from BRT_ACTION.msg import BRTAction, BRTGoal, BRTResult
#Importar tipo de dato usado para mensajes
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
#Importar algoritmos usados para analisis cinematico
from BraccioDEV import *
import numpy as np
#Importar marcadores AR TAG
from ar_track_alvar_msgs.msg import AlvarMarkers
import time

#Definir variables
posx=[]
posy=[]
posz=[]
posx1=[]
posy1=[]
posz1=[]
#Variar dependiendo la posicion de la camara respecto al robot y Medida de elemento a manipular
camx=-0.02
camy=0.525
camz=0.3
caja=0.05
#Iniciacion de variables
k=0
N=[]
NL=[]
xd=[0,0,0]

#reconocimiento de marcadores
def callback(markers):
        rate= rospy.Rate(1)
	#Declaracion de variables globales
        global posx,camx
        global posy,camy,k
        global posz,camz,caja,xd,NL
        #Reconocimiento y guardado de posiciones en las que se encuentra AR TAG respecto a la camara
        for m in markers.markers:
                    N.append(m.id)
             	    #Identificar TAG
		    NS=set(N)
		    NL=list(NS)
                    marker_pose = m.pose.pose
                    pos = marker_pose.position
                    ori = marker_pose.orientation
		    #Identificar y guardar posicion en una lista
                    posx1.append(pos.x)
	            posx2=set(posx1)
		    posx=list(posx2)
                    posy1.append(pos.y)
                    posy2=set(posy1)
		    posy=list(posy2)
                    posz1.append(pos.z)
	            posz2=set(posz1)
		    posz=list(posz2)
#Main principal
if __name__=="__main__":
    #Inicializar cliente
    rospy.init_node('BRT_action_client')
    #Suscripcion a topico para reconocer AR TAG
    sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback)
    time.sleep(3)
    print("Cliente activado") 
    #La cantidad de posiciones guardadas indican a cantidad de TAG detectados
    tag=len(NL)
    print("TAGs Detectados:")
    print(tag)
    i=0
    xr=[0,0,0]
    #Repeticion de solicitud de lugar detectado dependiendo de cantidad de tag
    for i in range(tag): 
	sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback)
    	time.sleep(3)
	#Inicializacion de pedido y conexion del cliente con server
        client = actionlib.SimpleActionClient('BRT_ACTION', BRTAction)
        client.wait_for_server()
	print("ID Marcador")
	print NL[i]
	#Calculo dimensional para adquirir posiciones de los TAG desde el robot.
        fx=camx-posx[i]
	fz=camz-posy[i]+caja
	fy=-(camy-posz[i])

	fx=np.round(fx,3)
	fy=np.round(fy,3)
	fz=np.round(fz,3)
	
	#Limitacion de altura
	if fz<0.10:
	       fz=0.1025
	if fz>0.13
	       fz=0.1025

        xd=[fy,fx,fz]
        xm=[posx,posy,posz]
        xcam=[-camy,camx,camz+caja]
	
        if np.array_equal(xd,xcam) == False:
	       print("AR Detectado")
        else: 
               print("Esperando lectura")
        #Variable de solicitud de datos
        goal = BRTGoal()
        #Si el resultado de posicion deseada es distinto que la posicion 0
        if np.array_equal(xd,xr) == False:
		    time.sleep(5)
		    #Se solicita posicion xd
                    goal.xd=xd
                    print("POSICION SOLICITADA:")
                    print(goal.xd)
		    #Envio de solicitud y espera de resultado
                    client.send_goal(goal)
                    client.wait_for_result()
                    #resultados despues de analisis y movimiento
                    MOV=client.get_result().MOVE
                    Q=client.get_result().q
                    MOV=np.round(MOV,3)
                    Q=np.round(Q,3)
                    print("POSICION FINAL:")
                    print(MOV)
                    print("ANGULOS DESPLAZADOS:")
                    print(Q)
