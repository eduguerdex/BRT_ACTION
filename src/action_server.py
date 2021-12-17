#!/usr/bin/env python
#Importar paquetes necesarios
import rospy
import time
#Importar elementos para uso de servidor y cliente
import actionlib
from BRT_ACTION.msg import BRTAction, BRTGoal, BRTResult
#Importar tipo de dato usado para mensajes
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
#Importar algoritmos usados para analisis cinematico
from BraccioDEV import *
import numpy as np

def gradiente(goal):
  #Definir Variables resultado y goal
  result = BRTResult()
  xd=goal.xd
  #Analisis de cinematica inversa con posicion solicitada.
  q=ik_gradient_BRT(xd, q0)
  print('Cin inv. finalizada')
  #Analisis de cinematica directa.
  Td = fkine_BRT(q)
  pos = Td[0:3,3]
  rmax=0.39
  ractual=np.linalg.norm(pos)
  print("Posicion final y orientacion del efector final")
  print(np.round(pos,3))

  if ractual>rmax:
        print("ractual")
        print(np.round(ractual,3))
        q = np.array([1.578,1.578,1.578,1.578,1.578])
  #Guardar resultado para enviar al cliente.
  result.q = q
  #Topicos para movimiento del brazo robot simulacion e implementacion
  topic1 = '/robot/joint1_position_controller/command'
  topic2 = '/robot/joint2_position_controller/command'
  topic3 = '/robot/joint3_position_controller/command'
  topic4 = '/robot/joint4_position_controller/command'
  topic5 = '/robot/joint5_position_controller/command'
  topicg = '/robot/gripper/command'
  topic6 = '/robot/joint6_position_controller/command'
  #Publicadores
  pub1 = rospy.Publisher(topic1, Float64, queue_size=10, latch=True)
  pub2 = rospy.Publisher(topic2, Float64, queue_size=10, latch=True)
  pub3 = rospy.Publisher(topic3, Float64, queue_size=10, latch=True)
  pub4 = rospy.Publisher(topic4, Float64, queue_size=10, latch=True)
  pub5 = rospy.Publisher(topic5, Float64, queue_size=10, latch=True)
  pubg = rospy.Publisher(topicg, Float64, queue_size=10, latch=True)
  pub6 = rospy.Publisher(topic6, Float64MultiArray, queue_size=10, latch=True)
  #Tipo de datos a enviar
  j1 = Float64()
  j2 = Float64()
  j3 = Float64()
  j4 = Float64()
  j5 = Float64()
  jg = Float64()
  j6 = Float64MultiArray()
  #Variables iniciales para procedimiento de movimiento y confirmacion
  m6=0
  j=0
  i=0
  l=0
  rate = rospy.Rate(5)
  #Limites de movimiento
  while j <len(q):
        if q[j]<0:
             q[j]=0
        if q[j]>3.14:
             q[j]=3.14
        j+=1
  #Movimiento del brazo robot, priorizando mov del Motor 1 y Motor 4.
  #Envio de datos 3 veces para confirmacion de movimiento
  while i<3:
        j1.data = np.round(q[0],2)
        pub1.publish(j1)
	qq3=(q[3]+0.40);
	j4.data = np.round(qq3,2)
	pub4.publish(j4)
        time.sleep(1)
        i+=1
  #Movimiento del brazo robot, orden de movimiento M2, M3, M5.	
  while l<3:
        j2.data = np.round(q[1],2)
  	pub2.publish(j2)
	j3.data = np.round(q[2],2)
	pub3.publish(j3)
	
	j5.data = np.round(q[4],2)
	pub5.publish(j5)
        time.sleep(1)
        l+=1
  #Movimiento de pinzas
  j6.data = [m6, m6]
  jg.data = m6  
  pubg.publish(jg)
  pub6.publish(j6)
  #Guardar angulos alcanzados en variable MOVE
  MOVE=np.array([j1,j2,j3,j4,j5,jg])
  print('MOVE')
  print(MOVE)
  
  JMOVE=np.array([q[0],q[1],q[2],q[3],q[4]])
  time.sleep(1)
  m6=3.14
  i=0
  
  #Cerrar Pinza, confirmar movimiento
  while i<3:
        j6.data = [m6, m6]
	jg.data = m6  
	pubg.publish(jg)
	pub6.publish(j6)
        time.sleep(1)
        print("confirm M6")
        i+=1
  	
  time.sleep(1)
  #Secuencia de movimientos para dejar elemento en un determinado punto
  #Cambio de posicion de la base para dejar el elemento en un punto cercano al anterior
  m1=[1.57-i*0.05,3.14-i*0.05,3.14-i*0.05,3.14-i*0.05,1.57]
  m2=[1.57,1.57,1.57,1.57,1.57]
  m3=[1.57,0   ,0   ,1.57,3.14]
  m4=[1.57,0.7 ,0.7 ,1.57,0]
  m5=[1.57,1.57,1.57,1.57,1.57]
  m6=[3.14,3.14 ,0   ,0   ,0]
  i=0
  rate = rospy.Rate(5)
  #Movimiento secuencial paso a paso recorriendo arreglos
  while i <len(m1):
        j1.data = m1[i]
        j2.data = m2[i]
        j3.data = m3[i]
        j4.data = m4[i]
        j5.data = m5[i]
        j6.data = [m6[i], m6[i]]
        jg.data = m6[i]
        MOVE=[j1,j2,j3,j4,j5,jg]
        print(MOVE)
	k=0
	#Confirmacion de movimiento y orden
	while k<2:
		pub2.publish(j2)
		pubg.publish(jg)
		pub6.publish(j6)
		time.sleep(1)
		k+=1
	p=0
	#Confirmacion de movimiento y orden
	while p<2:
		pub1.publish(j1)
		time.sleep(1)
		p+=1
        pub3.publish(j3)
        pub4.publish(j4)
        pub5.publish(j5)
        i+=1
  #Retorno a posicion de seguridad
  j1.data=1.57
  j2.data=0.75
  j3.data=3.14
  j4.data=1.57
  j5.data=1.57
  j6.data=0
  jg.data=0
  pub1.publish(j1)
  pub3.publish(j3)
  pub4.publish(j4)
  pub5.publish(j5)
  pubg.publish(jg)
  pub2.publish(j2)
  pub6.publish(j6)
  result.MOVE = pos 
  server.set_succeeded(result)

def inicio():
    #Declaracion de variables globales para inicializar servidor y robot
    global server,q0
    #Envio de datos para movimiento inicial del robot 	
    m1=1.57
    m2=0.75
    m3=3.14
    m4=3.14
    m5=1.57
    m6=0
    topic1 = '/robot/joint1_position_controller/command'
    topic2 = '/robot/joint2_position_controller/command'
    topic3 = '/robot/joint3_position_controller/command'
    topic4 = '/robot/joint4_position_controller/command'
    topic5 = '/robot/joint5_position_controller/command'
    topicg = '/robot/gripper/command'
    topic6 = '/robot/joint6_position_controller/command'
    pub1 = rospy.Publisher(topic1, Float64, queue_size=10, latch=True)
    pub2 = rospy.Publisher(topic2, Float64, queue_size=10, latch=True)
    pub3 = rospy.Publisher(topic3, Float64, queue_size=10, latch=True)
    pub4 = rospy.Publisher(topic4, Float64, queue_size=10, latch=True)
    pub5 = rospy.Publisher(topic5, Float64, queue_size=10, latch=True)
    pubg = rospy.Publisher(topicg, Float64, queue_size=10, latch=True)
    pub6 = rospy.Publisher(topic6, Float64MultiArray, queue_size=10, latch=True)        
    j1 = Float64()
    j2 = Float64()
    j3 = Float64()
    j4 = Float64()
    j5 = Float64()
    jg = Float64()
    j6 = Float64MultiArray()
    j1.data=m1
    j2.data=m2
    j3.data=m3
    j4.data=m4
    j5.data=m5
    j6.data=m6
    jg.data=m6
    pub1.publish(j1)
    pub2.publish(j2)
    pub3.publish(j3)
    pub4.publish(j4)
    pub5.publish(j5)
    pubg.publish(jg)
    pub6.publish(j6)
    # Initial configuration
    q0 = np.array([1.578,1.578,1.578,1.578,1.578])
    # Resultado con la posicion inicial, el efector final respecto a la base
    T = fkine_BRT(q0)
    x0 = T[0:3,3]
    time.sleep(2)
    print("Posicion inicial y orientacion del efector final")
    print(np.round(x0,3))   
    #Inicializacion de Servidor	
    server = actionlib.SimpleActionServer('BRT_ACTION', BRTAction, gradiente, False)
    server.start()


if __name__=="__main__":
  #Inicializar nodo del servidor
  rospy.init_node('BRT_action_server')
  print("Action Server has started")
  #Funcion para inicio de toma de valores y movimiento
  inicio()
  rospy.spin()
