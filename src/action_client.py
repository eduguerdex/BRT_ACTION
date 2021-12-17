#!/usr/bin/env python
import rospy
import actionlib
from BRT_ACTION.msg import BRTAction, BRTGoal, BRTResult
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from BraccioDEV import *
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers
import time

posx=[]
posy=[]
posz=[]
posx1=[]
posy1=[]
posz1=[]
camx=-0.02
camy=0.525
camz=0.3
caja=0.05
k=0
N=[]
NL=[]
xd=[0,0,0]
def callback(markers):
        rate= rospy.Rate(1)
        global posx,camx
        global posy,camy,k
        global posz,camz,caja,xd,NL
        
        if k==0:
            for m in markers.markers:
                    N.append(m.id)
             	    #Identificar TAG
		    NS=set(N)
		    NL=list(NS)
                    marker_pose = m.pose.pose
                    pos = marker_pose.position
                    ori = marker_pose.orientation
		    #Identificar POS
                    posx1.append(pos.x)
	            posx2=set(posx1)
		    posx=list(posx2)
                    posy1.append(pos.y)
                    posy2=set(posy1)
		    posy=list(posy2)
                    posz1.append(pos.z)
	            posz2=set(posz1)
		    posz=list(posz2)

if __name__=="__main__":
    rospy.init_node('BRT_action_client')
    sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback)
    time.sleep(3)
    print("Cliente activado")
    tag=len(NL)
    print("TAGs Detectados:")
    print(tag)
    i=0
    xr=[0,0,0]
    for i in range(tag): 
	sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback)
    	time.sleep(3)
        client = actionlib.SimpleActionClient('BRT_ACTION', BRTAction)
        client.wait_for_server()
	print("ID Marcador")
	print NL[i]
        fx=camx-posx[i]
	fz=camz-posy[i]+caja
	fy=-(camy-posz[i])

	fx=np.round(fx,3)
	fy=np.round(fy,3)
	fz=np.round(fz,3)

	if fz<0.10:
	       fz=0.1025
	if fz>0.122:
	       fz=0.1025
	#if fx<0:
	 #      fz=0.115

        xd=[fy,fx,fz]
        xm=[posx,posy,posz]
        xcam=[-camy,camx,camz+caja]
        if np.array_equal(xd,xcam) == False:
	       print("AR Detectado")
        else: 
               print("Esperando lectura")

        goal = BRTGoal()
        
        if np.array_equal(xd,xr) == False:
		    time.sleep(5)
                    goal.xd=xd
                    print("POSICION SOLICITADA:")
                    print(goal.xd)
                    client.send_goal(goal)
                    client.wait_for_result()
                    #resultados
                    MOV=client.get_result().MOVE
                    Q=client.get_result().q
                    MOV=np.round(MOV,3)
                    Q=np.round(Q,3)
                    print("POSICION FINAL:")
                    print(MOV)
                    print("ANGULOS DESPLAZADOS:")
                    print(Q)
