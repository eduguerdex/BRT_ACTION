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

posx=0
posy=0
posz=0
camx=0.0
camy=0.45
camz=0.3
caja=0.07
k=0
xd=[0,0,0]
def callback(markers):
        rate= rospy.Rate(1)
        global posx,camx
        global posy,camy,k
        global posz,camz,caja,xd
        
        if k==0:
            for m in markers.markers:
                    marker_pose = m.pose.pose
                    pos = marker_pose.position
                    ori = marker_pose.orientation
                    posx=pos.x
                    posy=pos.y
                    posz=pos.z
                    k=1

            fx=camx-posx
            fz=camz-posy+caja
            fy=-(camy-posz)

            fx=np.round(fx,3)
            fy=np.round(fy,3)
            fz=np.round(fz,3)

            if fz<0.1:
               fz=0.11
            xd=[fy,fx,fz]
            xm=[posx,posy,posz]
            xcam=[-camy,camx,camz+caja]
            if np.array_equal(xd,xcam) == False:
                print("Posicion del camara vs marcador")
                print xm

            else: 
                print("Esperando lectura")
            rate.sleep()
        return xd 

if __name__=="__main__":
    rospy.init_node('BRT_action_client')
    print("Cliente activado")

    client = actionlib.SimpleActionClient('BRT_ACTION', BRTAction)
    client.wait_for_server()

    goal = BRTGoal()
    sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback)
    xr=[0,0,0]
    time.sleep(3)

    if np.array_equal(xd,xr) == False:
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
    