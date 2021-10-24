# -*- coding: utf-8 -*-
"""
Created on Sat Dec 22 18:25:39 2019

@author: riley
"""

import vrep
import sys
import time

velcoef = 0

#-----Try to connect---------------
vrep.simxFinish(-1)
clientID = vrep.simxStart('127.0.0.1',19999,True,True,10000,5)
if clientID != -1:
    print("Connected to remote API server")
else:
    print("Not connected to remote API server")
    sys.exit ("could not connect")
    
#-----Start the Paused Simulation

err_code = vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot)
#err_code = vrep.simxSetJointPosition(clientID,j2,0,vrep.simx_opmode_oneshot)

#-----Initialize Joint Handles---------
err_code,j1 = vrep.simxGetObjectHandle(clientID,"J1",vrep.simx_opmode_blocking)
err_code,j2 = vrep.simxGetObjectHandle(clientID,"J2",vrep.simx_opmode_blocking)

#-----Function to get the position of joint 2 (the free joint)---------
def updatePendPos():
    err_code,posj2 = vrep.simxGetJointPosition(clientID, j2,vrep.simx_opmode_streaming)
    return posj2
def updateMotPos():
    err_code,posj1 = vrep.simxGetJointPosition(clientID, j1,vrep.simx_opmode_streaming)
    return posj1
   
kp = 29#28
ki = 1.8#1.8
kd = 0.05#0.01 - 0.02 <---BIG MAYBE
perr = 0
target = 0
I=0
inv = 0.001
strt = 0

kp1 = 7#6 - 5 -8
ki1 = 0.005#0.001 - 0.001
kd1 = 800#300 - 10
po1d = 0
I1 = 0

a = 0
spe = -5
poo=0

while True:
    
    po = updatePendPos()
    ## SWING UP ##
    #lpo = 0
#    while (po>0.6 or po<-0.6):
#        po = updatePendPos()
#        if abs(poo)>abs(po):
#            if po<0:
#                vel = -6#-5
#            else:
#                vel = 6#5
#            err_code = vrep.simxSetJointTargetVelocity(clientID,j1,vel,vrep.simx_opmode_streaming)
#        poo = po
        
        
    err = target - po
    #print(err)
    P = err * kp
    I = I + ki*err
    D = kd * (err-perr)/inv
    velcoef = P + I + D
    err_codey = vrep.simxSetJointTargetVelocity(clientID,j1,velcoef,vrep.simx_opmode_streaming)
    if(err_codey != 0 and strt == 10):
        err_code = vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)
        break
    perr = err
    #print(po)
    if (po>1 or po<-1):
        err_code = vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)
        break
    
    if(err-perr<0.001 or err-perr>-0.001):# 0.0001 and -0.0001
        po1 = updateMotPos()

        if((po1-po1d) < 3):
            D1 = kd1*(po1-po1d)
            P1 = po1*kp1
            I1 = I1+ki1*po1

        pid1 = 0.001*(P1+I1+D1)
        po1d = po1     
        target = pid1

    else:
        target = 0 
    
    #print(I1)
    time.sleep(inv)
    if(strt <10 ):
        strt = strt+1
    a = a + 1
    if (a>10000):
        if (target == -1):
            target =1
        else:
            target = -1
        a = 0