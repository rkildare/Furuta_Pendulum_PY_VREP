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


#-----Initialize Joint Handles---------
err_code,j1 = vrep.simxGetObjectHandle(clientID,"J1",vrep.simx_opmode_blocking)
err_code,j2 = vrep.simxGetObjectHandle(clientID,"J2",vrep.simx_opmode_blocking)

#-----Function to get the position of joint 2 (the free joint)---------
def jupdate():
    err_code,posj2 = vrep.simxGetJointPosition(clientID, j2,vrep.simx_opmode_streaming)
    return posj2


#-----PI----

kp = 5#5
ki = 90#90
erri = 0
inv = 0.01
target = 0
    
    
while True:
    po = jupdate() #Get Pendulum Position

    #-----Establish a target - Had trouble with reversing the pendulums momentum without this block
    if (po<0):
        target = 0.1
    elif(po>0):
        target = -0.1
    else:
        target = 0
    #------
    
    err = target-po #Calculate Error
    P = err * kp #Calculated value of P
    I = erri * ki #Calculated value of I
    erri = erri + inv*err # Keep a running sum of error for the integral function
    velcoef = (P + I) #Summation of P and I to get the new velocity of joint 1
    err_code = vrep.simxSetJointTargetVelocity(clientID,j1,velcoef,vrep.simx_opmode_streaming)#Set the velocity of joint 1
    time.sleep(inv)#Wait a short abount of time