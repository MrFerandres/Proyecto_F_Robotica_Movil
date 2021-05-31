import numpy as np
import time
import math as m
import random
import matplotlib.pyplot as plt
import os
import sim as vrep # access all the VREP elements
from skimage.draw import line
import threading

global carprio #Cosa de hilos
carprio=[]
for i in range(6):
    carprio.append([])

def angdiff(t1, t2):
    """
    Compute the angle difference, t2-t1, restricting the result to the [-pi,pi] range
    """
    # The angle magnitude comes from the dot product of two vectors
    angmag = m.acos(m.cos(t1) * m.cos(t2) + m.sin(t1) * m.sin(t2))
    # The direction of rotation comes from the sign of the cross product of two vectors
    angdir = m.cos(t1) * m.sin(t2) - m.sin(t1) * m.cos(t2)
    return m.copysign(angmag, angdir)

#Clase para Crear Hilos
class myRobot(threading.Thread):
    def __init__(self, client,numb):
        threading.Thread.__init__(self)
        self.errp=10
        self.clientID=client
        self.id=numb
        self.usensor =[]

        if(self.id==-1):
            print('Inicializando robot y motores del robot {}'.format(self.id+1))
            eer, self.robot=vrep.simxGetObjectHandle(self.clientID, 'Pioneer_p3dx', vrep.simx_opmode_blocking)
            eer, self.motorL=vrep.simxGetObjectHandle(self.clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_blocking)
            eer, self.motorR=vrep.simxGetObjectHandle(self.clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_blocking)
            for i in range(1,17):
                err, s = vrep.simxGetObjectHandle(self.clientID, 'Pioneer_p3dx_ultrasonicSensor'+str(i), vrep.simx_opmode_blocking)
                self.usensor.append(s)
        else:
            print('Inicializando robot y motores del robot {}'.format(self.id+1))
            eer, self.robot = vrep.simxGetObjectHandle(self.clientID, 'Pioneer_p3dx#'+str(self.id), vrep.simx_opmode_blocking)
            err, self.motorL = vrep.simxGetObjectHandle(self.clientID, 'Pioneer_p3dx_leftMotor#' + str(self.id), vrep.simx_opmode_blocking)
            err, self.motorR = vrep.simxGetObjectHandle(self.clientID, 'Pioneer_p3dx_rightMotor#' + str(self.id), vrep.simx_opmode_blocking)
            for i in range(1,17):
                # print('Agregando sensor {} del robot {}'.format(j,i))
                err, s = vrep.simxGetObjectHandle(self.clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(i) + '#' + str(self.id),vrep.simx_opmode_blocking)
                self.usensor.append(s)
        print('haciendo primera lectura negativa de los sensores del robot {}'.format(self.id+1))
        for i in range(16):
            err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(self.clientID, self.usensor[i], vrep.simx_opmode_streaming)
        self.puntox = []
        self.puntoy = []
        for i in range(6):
            self.puntox.append(random.uniform(-5.5,5.5))
            self.puntoy.append(random.uniform(-5.5,5.5))
        print('El robot {}, tendra que pasar por los puntos\nx={}\ny={}\n'.format(self.id+1,self.puntox,self.puntoy))

        #Datos para los hilos
        ret, carpos = vrep.simxGetObjectPosition(self.clientID, self.robot, -1, vrep.simx_opmode_streaming)
        carprio[self.id + 1].append(self.id + 1) # id del robot
        carprio[self.id + 1].append(carpos[0])  # posicion en x
        carprio[self.id + 1].append(carpos[1])  # posicion en y
        carprio[self.id + 1].append(False)  # estado del robot (frenado porque tiene que esperar)
        carprio[self.id + 1].append(False)   # ya acabo todos los puntos?

    def run(self):
        print('inicia hilo para el robot {}\n{}'.format(self.id+1,carprio[self.id+1]))
        ret, carpos = vrep.simxGetObjectPosition(self.clientID, self.robot, -1, vrep.simx_opmode_streaming)
        ret, carrot = vrep.simxGetObjectOrientation(self.clientID, self.robot, -1, vrep.simx_opmode_streaming)
        for i in range(len(self.puntox)):
            self.errp=10
            while self.errp>0.5:
                carprio[self.id +1][3] = False

                ret, carpos = vrep.simxGetObjectPosition(self.clientID, self.robot, -1, vrep.simx_opmode_buffer)
                ret, carrot = vrep.simxGetObjectOrientation(self.clientID, self.robot, -1, vrep.simx_opmode_streaming)
                #print(self.errp)
                difx=(self.puntox[i] - carpos[0])**2
                dify=(self.puntoy[i] - carpos[1])**2
                while m.isnan(difx) or m.isnan(dify):
                    ret, carpos = vrep.simxGetObjectPosition(self.clientID, self.robot, -1, vrep.simx_opmode_buffer)
                    difx = (self.puntox[i] - carpos[0]) ** 2
                    dify = (self.puntoy[i] - carpos[1]) ** 2
                self.errp = m.sqrt(difx + dify)
                carprio[self.id + 1][1]=carpos[0]
                carprio[self.id + 1][2] = carpos[1]

                angd = m.atan2(self.puntoy[i] - carpos[1], self.puntox[i] - carpos[0])
                errh = angdiff(carrot[2], angd)

                #Si el sensor detecta algo, nos dice con quien se detecto
                for j in range(1,7):
                    err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(self.clientID,self.usensor[j],vrep.simx_opmode_buffer)
                    if state == True:
                        #print('el sensor {} del robot {}, detecto un objeto'.format(j + 1, self.id + 1))
                        for k in range(6):
                            if k==self.id+1:
                                continue
                            distancia=m.sqrt((carprio[k][1]-carprio[self.id+1][1])**2+(carprio[k][2]-carprio[self.id+1][2])**2)
                            if distancia<0.75 and (self.id<k and carprio[k][4]==False):
                                t = time.time()
                                while time.time() - t < 2:
                                    errf = vrep.simxSetJointTargetVelocity(self.clientID, self.motorL, -1,vrep.simx_opmode_streaming)
                                    errf = vrep.simxSetJointTargetVelocity(self.clientID, self.motorR, -1,vrep.simx_opmode_streaming)
                                errf = vrep.simxSetJointTargetVelocity(self.clientID, self.motorL, 0,vrep.simx_opmode_streaming)
                                errf = vrep.simxSetJointTargetVelocity(self.clientID, self.motorR, 0,vrep.simx_opmode_streaming)
                            elif distancia <0.5:
                                carprio[self.id +1][3] = True
                            elif distancia<1:
                                #print('el robot {} se topo con el robot {}'.format(k,self.id+1))
                                if (self.id<k and carprio[k][4]==False):
                                    carprio[self.id +1][3] = True
                                else:
                                    carprio[self.id +1][3] = False
                if carprio[self.id +1][3]:
                    errf = vrep.simxSetJointTargetVelocity(self.clientID, self.motorL, 0, vrep.simx_opmode_streaming)
                    errf = vrep.simxSetJointTargetVelocity(self.clientID, self.motorR, 0, vrep.simx_opmode_streaming)
                    continue

                Kv = 0.5
                Kh = 2.5
                hd = 0
                r = 0.5 * 0.195
                L = 0.311

                v = Kv * self.errp
                omega = Kh * errh

                #Uncomment for continuous control
                ul = (v / r - L * omega / (2 * r)) * 0.2  # se multiplica por 0.5 porque la velocidad es mucha y cuando calcula otra vez el ERRP se pasa haciendo que este en un ciclo infinito
                ur = (v / r + L * omega / (2 * r)) * 0.2

                # Para girar a la derecha (ang neg) motorR- & motorL+ -> cuando ang > 0
                # Para girar a la izquierda (ang pos) motorR+ & motorL- -> cuando ang < 0
                errf = vrep.simxSetJointTargetVelocity(self.clientID, self.motorL, ul, vrep.simx_opmode_streaming)
                errf = vrep.simxSetJointTargetVelocity(self.clientID, self.motorR, ur, vrep.simx_opmode_streaming)



                #print('\nrobot {}\nrot del robot {}\npos del robot {}\n'.format(self.id+1,carrot,carpos))
            print('Se encontro el punto:x={}||y={} \nDel robot {} con un errp de ={}\nUbicacion del robot x={}\ny={}'.format(self.puntox[i],self.puntoy[i],self.id+1,self.errp,carpos[0],carpos[1]))
            print('xd={}\nyd={}\nsqrt={}\n'.format((self.puntox[i] - carpos[0]) ** 2,(self.puntoy[i] - carpos[1]) ** 2,m.sqrt((self.puntox[i] - carpos[0]) ** 2 + (self.puntoy[i] - carpos[1]) ** 2)))
        errf = vrep.simxSetJointTargetVelocity(self.clientID, self.motorL, 0, vrep.simx_opmode_streaming)
        errf = vrep.simxSetJointTargetVelocity(self.clientID, self.motorR, 0, vrep.simx_opmode_streaming)

        """
        t = time.time()
        while time.time() - t < 20:
            for i in range(16):
                err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(self.clientID,self.usensor[i],vrep.simx_opmode_buffer)
                if state == True:
                    print('el sensor {} del robot {}, detecto un objeto'.format(i+1,self.id+1))

        ret, carpos = vrep.simxGetObjectPosition(self.clientID, self.robot, -1, vrep.simx_opmode_streaming)
        ret, carrot = vrep.simxGetObjectOrientation(self.clientID, self.robot, -1, vrep.simx_opmode_streaming)
        print('\nrobot {}\nrot del robot {}\npos del robot {}\n'.format(self.id + 1, abs(carrot[2]*180/m.pi), carpos))
        """
        print('El robot {}, ya acabo de navegar todos los puntos'.format(self.id))
        carprio[self.id + 1][4]=True
        for k in range(6):
            while not carprio[k][4]:
                for i in range(8):
                    err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(self.clientID,self.usensor[i],vrep.simx_opmode_buffer)
                    if state == True:
                        t=time.time()
                        while time.time() - t < 2:
                            errf = vrep.simxSetJointTargetVelocity(self.clientID, self.motorL, -1,vrep.simx_opmode_streaming)
                            errf = vrep.simxSetJointTargetVelocity(self.clientID, self.motorR, -1,vrep.simx_opmode_streaming)
                        errf = vrep.simxSetJointTargetVelocity(self.clientID, self.motorL, 0,vrep.simx_opmode_streaming)
                        errf = vrep.simxSetJointTargetVelocity(self.clientID, self.motorR, 0,vrep.simx_opmode_streaming)



                for i in range(8,16):
                    err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(self.clientID,self.usensor[i],vrep.simx_opmode_buffer)
                    if state == True:
                        t = time.time()
                        while time.time() - t < 2:
                            errf = vrep.simxSetJointTargetVelocity(self.clientID, self.motorL,  1,vrep.simx_opmode_streaming)
                            errf = vrep.simxSetJointTargetVelocity(self.clientID, self.motorR,  1,vrep.simx_opmode_streaming)
                        errf = vrep.simxSetJointTargetVelocity(self.clientID, self.motorL, 0,vrep.simx_opmode_streaming)
                        errf = vrep.simxSetJointTargetVelocity(self.clientID, self.motorR, 0,vrep.simx_opmode_streaming)


        print('\n\nTermina hilo {} con la info de carprio ={}\n\n'.format(self.id + 1,carprio[self.id + 1]))



vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',-1,True,True,5000,5) # start a connection
if clientID!=-1:
	print ('Connected to remote API server {}'.format(clientID))
else:
	print('Not connected to remote API server')
	sys.exit("No connection")
print(carprio)


robot0=myRobot(clientID,-1)
robot1=myRobot(clientID,0)
robot2=myRobot(clientID,1)
robot3=myRobot(clientID,2)
robot4=myRobot(clientID,3)
robot5=myRobot(clientID,4)

robot0.start()
robot1.start()
robot2.start()
robot3.start()
robot4.start()
robot5.start()

robot0.join()
robot1.join()
robot2.join()
robot3.join()
robot4.join()
robot5.join()



print ("Saliendo del hilo principal\n")

alpha = input('Da enter para salir\n')


# Getting handles for the motors and robot
# Falta poner los otros robots con sus respectivos nombres
"""
motorL = []
motorR = []
robot = []
err, l = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_blocking)
err, r = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_blocking)
err, rob = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx', vrep.simx_opmode_blocking)
motorL.append(l)
motorR.append(r)
robot.append(rob)

for i in range(5):
    err, l = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor#'+str(i), vrep.simx_opmode_blocking)
    err, r = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor#'+str(i), vrep.simx_opmode_blocking)
    err, rob = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx#'+str(i), vrep.simx_opmode_blocking)
    motorL.append(l)
    motorR.append(r)
    robot.append(rob)


# Inicializacion de los sensores
# Faltan los otros sensores
print('Iniciando los 16 sensores de los 6 robots')
usensor = []
for i in range(6):
    usensor.append([])

for i in range(1,17):
    err, s = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor'+str(i), vrep.simx_opmode_blocking)
    usensor[0].append(s)

for i in range(5):
    for j in range(1,17):
        #print('Agregando sensor {} del robot {}'.format(j,i))
        err, s = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor'+str(j)+'#'+str(i), vrep.simx_opmode_blocking)
        usensor[i+1].append(s)

print('Iniciando primera lectura falsa de todos los sensores')
# Lectura falsa de los sensores
for i in range(6):
    for j in range(16):
        err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(clientID, usensor[i][j], vrep.simx_opmode_streaming)



ret, carrot = vrep.simxGetObjectOrientation(clientID, robot[0], -1, vrep.simx_opmode_blocking)
print('\nla posicion del robot 1 es de {}\n'.format(carrot))

errf = vrep.simxSetJointTargetVelocity(clientID, motorL[0], -1, vrep.simx_opmode_streaming)
errf = vrep.simxSetJointTargetVelocity(clientID, motorR[0],  1, vrep.simx_opmode_streaming)
errf = vrep.simxSetJointTargetVelocity(clientID, motorL[1], -1, vrep.simx_opmode_streaming)
errf = vrep.simxSetJointTargetVelocity(clientID, motorR[1],  1, vrep.simx_opmode_streaming)
errf = vrep.simxSetJointTargetVelocity(clientID, motorL[2], -1, vrep.simx_opmode_streaming)
errf = vrep.simxSetJointTargetVelocity(clientID, motorR[2],  1, vrep.simx_opmode_streaming)
errf = vrep.simxSetJointTargetVelocity(clientID, motorL[3], -1, vrep.simx_opmode_streaming)
errf = vrep.simxSetJointTargetVelocity(clientID, motorR[3],  1, vrep.simx_opmode_streaming)
errf = vrep.simxSetJointTargetVelocity(clientID, motorL[4], -1, vrep.simx_opmode_streaming)
errf = vrep.simxSetJointTargetVelocity(clientID, motorR[4],  1, vrep.simx_opmode_streaming)
errf = vrep.simxSetJointTargetVelocity(clientID, motorL[5], -1, vrep.simx_opmode_streaming)
errf = vrep.simxSetJointTargetVelocity(clientID, motorR[5],  1, vrep.simx_opmode_streaming)





t = time.time()

while time.time()-t < 12:
    for i in range(6):
        for j in range(16):
            err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(clientID, usensor[i][j], vrep.simx_opmode_buffer)
            if state == True:
                print('el sensor {} del robot {}, detecto un objeto'.format(j+101,i+101))



    #errf = vrep.simxSetJointTargetVelocity(clientID, motorL[0], -1, vrep.simx_opmode_streaming)
    #errf = vrep.simxSetJointTargetVelocity(clientID, motorR[0], 1, vrep.simx_opmode_streaming)
    #errf = vrep.simxSetJointTargetVelocity(clientID, motorL[1], 1, vrep.simx_opmode_streaming)
    #errf = vrep.simxSetJointTargetVelocity(clientID, motorR[1], -1, vrep.simx_opmode_streaming)
    #errf = vrep.simxSetJointTargetVelocity(clientID, motorL[2], -1, vrep.simx_opmode_streaming)
    #errf = vrep.simxSetJointTargetVelocity(clientID, motorR[2], -1, vrep.simx_opmode_streaming)
    #errf = vrep.simxSetJointTargetVelocity(clientID, motorL[3], 1, vrep.simx_opmode_streaming)
    #errf = vrep.simxSetJointTargetVelocity(clientID, motorR[3], 1, vrep.simx_opmode_streaming)
    #errf = vrep.simxSetJointTargetVelocity(clientID, motorL[4], 0, vrep.simx_opmode_streaming)
    #errf = vrep.simxSetJointTargetVelocity(clientID, motorR[4], 1, vrep.simx_opmode_streaming)
    #errf = vrep.simxSetJointTargetVelocity(clientID, motorL[5], 0, vrep.simx_opmode_streaming)
    #errf = vrep.simxSetJointTargetVelocity(clientID, motorR[5], -1, vrep.simx_opmode_streaming)


for i in range(6):
    print('Apagando el motor {} '.format(i))
    errf = vrep.simxSetJointTargetVelocity(clientID, motorL[i], 0.0, vrep.simx_opmode_streaming)
    errf = vrep.simxSetJointTargetVelocity(clientID, motorR[i], 0.0, vrep.simx_opmode_streaming)
"""
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)