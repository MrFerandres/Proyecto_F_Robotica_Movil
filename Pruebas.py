"""
    Este es solo un documento con pruebas con hilos que estaba haciendo para enterarme como se usaban
    Para mayor documentacion buscarn en internet
"""

import threading
import time
import random



class myRobot(threading.Thread):
    def __init__(self, id, nombre):
        threading.Thread.__init__(self)
        self.numb=id
        self.nombre=nombre
        self.puntox = []
        self.puntoy = []
        for i in range(6):
            self.puntox.append(random.uniform(-7, 7))
            self.puntoy.append(random.uniform(-7, 7))
    def run(self):
        print('hola, yo soy el hilo numero {} y mi nombre es {}. mucho gusto \n'.format(self.numb,self.nombre))
        time.sleep(10-self.numb*2)
        print('Los puntos por el cual va a pasar el robot {} son \nx={}\ny={}\n'.format(self.numb,self.puntox,self.puntoy))
        print('Bueno, ya me voy, yo era {} \n'.format(self.nombre))


#creo mis hilos
hilo1=myRobot(1,'Fer')
hilo2=myRobot(2,'marco')

#inicio los hilos
hilo1.start()
hilo2.start()



print ("Aqui acaba el hilo principal")