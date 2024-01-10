from paho.mqtt import client as mqtt_client
from Controller import MovementController

class Movement():

    client: mqtt_client.Client
    moveCon: MovementController
    methodMap:dict
    def __init__(self,moveCon:MovementController,id,ip,port=1883):

        self.moveCon = moveCon
        self.id = id
        self.methodMap = {
            "forward":self.moveCon.forward,
            "back":self.moveCon.back,
            "left":self.moveCon.left,
            "right":self.moveCon.right,
            "turnLeft":self.moveCon.turnLeft,
            "turnRight":self.moveCon.turnRight,
            "stop":self.moveCon.stop
        }

        self.client = mqtt_client.Client()
        self.client.connect(ip, port, 60)
        self.client.on_message = self.on_message
        self.client.loop_start()

        self.client.subscribe("movement")

        
        print("Movement Controller MQTT Initialized")
    def on_message(self,client, userdata, msg):
        print(msg.topic+" "+str(msg.payload))
        self.methodMap[msg.topic](msg.payload)
        
    

    def start(self):
        self.client.loop_start()
    def stop(self):
        self.client.loop_stop()
    def __del__(self):
        self.client.loop_stop()
        self.client.disconnect()
        print("Movement Controller Deinitialized")
        

