from machine import I2C
from machine import Pin
import utime, _thread
from hydronode import NodeType,NodeFactory,Communication

NODE = NodeType.CLIMATE
ADDRESS = 0x1A

hydroponics = NodeFactory().createNode(ADDRESS,NODE)
comms = Communication(ADDRESS,NODE)

def processTask():
    print("process started")
    hydroponics.initialize()
    while True:
        hydroponics.process()
        #hydroponics.printData()
        utime.sleep(1)
        
def comTask():
    print("communication started")
    while True:
        #print("thread 1 Running")
        if comms.checkCommand() == True:
            data = hydroponics.getData()
            if data != None:
                comms.sendData(data)
        utime.sleep(0.1)

_thread.start_new_thread(processTask,())
comTask()
#processTask()
#comTask()

