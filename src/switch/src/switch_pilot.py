import rospy
from switch.msg import switch as switch_message
from speedlib.dcc import dcc_object, dcc_switches
from speedlib.dcc.dcc_object import DCCObject
from speedlib.dcc .dcc_switches import Switch
import sys
import signal

def start_controller():
    dcc_object.start()

def stop_controller(signal, frame):
    dcc_object.stop()
    sys.exit(0)

class SwitchPiloteNode:
    
    def __init__(self, num_switch, biais_id):
        num_switch = 3
        self.switch ={}
        for i in range(1, num_switch):
            self.switch[i] = Switch("DCC"+str(i), i, biais_id)
    
    def callback(self, data):
        print(data)

        if not isinstance(data.switch_name, str):
            raise TypeError("Switch_name must be a str but got "+str(data.switch_name))
        
        if not isinstance(data.switch_number, int):
            raise TypeError("Switch_number must be an int but got "+str(data.switch_number))

        if not isinstance(data.biais_id, int):
            raise TypeError("biais_id must be a str but got "+str(data.biais_id))
        
        if data.biais_id not in [1, 2]:
            raise ValueError("biais_id must be 1 or 2 but got :" +str(data.biais_id))
        
        if data.switch_command == "biais":
            self.switch[data.switch_number].biais = data.biais_state
        
        if data.switch_command == "biais_info":
            print(self.switch[data.switch_number])

if __name__=='__main__':

   start_controller()
   switchnode = SwitchPiloteNode()
   rospy.init_node('switch', anonymous=True)
   print("Initialisation du noeud")
   rospy.Subscriber("switch/command", switch_message, switchnode.callback)

   signal.signal(signal.SIGINT,stop_controller)

   rospy.spin()
