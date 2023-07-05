import threading
import time
import sys 
sys.path.append(r'/home/administrator/copter_script')
from multiCopterHandler import *

altitude = 10

class MyCopterHandler(CopterHandler):
    def __init__(self, name):
        super(MyCopterHandler,self).__init__(name)

        self.flight_thread = threading.Timer(0, self.flight_trajectory)
        self.flight_thread.daemon = True
        self.flight_thread.start()

        self.battery_thread = threading.Timer(0, self.battery_condition)
        self.battery_thread.daemon = True
        self.battery_thread.start()

    def flight_trajectory(self):
        time.sleep(1)
        print("arm:", self.armed, "mode:", self.mode)
        if self.connected:
            self.change_mode(MODE_GUIDED)
            self.arm(True)
            self.takeoff(altitude)
            time.sleep(20)
            while True:
                self.move_local(-15,25,altitude)
                time.sleep(40)
                self.move_local(20,25,altitude)
                time.sleep(40)
                self.move_local(20,-25,altitude)
                time.sleep(40)
                self.move_local(-15,-25,altitude)
                time.sleep(40)

    def battery_condition(self):
        time.sleep(1)
        if self.connected:
            print("testing battery")
            while True:
                #print('position: '+str(self.x)+' '+str(self.y)+' '+str(self.z))
                if self.battery<0.2:
                    self.return_to_home()
                    break
            self.disconnect()



if __name__ == "__main__":
    v = MyCopterHandler("drone1")
    v.enable_topics_for_read()
    v.connect("node1", rate=10)