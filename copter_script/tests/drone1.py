import threading
import time
import sys 
sys.path.append(r'/home/administrator/copter_script')
from multiCopterHandler import *

altitude = 20

class MyCopterHandler(CopterHandler):
    def __init__(self,name):
        super(MyCopterHandler,self).__init__(name)

        self.flight_thread = threading.Timer(0, self.flight_trajectory)
        self.flight_thread.daemon = True
        self.flight_thread.start()

        self.battery_thread = threading.Timer(0, self.battery_condition)
        self.battery_thread.daemon = True
        self.battery_thread.start()

    def flight_trajectory(self):
        time.sleep(1)
        #print("arm:", self.armed, "mode:", self.mode)
        if self.connected:
            self.change_mode(MODE_GUIDED)
            self.arm(True)
            self.takeoff(altitude)
            time.sleep(10)
            while not self.returning_to_home:
                self.move_local(-20,20,altitude)
                time.sleep(15)
                self.move_local(20,20,altitude)
                time.sleep(15)
                self.move_local(20,-20,altitude)
                time.sleep(15)
                self.move_local(-20,-20,altitude)
                time.sleep(15)
            print("thread traj finished")
            return

    def battery_condition(self):
        time.sleep(1)
        if self.connected:
            print("testing battery")
            while True:
                #print('position: '+str(self.x)+' '+str(self.y)+' '+str(self.z))
                if self.battery<0.85:
                    self.return_to_home()
                    break
            print("thread batt finished")
            return



if __name__ == "__main__":
    drone1 = MyCopterHandler("drone1")
    drone1.enable_topics_for_read()
    drone1.connect("node1", rate=10)
    print("au sol")
