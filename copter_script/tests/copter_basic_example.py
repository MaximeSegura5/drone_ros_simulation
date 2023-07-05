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

    def flight_trajectory(self):
        time.sleep(1)
        print("arm:", self.armed, "mode:", self.mode)
        if self.connected:
            self.change_mode(MODE_GUIDED)
            self.arm(True)
            self.takeoff(altitude)
            time.sleep(15)
            #self.move_global(-35.36154932,149.16500469,10)
            self.move_local(20,20,altitude)
            time.sleep(15)
            self.change_mode(MODE_LAND)
            time.sleep(10)
            print("landing DONE")


if __name__ == "__main__":
    v = MyCopterHandler("drone1")
    v.enable_topics_for_read()
    v.connect("node1", rate=10)