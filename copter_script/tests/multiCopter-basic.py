import subprocess
import sys

sys.path.append(r'/home/administrator/copter_script/tests')
subprocess.call(["python", "drone1.py"])
print("drone1 at home")
subprocess.call(["python", "drone2.py"])
print("drone2 at home")