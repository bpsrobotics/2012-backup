import serial
import time
from networktables import NetworkTables
NetworkTables.initialize(server="10.28.98.2")
sd = NetworkTables.getTable("SmartDashboard")


ard = serial.Serial("COM7",9600)
while True:
	try:
		thingy = ard.readline()
		value = float(thingy)
		sd.putNumber("arm controller deg",value)
		print(value)
	except: pass
	time.sleep(1.0/50.0)
