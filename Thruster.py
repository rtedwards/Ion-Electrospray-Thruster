#Author:  Robert Edwards
	
import Adafruit_BBIO.UART as UART #Used to access UART on BBB
import serial
import time
import datetime
import sys

UART.setup("UART1")
ser = serial.Serial(port = "/dev/ttyO1", baudrate=115200)

SETUP = 0x00
HVDAC = 0x02
THRUST_GAP_TIME = 0x03
THRUST_TIMEOUT = 0x04	#10ms incrememnts
THRUST_REVERSE_TIME = 0x05	#time thrusters enabled before gap time
THRUST_REGISTER = 0x07
HV_SETPOINT = 0x08	#*4.096*455.05/65536
TEST_REGISTER = 0x0C
STATUS = 0x0D
FPGA_REV_REG_LOW = 0x0E
FPGA_REV_REG_HIGH = 0x0F
ADC0 = 0x10	#3.3V supply
ADC1 = 0x11	#+HK, +HV supply
ADC2 = 0x12 #-HK, -HV supply
ADC3 = 0x13	#+BUS, raw bus voltage
ADC4 = 0x14	#IS, primary side current
ADC5 = 0x15	#+5V supply voltage
ADC6 = 0x16	#TS1, temp sensor 1
ADC7 = 0x17	#TS2, temp sensor 2
CONTROLLER_GAIN = 0x1A	#HC controller gain
HVDAC_LIMIT = 0x1B	#limit at which HVDAC is ignored
HV_SETPOINT_LIMIT = 0x1C	#limit at which HV_Set is ignored
BTV_ADDR_SET = 0x20	#Batch thruster vector address
BTV_WRT_PORT = 0x21 	#Batch thrust vector write port
CMD_WRT_PORT = 0x22	#Batch command write port
HV_DIFF_SET = 0x23	#sets limit for HVCOM - GND
AUTO_PERIOD_SET = 0x26	#Set period range for HV Controller
AUTO_PULSE_WIDTH_SET = 0x27	#Set pulse used for voltage control
AUTO_PERIOD_READ = 0x28	#Read register returns the period
AUTO_PULSE_WIDTH_READ = 0x29	#Read register returns pulse width
SERIAL_PORT_FORCE = 0x30	#Switches from SPI to Serial

#Initialize variables
echo = 0
inbyte = 0
status_bits = bytearray([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])

setup = 0
hvdac = 0
thrust_gap_time = 0
thrust_timeout = 0
thrust_reverse_time = 0
thrust_register = 0
hv_setpoint = 0
status = 0
adc0 = 0
adc1 = 0
adc2 = 0
adc3 = 0
adc4 = 0
adc5 = 0
adc6 = 0
adc7 = 0
controller_gain = 0
hvdac_limit = 0
hv_setpoint_limit = 0
hv_diff_set = 0
auto_period_set = 0
auto_pulse_width_set = 0
auto_period_read = 0
auto_pulse_width_read = 0


### MAIN PROGRAM ###
def main():
	ser.close()
	ser.open()

	print "\n\n***** Thruster Setup and Control Program *****"
	
	print "Setting Defaults"
	print "Configuring to Serial Communications"
	print "Setting CL Voltage: Controller Gain Scheduler Enabled"
	print "Setting Controller Gain: 0x40"
	print "Setting HVDAC Limit: 0x0300"
	print "Setting HV Setpoint Limit: 1745V (0x038E)"
	print "Setting HV Difference Limit: 0xF000"
	print "Enabling Thruster 0"

	set2Serial()
	writeThruster(0x00, 0x0050)     #CL voltage, controller gain
    writeThruster(0x1A, 0x0040)     #Default Controller Gain	
	writeThruster(0x1B, 0x300)	#Set HVDAC Limit
	writeThruster(0x1C, 0x38E)	#Set HV Setpoint Limit
	writeThruster(0x23, 0xF000)	#Set HV_DIF 
	writeThruster(0x07, 0x0001)	#Set Thruster 0 on
	
	while ser.isOpen():
		print "\n(A) View ADC Data"
		print "(D) Read Thruster Data"
		print "(L) Set Limits"
		print "(T) Set Thruster"
		print "(Set) Set Register Manually"
		print "(Read) Read Register Manually"
		print "(Record) Record data in file"

		selection = raw_input("Selection: ")


		if (selection == 'A'):	#Print ADCs
			setADC0(readThruster(ADC0))
			setADC1(readThruster(ADC1))
			setADC2(readThruster(ADC2))
			setADC3(readThruster(ADC3))
			setADC4(readThruster(ADC4))
			setADC5(readThruster(ADC5))
			setADC6(readThruster(ADC6))
			setADC7(readThruster(ADC7))
			print "ADC0 (3.3V Supply):  ", getADC0(), "V"
			print "ADC1 (+HK Supply):   ", getADC1(), "V"
			print "ADC2 (-HK Supply):   ", getADC2(), "V"
			print "ADC3 (+BUS raw bus): ", getADC3(), "V"
			print "ADC4 (IS current):   ", getADC4(), "A"
			print "ADC5 (+5V Supply):   ", getADC5(), "V"
			print "ADC6 (Temp 1):       ", getADC6(), "C"
			print "ADC7 (Temp 2):       ", getADC7(), "C"

		elif (selection == 'D'):
                        readThrusterData()

		elif (selection == 'L'):
			in_bounds = 0
			while (in_bounds == 0):
				set_value = int(raw_input("HVDAC Limit (0 - 3FF): "), 16)
        			if (set_value < 0 or set_value > 0x3FF):
					print "Value out of range"
				else:
					in_bounds = 1
			writeThruster(0x1B, set_value)

			in_bounds = 0
			while (in_bounds == 0):
				set_value = int(raw_input("HV Setpoint Limit (0 - 1745V): "))
				if (set_value < 0 or set_value > 1745):
					print "Value out of range"
				else:
					in_bounds = 1
			writeThruster(0x1C, set_value)

			in_bounds = 0
			while (in_bounds == 0):			
				set_value = int(raw_input("HV Difference (0 - FFFF): "),16)
        			if (set_value < 0 or set_value > 0xFFFF):
					print "Value out of range"
				else:
					in_bounds = 1
			writeThruster(0x23, set_value)
		
		elif (selection == 'T'):
			in_bounds = 0
			while (in_bounds == 0):
				set_value = int(raw_input("Set Thrust Gap Time (1 - 256mS): "))
				if (set_value < 1 or set_value > 256):
					print "Value out of range"
				else:
					in_bounds = 1
        		writeThruster(0x03, set_value)

			in_bounds = 0
			while (in_bounds == 0):
       				set_value = int(raw_input("Set Thrust Reverse Time (1 - 65535mS): "))
        			if (set_value < 1 or set_value > 65535):
					print "Value out of range"
				else:
					in_bounds = 1
			writeThruster(0x05, set_value)

		elif (selection == 'Set'):
			setRegister()

		elif (selection == 'Read'):
			val = readRegister()

		elif (selection == 'Record'):
			print "\n Hit Ctrl-C to exit"
			f = open('Thruster.dat', 'w')	#Reset file
			f.write('%-12s %-12s %-12s %-12s %-12s %-12s %-12s %-12s %-12s %-12s %-12s %-12s %-12s %-12s\n' % \
				('Time[s]', \
				'HVDAC', \
				'GapTime[mS]', \
				'Timeout[S]', \
				'RevTime[mS]', \
				'ADC0[V]', \
				'ADC1[V]', \
				'ADC2[V]', \
				'ADC3[V]', \
				'ADC4[A]', \
				'ADC5[V]', \
				'ADC6[C]', \
				'ADC7[C]', \
				'Gain'))

			f.close()
			try:
				time0 = int(time.time()*10)
				previous_time = 0
				while True:
					time1 = int(time.time()*10)	#miliseconds
					#print "time:", time1-time0
					#print "time:", (time1-time0)%5
					
					#Record data once every 500mS
					if ((time1 - time0) % 5 == 0 and (time1-time0) != previous_time):
						print "Recording data"
						alrdy_recorded = 1
						saveData(time1-time0)	#get data from registers
					previous_time = time1-time0

			except KeyboardInterrupt:
				print "Closing file"
				break;

		else:
			print "ERROR: Don't be dense.   Read the instructions!\n"
	
	ser.close()
### End Main() ###

def readThrusterData():
	# Since I didn't want to pass in all the variable to the function
	# they have to be declared as global otherwise Python interprets 
	# that I want to initialize local variables.  This was only a test 
	# script.  I realize there are better and more robust ways to
	# accomplish the same thing.
	global setup
    global hvdac
    global thrust_gap_time
    global thrust_timeout
    global thrust_reverse_time
    global thrust_register
    global hv_setpoint
    global status
    global adc0
    global adc1
    global adc2
    global adc3
    global adc4
    global adc5
    global adc6
    global adc7
    global controller_gain
    global hvdac_limit
    global hv_setpoint_limit
    global hv_diff_set
    global auto_period_set
    global auto_pulse_width_set
    global auto_period_read
    global auto_pulse_width_read

	setThrusterData()
	print "0x00 - SETUP:", hex(setup)
	print "    ",
	for i in range(0,4):
		print (setup << i) & 0x01,
	print ""
	print "0x02 - HVDAC:", hex(hvdac)
	print "0x03 - THRUST_GAP_TIME:", thrust_gap_time, "mS"
	print "0x04 - THRUST_TIMEOUT:", thrust_timeout, "mS"
	print "0x05 - THRUST_REVERSE_TIME:", thrust_reverse_time, "mS"
	print "0x07 - THRUST_REGISTER:", hex(thrust_register)
	print "0x08 - HV_SETPOINT:", hv_setpoint, "V"
	print "0x0D - STATUS:", hex(status)
	readStatus()
	print "0x10 - ADC0:", adc0, "V"
	print "0x11 - ADC1:", adc1, "V"
	print "0x12 - ADC2:", adc2, "V"
	print "0x13 - ADC3:", adc3, "V"
	print "0x14 - ADC4:", adc4, "A"
	print "0x15 - ADC5:", adc5, "V"
	print "0x16 - ADC6:", adc6, "V"
	print "0x17 - ADC7:", adc7, "V"
	print "0x1A - CONTROLLER_GAIN:", hex(controller_gain), "(", controller_gain, ")"
	print "0x1B - HVDAC_LIMIT:", hex(hvdac_limit)
	print "0x1C - HV_SETPOINT_LIMIT:", hv_setpoint, "V"
	print "0x23 - HV_DIFF_SET:", hv_diff_set, "V"
	print "0x26 - AUTO_PERIOD_SET:", auto_period_set, "uS"
	print "0x27 - AUTO_PULSE_WIDTH_SET:", auto_pulse_width_set, "uS"
	print "0x28 - AUTO_PERIOD_READ:", auto_period_read, "uS"
	print "0x29 - AUTO_PULSE_WIDTH_READ:", auto_pulse_width_read, "uS"


def writeThruster(register, value):
	echo = bytearray([0x00, 0x00, 0x00])
	command = bytearray([0x00, 0x00, 0x00])
	command[0] = register << 1
	command[1] = value & 0xFF	#LSB
	command[2] = value >> 8		#MSB

	ser.write(chr(command[0]))	#Register
	ser.write(chr(command[2]))	#MSB
	ser.write(chr(command[1]))	#LSB

	#Wait for response
	i = 0
	while (ser.inWaiting() < 3):
		i = i+1
		if (i % 10 == 0):
			print ".",

	echo[0] = ord(ser.read(1))
	echo[1]	= ord(ser.read(1))
	echo[2]	= ord(ser.read(1))

	if (command == echo):
		return 1 #command received
	else:
		print "Echo failed."
		return 0


def readThruster(register):
	echo = bytearray([0x00, 0x00, 0x00])
	register = (register << 1) | 0x01	#add read bit	

	ser.write(chr(register))

	#Wait for response
	i = 0
	while (ser.inWaiting() < 3):
                i = i+1
               # if (i % 10 == 0):
               #         print ".",

        echo[0] = ord(ser.read(1))
        echo[1] = ord(ser.read(1))
        echo[2] = ord(ser.read(1))

	data_read = echo[1]
	data_read = (data_read << 8) | echo[2]

	if (register == echo[0]):
		return data_read
	else:
		print "Echo failed."
		return 0


def setRegister():
	register = int(raw_input("Register: "),16)
	value = int(raw_input("Value: "),16)
	success = writeThruster(register, value)

	if (success == 0):
		print "Failed to set Register -", register

def readRegister():
	register = int(raw_input("Register: "),16)
        data = readThruster(register)
	print  "Reg", hex(register), "=", hex(data), "(", data, ")"
	
	return data	

def set2Serial():
	success = writeThruster(SERIAL_PORT_FORCE, 0xcafe)
        if (success == 1):
                print "Now Serial Mode"


def readStatus():
	#Shift through bits in "status" and interpret result
	status = readThruster(0x0D)
	status_bits[0] = status & 0x01	#Reserved
	status_bits[1] = (status >> 1) & 0x01	#batch executing (0=rdy)
	status_bits[2] = (status >> 2) & 0x01	#unused
	status_bits[3] = (status >> 3) & 0x01	#unused
	status_bits[4] = (status >> 4) & 0x01	#unused
	status_bits[5] = (status >> 5) & 0x01	#unused
	status_bits[6] = (status >> 6) & 0x01	#HV_Diff
	status_bits[7] = (status >> 7) & 0x01	#unused
	status_bits[8] = (status >> 8) & 0x01	#HV_Over
	status_bits[9] = (status >> 9) & 0x01	#HV_Under
	status_bits[10] = (status >> 10) & 0x01	#I_Limit
	status_bits[11] = (status >> 11) & 0x01	#Limits_OK (1=OK)
	status_bits[12] = (status >> 12) & 0x01	#reserved
	status_bits[13] = (status >> 13) & 0x01	#reserved
	status_bits[14] = (status >> 14) & 0x01	#??
	status_bits[15] = (status >> 15) & 0x01	#??

	print "Status bits (0-15):"
	print "    ",
	for i in range(0,15):
		print status_bits[i],
	print ""

	if (status_bits[11] == 0):
		print "    WARNING: Nearing Limits!"
		if (status_bits[6] == 1):
			print "        HV_Diff at limit! SHUTDOWN"
		if (status_bits[8] == 1):
			print "        HV_Over at limit! SHUTDOWN"
		if (status_bits[9] == 1):
                        print "        HV_Under at limit! SHUTDOWN"
		if (status_bits[10] == 1):
                        print "        I_Limit at limit! SHUTDOWN"


def saveData(time_step_):
	setThrusterData()	#Read all data registers
	with open('Thruster.dat', 'a') as file_:
		file_.write('%-12s %-12s %-12s %-12s %-12s %-12s %-12s %-12s %-12s %-12s %-12s %-12s %-12s %s\n' \
			% (str(time_step_/10.0), \
			str(hvdac), \
			str(thrust_gap_time), \
			str(thrust_timeout), \
			str(thrust_reverse_time), \
			str(adc0), \
			str(adc1), \
			str(adc2), \
			str(adc3), \
			str(adc4), \
			str(adc5), \
			str(adc6), \
			str(adc7), \
			str(controller_gain)))


### Set() Functions ###
def setHVDAC():
	hvdac = readThruster(HVDAC)

def setThrustGapTime():
	thrust_gap_time = readThruster(THRUST_GAP_TIME)

def setThrustTimeout():
	thrust_timeout = readThruster(THRUST_TIMEOUT)

def setThrustReverseTime():
	thrust_reverse_time = readThruster(THRUST_REVERSE_TIME)

def setADC0(value):
	global adc0 
	adc0 = value*(4.096/4096)

def setADC1(value):
    global adc1 
	adc1 = value*(4.096/4096) * 455.55

def setADC2(value):
    global adc2 
	adc2 = value*(4.096/4096) * 454.55

def setADC3(value):
    global adc3 
	adc3 = value*(4.096/4096) * 3.564

def setADC4(value):
    global adc4 
	adc4 = value*(4.096/4096)

def setADC5(value):
    global adc5 
	adc5 = value*(4.096/4096) * 2

def setADC6(value):
    global adc6 
	adc6 = (value*(4.096/4096) - 0.5) * 100

def setADC7(value):
    global adc7 
	adc7 = (value*(4.096/4096) - 0.5) * 100
	
def setControllerGain():
	global controller_gain
	controller_gain = readThruster(CONTROLLER_GAIN)

def setThrusterData():
	# Since I didn't want to pass in all the variable to the function
	# they have to be declared as global otherwise Python interprets 
	# that I want to initialize local variables.  This was only a test 
	# script.  I realize there are better and more robust ways to
	# accomplish the same thing.
	global setup
	global hvdac
	global thrust_gap_time
	global thrust_timeout
	global thrust_reverse_time
	global thrust_register
	global hv_setpoint
	global status
	global adc0
	global adc1
	global adc2
	global adc3
	global adc4
	global adc5
	global adc6
	global adc7
	global controller_gain
	global hvdac_limit
	global hv_setpoint_limit
	global hv_diff_set
	global auto_period_set
	global auto_pulse_width_set
	global auto_period_read
	global auto_pulse_width_read

	setup = readThruster(SETUP)
	hvdac = readThruster(HVDAC)
	thrust_gap_time = readThruster(THRUST_GAP_TIME)
	thrust_timeout = readThruster(THRUST_TIMEOUT)
	thrust_reverse_time = readThruster(THRUST_REVERSE_TIME)
	thrust_register = readThruster(THRUST_REGISTER)
	hv_setpoint = readThruster(HV_SETPOINT)
	status = readThruster(STATUS)
	adc0 = readThruster(ADC0)*(4.096/4096)
	adc1 = readThruster(ADC1)*(4.096/4096)*455.55
	adc2 = readThruster(ADC2)*(4.096/4096)*454.55
	adc3 = readThruster(ADC3)*(4.096/4096)*3.564
	adc4 = readThruster(ADC4)*(4.096/4096)
	adc5 = readThruster(ADC5)*(4.096/4096)*2
	adc6 = (readThruster(ADC6)*(4.096/4096) - 0.5)*100
	adc7 = (readThruster(ADC7)*(4.096/4096) - 0.5)*100
	controller_gain = readThruster(CONTROLLER_GAIN)
	hvdac_limit = readThruster(HVDAC_LIMIT)
	hv_setpoint_limit = readThruster(HV_SETPOINT_LIMIT)
	hv_diff_set = readThruster(HV_DIFF_SET)
	auto_period_set = readThruster(auto_period_set)
	auto_pulse_width_set = readThruster(AUTO_PULSE_WIDTH_SET)
	auto_period_read = readThruster(AUTO_PERIOD_READ)
	auto_pulse_width_read = readThruster(AUTO_PULSE_WIDTH_READ)


### Get() Functions ###
def getHVDAC():
    return hvdac

def getThrustGapTime():
    return thrust_gap_time

def getThrustTimeout():
    return thrust_timeout

def getThrustReverseTime():
    return thrust_reverse_time

def setADC0(value):
    global adc0
    adc0 = value*(4.096/4096)

def getADC0():
	return adc0

def getADC1():
	return adc1

def getADC2():
	return adc2

def getADC3():
	return adc3

def getADC4():
	return adc4

def getADC5():
	return adc5

def getADC6():
	return adc6

def getADC7():
	return adc7

def getControllerGain():
	return controller_gain


	
###### Main Program #####	
if __name__ == "__main__":
    main()





