from tkinter import *
from tkinter.messagebox import *
from tkinter.filedialog import *

import serial
import time
import threading

PORT = "/dev/ttyUSB0"
# on windows, it should be something like "COM$" where $ is a number, check your device manager and find the USB-UART bridge to know the correct port
# on linux, I don't really know - for me it was always /dev/ttyUSB0 or /dev/ttyUSB1
# if the port isn't correct, you'll get an error in the python console looking like "serial.serialutil.SerialException: [Errno 2] could not open port <port>: [Errno 2] No such file or directory: '<port>'"

# Commands to send to MCU
COMMAND_NOP        = 0x00
COMMAND_SYNC_1     = 0x33
COMMAND_SYNC_2     = 0xCC
COMMAND_SYNC_START = 0xFF
COMMAND_SYNC_DONE  = 0x77

# Responses from MCU
RESP_USB_ACK       = 0x90
RESP_UPDATE_ACK    = 0x91
RESP_UPDATE_NACK   = 0x92
RESP_SYNC_START    = 0xFF
RESP_SYNC_1        = 0xCC
RESP_SYNC_OK       = 0x33

# Actual Switch DPAD Values
A_DPAD_CENTER    = 0x08
A_DPAD_U         = 0x00
A_DPAD_U_R       = 0x01
A_DPAD_R         = 0x02
A_DPAD_D_R       = 0x03
A_DPAD_D         = 0x04
A_DPAD_D_L       = 0x05
A_DPAD_L         = 0x06
A_DPAD_U_L       = 0x07

# Enum DIR Values
DIR_CENTER    = 0x00
DIR_U         = 0x01
DIR_R         = 0x02
DIR_D         = 0x04
DIR_L         = 0x08
DIR_U_R       = DIR_U + DIR_R
DIR_D_R       = DIR_D + DIR_R
DIR_U_L       = DIR_U + DIR_L
DIR_D_L       = DIR_D + DIR_L

BTN_NONE         = 0x0000000000000000
BTN_Y            = 0x0000000000000001
BTN_B            = 0x0000000000000002
BTN_A            = 0x0000000000000004
BTN_X            = 0x0000000000000008
BTN_L            = 0x0000000000000010
BTN_R            = 0x0000000000000020
BTN_ZL           = 0x0000000000000040
BTN_ZR           = 0x0000000000000080
BTN_MINUS        = 0x0000000000000100
BTN_PLUS         = 0x0000000000000200
BTN_LCLICK       = 0x0000000000000400
BTN_RCLICK       = 0x0000000000000800
BTN_HOME         = 0x0000000000001000
BTN_CAPTURE      = 0x0000000000002000

DPAD_CENTER      = 0x0000000000000000
DPAD_U           = 0x0000000000010000
DPAD_R           = 0x0000000000020000
DPAD_D           = 0x0000000000040000
DPAD_L           = 0x0000000000080000
DPAD_U_R         = DPAD_U + DPAD_R
DPAD_D_R         = DPAD_D + DPAD_R
DPAD_U_L         = DPAD_U + DPAD_L
DPAD_D_L         = DPAD_D + DPAD_L


keys = {"KEY_A": BTN_A, "KEY_B": BTN_B, "KEY_X": BTN_X, "KEY_Y": BTN_Y, "KEY_PLUS": BTN_PLUS, "KEY_MINUS": BTN_MINUS, "KEY_HOME": BTN_HOME, "KEY_CAPTURE": BTN_CAPTURE,
		"KEY_ZL": BTN_ZL, "KEY_ZR": BTN_ZR, "KEY_L": BTN_L, "KEY_R": BTN_R, "KEY_LSTICK": BTN_LCLICK, "KEY_RSTICK": BTN_RCLICK,
		"KEY_DLEFT": DPAD_L, "KEY_DRIGHT": DPAD_R, "KEY_DUP": DPAD_U, "KEY_DDOWN": DPAD_D}

# Convert DPAD value to actual DPAD value used by Switch
def decrypt_dpad(dpad):
	if dpad == DIR_U:
		dpadDecrypt = A_DPAD_U
	elif dpad == DIR_R:
		dpadDecrypt = A_DPAD_R
	elif dpad == DIR_D:
		dpadDecrypt = A_DPAD_D
	elif dpad == DIR_L:
		dpadDecrypt = A_DPAD_L
	elif dpad == DIR_U_R:
		dpadDecrypt = A_DPAD_U_R
	elif dpad == DIR_U_L:
		dpadDecrypt = A_DPAD_U_L
	elif dpad == DIR_D_R:
		dpadDecrypt = A_DPAD_D_R
	elif dpad == DIR_D_L:
		dpadDecrypt = A_DPAD_D_L
	else:
		dpadDecrypt = A_DPAD_CENTER
	return dpadDecrypt

	
def toValidJoy(value):
	return 128-int(int(value)/256)

def toValidJox(value):
	return int(int(value)/256)+128

def revertJoy(value):
	return (128-value)*256

def revertJox(value):
	return (value-128)*256


# Convert CMD to a packet
def cmd_to_packet(command,left_x,left_y,right_x,right_y):
	cmdCopy = command
	low              =  (cmdCopy & 0xFF)  ; cmdCopy = cmdCopy >>  8
	high             =  (cmdCopy & 0xFF)  ; cmdCopy = cmdCopy >>  8
	dpad             =  (cmdCopy & 0xFF)  ; cmdCopy = cmdCopy >>  8

	dpad = decrypt_dpad(dpad)
	packet = [high, low, dpad, left_x, left_y, right_x, right_y, 0x00]
	# print (hex(command), packet, lstick_angle, lstick_intensity, rstick_angle, rstick_intensity)
	return packet

def crc8_ccitt(old_crc, new_data):
	data = old_crc ^ new_data

	for i in range(8):
		if (data & 0x80) != 0:
			data = data << 1
			data = data ^ 0x07
		else:
			data = data << 1
		data = data & 0xff
	return data

class ArduinoSerial:
	callbackMinElapsedFrames = 20		# the callback function will be called every 20 frames to prevent lag
	def __init__(self, port):
		self.ser = serial.Serial(port = port, baudrate = 19200, timeout = 1)

		if not self.ser.is_open:
			self.ser.open()

		self.force_stop = False

	def sync(self, callback = None):
		print ("syncing...")
		self.write_bytes([0xFF] * 9)

		self.wait_for_data()
		byte_in = self.read_byte_latest()
		print (byte_in)
		
		while not self.force_stop and (byte_in == 0x00 or byte_in == 0x88):
			print ("INTERRUPTED1")
			self.write_bytes([0xFF] * 9)
			byte_in = self.read_byte()

		print (byte_in)

		if byte_in == RESP_SYNC_START:
			self.write_byte(COMMAND_SYNC_1)
			byte_in = self.read_byte()
			while not self.force_stop and byte_in in (0x55, 0x88, 0xFF):
				print ("INTERRUPTED2")
				byte_in = self.read_byte()

			print (byte_in)

			if self.force_stop:
				self.force_stop = False
				if callback:
					return callback(False)
				else:
					return False

			print ("START")

			if byte_in == RESP_SYNC_1:
				self.write_byte(COMMAND_SYNC_2)
				byte_in = self.read_byte()
				while not self.force_stop and byte_in == 0x55:
					print ("INTERRUPTED3")
					byte_in = self.read_byte()
				print (byte_in)

				if not self.force_stop and byte_in == RESP_SYNC_OK:
					self.write_byte(COMMAND_SYNC_DONE)
					in_sync = self.send_packet()
					if callback:
						return callback(True)
					else:
						return True

		self.force_stop = False

		if callback:
			return callback(False)
		else:
			return False

	def run_inputs(self, inputs, waitFrames = 5, callback = None):
		frameno = -waitFrames
		
		self.force_stop = False
		last = time.perf_counter()
		self.discard_all()
		#print (self.ser.in_waiting)	# should be 0

		for frame in inputs:
			while frameno < frame[0] and not self.force_stop:
				startTime = time.perf_counter()
				self.send_cmd()
				byte_in = self.read_byte()
				while byte_in != 0x38:
					byte_in = self.read_byte()

				if callback and frameno >= 0 and not frameno % self.callbackMinElapsedFrames:
					callback(frameno)

				frameno += 1

			if self.force_stop:
				self.force_stop = False
				break

			command = 0
			for key in frame[1]:
				if key != "NONE":
					command |= keys[key]

			self.send_cmd(command, toValidJox(frame[2][0]), toValidJoy(frame[2][1]), toValidJox(frame[3][0]), toValidJoy(frame[3][1]))

			byte_in = self.read_byte()
			nb = 0
			while byte_in != 0x38:
				byte_in = self.read_byte()
				nb += 1

			if callback and not frameno % self.callbackMinElapsedFrames:
				callback(frameno)
			frameno += 1

			#print (frameno, nb, time.perf_counter() - last)
			last = time.perf_counter()

		self.send_cmd()
		if callback:
			callback(-1)


	def send_cmd(self, command = 0, left_x = 128, left_y = 128, right_x = 128, right_y = 128):
		success = self.send_packet(cmd_to_packet(command, left_x, left_y, right_x, right_y))
		return success

	def send_packet(self, packet = [0x00,0x00,0x08,0x80,0x80,0x80,0x80,0x00]):
		bytes_out = packet[:]

		crc = 0
		for byte in packet:
			crc = crc8_ccitt(crc, byte)
		bytes_out.append(crc)
		self.write_bytes(bytes_out)

		byte_in = self.read_byte()
		while byte_in == 0x38:
			byte_in = self.read_byte()

		return byte_in == RESP_USB_ACK

	def wait_for_data(self, timout = 1.0, sleepTime = 0.1):
		t1 = t0 = time.perf_counter()
		inWaiting = self.ser.in_waiting
		while not self.force_stop and (t1 - t0 < sleepTime or inWaiting == 0):
			time.sleep(sleepTime)
			inWaiting = self.ser.in_waiting
			t1 = time.perf_counter()

	def read_bytes(self, size):
		bytes_in = self.ser.read(size)
		return list(bytes_in)

	def read_byte(self):
		bytes_in = self.read_bytes(1)
		if len(bytes_in):
			return bytes_in[0]
		return 0

	def read_byte_latest(self):
		inWaiting = self.ser.in_waiting
		if inWaiting == 0:
			inWaiting = 1
		bytes_in = self.read_bytes(inWaiting)
		if len(bytes_in) != 0:
			return bytes_in[0]
		return 0

	def discard_all(self):
		#while self.ser.in_waiting:
		#	self.ser.read(self.ser.in_waiting)
		self.ser.reset_input_buffer()

	def write_bytes(self, bytes_out):
		self.ser.write(bytearray(bytes_out))

	def write_byte(self, byte_out):
		self.write_bytes([byte_out])

	def countvsync(self, callback):
		self.discard_all()

		start = time.perf_counter()
		counter = 0
		while not self.force_stop:
			if time.perf_counter() >= start + 2:
				start = time.perf_counter()
				callback(counter)
				counter = 0

			byte_in = self.read_byte()
			if byte_in == 0x38:
				counter += 1

		self.force_stop = False

	def close(self):
		self.force_stop = True
		time.sleep(0.5)
		self.send_cmd()
		while self.read_byte() != 0x38:
			pass

		self.ser.close()

class Script:
	def __init__(self, filename = '', inputs = []):
		if filename and inputs:
			raise ValueError("Can't generate a script with both a filename and inputs")

		if filename:
			self.inputs = []

			try:
				f = open(filename)
			except FileNotFoundError:
				showerror("Script error", "Failed to open file " + str(filename))
				return

			lines = f.readlines()
			f.close()

			lineno = 0

			for line in lines:
				lineno += 1
				if not line.strip():
					continue

				try:
					frameno, buttons, lstick, rstick = line.split(' ')

					frameno = int(frameno)
					buttons = buttons.split(';')

					lstick = tuple(map(int, lstick.split(';')))
					rstick = tuple(map(int, rstick.split(';')))

				except ValueError:
					if not askokcancel("Script warning", "Malformed script " + str(filename) + " at line " + str(lineno), icon = "warning"):
						return
					continue

				for val in lstick + rstick:
					if val < -32768 or val >= 32768:
						showerror("Script error", "Invalid stick coordinates in " + str(filename) + " at line " + str(lineno))

				for button in buttons:
					if button not in keys and button != "NONE":
						if not askokcancel("Script warning", "Unknown key " + button + " in script " + str(filename) + " at line " + lineno, icon = "warning"):
							return

				self.inputs.append((frameno, buttons, lstick, rstick))

		else:
			self.inputs = inputs

		self.inputs.sort()
		if self.inputs:
			self.nb_frames = self.inputs[-1][0]
		else:
			self.nb_frames = 0

	def shiftFrames(self, shift):
		return Script(inputs = list(map(lambda frame: (frame[0] + shift,) + frame[1:], self.inputs)))

class MainGUI:
	def __init__(self, port):
		self.f = Tk()
		self.f.title("Arduino TAS GUI")

		self.serial = ArduinoSerial(port)

		self.f.protocol("WM_DELETE_WINDOW", self.on_close)

		self.sync_frame = Frame(self.f, borderwidth = 2, relief = "groove")
		self.sync_frame.grid(row = 0, column = 0, padx = 10, pady = 13)

		self.sync_button = Button(self.sync_frame, text = "Sync", command = self.sync)
		self.sync_button.grid(row = 0, column = 0, padx = 7, pady = 5)

		self.sync_label = Label(self.sync_frame, text = "Not synchronized")
		self.sync_label.grid(row = 0, column = 1, padx = 7, pady = 5)

		self.script_frame = Frame(self.f, borderwidth = 2, relief = "groove")
		self.script_frame.grid(row = 1, column = 0, padx = 10, pady = 13)

		self.open_script_button = Button(self.script_frame, text = "Open script", command = self.open_script)
		self.open_script_button.grid(row = 0, column = 0, padx = 7, pady = 5)

		self.run_script_button = Button(self.script_frame, text = "Run script", command = self.run_script)
		self.run_script_button.grid(row = 0, column = 1, padx = 7, pady = 5)

		self.script_filename_label = Label(self.script_frame, text = "No script")
		self.script_filename_label.grid(row = 1, column = 0, columnspan = 2, padx = 6, pady = 10)

		self.script_frame_number_label = Label(self.script_frame, text = "Frame - / -")
		self.script_frame_number_label.grid(row = 2, column = 0, columnspan = 2, padx = 6, pady = 5)

		self.test_vsync_button = Button(self.f, text = "Test V-sync", command = self.test_vsync)
		self.test_vsync_button.grid(row = 2, column = 0, padx = 10, pady = 13)

		self.script = Script()

		self.is_running = False

		self.synced = False
		self.is_syncing = False

		self.is_vsync_testing = False

		self.f.mainloop()

	def sync(self):
		if self.is_running:
			return showerror("Error", "Already running a script")
		if self.is_vsync_testing:
			return showerror("Error", "Testing v-sync")

		if not self.is_syncing:
			sync_thread = threading.Thread(target = self.serial.sync, args = (self.sync_finished,))
			sync_thread.start()
			self.sync_label.config(text = "Syncing...")
			self.is_syncing = True

			self.sync_button.config(text = "Stop sync")
		else:
			self.serial.force_stop = True
			self.sync_label.config(text = "Sync failed")
			self.is_syncing = False
 
			self.sync_button.config(text = "Sync")

	def sync_finished(self, state):
		self.sync_label.config(text = ("Sync failed", "Synchronized")[state])
		self.is_syncing = False
		self.synced = state
		self.sync_button.config(text = "Sync")

	def on_close(self):
		self.serial.close()
		self.f.destroy()

	def test_vsync(self):
		if self.is_syncing or not self.synced:
			return showerror("Error", "Not synced")

		if self.is_running:
			return showerror("Error", "Already running a script")

		if self.is_vsync_testing:
			self.serial.force_stop = True
			self.vsync_frame.destroy()
			self.is_vsync_testing = False

			self.test_vsync_button.config(text = "Test V-sync")
			return

		self.test_vsync_button.config(text = "Stop test")
		self.is_vsync_testing = True

		self.vsync_frame = Frame(self.f)
		self.vsync_frame.grid(row = 0, column = 1, rowspan = 3, padx = 10, pady = 13, sticky = "EW")

		self.vsync_entry = Text(self.vsync_frame, width = 50, height = 16)
		self.vsync_entry.grid(row = 0, column = 0, sticky = "NSEW")
		self.vsync_entry.insert(END, "These values should be around 120\n(values between 117-123 should work)\nand they should be printed every two seconds.\nIf not, check your setup and\nespecially the connection to the VGA port.\n")
		
		vsync_scroll = Scrollbar(self.vsync_frame, orient = VERTICAL,
			command = lambda op, val, units = None: self.vsync_entry.yview_scroll(val, units) if op == "scroll" else self.vsync_entry.yview_moveto(val))
		vsync_scroll.grid(row = 0, column = 1, sticky = "NS")

		self.vsync_entry.config(yscrollcommand = vsync_scroll.set, state = DISABLED)

		vsync_thread = threading.Thread(target = self.serial.countvsync, args = (self.vsync_update,))
		vsync_thread.start()

	def vsync_update(self, val):
		self.vsync_entry.config(state = NORMAL)
		self.vsync_entry.insert(END, str(val) + '\n')
		self.vsync_entry.config(state = DISABLED)

	def run_script(self):
		if self.is_syncing or not self.synced:
			return showerror("Error", "Not synced")
		if self.is_vsync_testing:
			return showerror("Error", "Testing v-sync")

		if not self.is_running:
			run_thread = threading.Thread(target = self.serial.run_inputs, args = (self.script.inputs, 5, self.run_update))
			run_thread.start()
			self.is_running = True

			self.run_script_button.config(text = "Stop script")
		else:
			self.serial.force_stop = True
			self.script_frame_number_label.config(text = "Frame - / " + str(self.script.nb_frames))
			self.is_running = False

			self.run_script_button.config(text = "Run script")

	def run_update(self, frameno):
		if frameno == -1:
			self.script_frame_number_label.config(text = "Frame - / " + str(self.script.nb_frames))
			self.is_running = False

			self.run_script_button.config(text = "Run script")
		else:
			self.script_frame_number_label.config(text = "Frame " + str(frameno) + " / " + str(self.script.nb_frames))

	def open_script(self):
		filename = askopenfilename(title = "Select a script", filetypes = [("Text file", "*.txt"), ("All files", '*')])
		self.script = Script(filename)
		if self.script.inputs:
			self.script_filename_label.config(text = "Loaded script: " + filename)
			self.script_frame_number_label.config(text = "Frame - / " + str(self.script.nb_frames))

def countVSyncs():
	p = time.perf_counter()
	ser = ArduinoSerial(PORT)

	if not ser.sync():
		print('Could not sync!')
		sys.exit()
	start = time.perf_counter();
	counter=0
	while time.perf_counter() - p < 30:
		if(time.perf_counter() >= start+2):
			start=time.perf_counter();
			print(counter);
			counter=0
		byte_in = ser.read_byte()
		if(byte_in==0x38):
			counter+=1
	

if __name__ == "__main__":
	m = MainGUI(PORT)
	#countVSyncs()
