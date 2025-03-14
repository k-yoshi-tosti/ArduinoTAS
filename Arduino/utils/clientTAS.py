#!/usr/bin/env python3
import serial
import select
import struct
import sys
import io
import time
import math
from pynput.keyboard import Key,Listener
from pynput import keyboard

PORT = "/dev/ttyUSB0"

STATE_OUT_OF_SYNC   = 0
STATE_SYNC_START    = 1
STATE_SYNC_1        = 2
STATE_SYNC_2        = 3
STATE_SYNC_OK       = 4

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

LSTICK_CENTER    = 0x0000000000000000
LSTICK_R         = 0x00000000FF000000 #   0 (000)
LSTICK_U_R       = 0x0000002DFF000000 #  45 (02D)
LSTICK_U         = 0x0000005AFF000000 #  90 (05A)
LSTICK_U_L       = 0x00000087FF000000 # 135 (087)
LSTICK_L         = 0x000000B4FF000000 # 180 (0B4)
LSTICK_D_L       = 0x000000E1FF000000 # 225 (0E1)
LSTICK_D         = 0x0000010EFF000000 # 270 (10E)
LSTICK_D_R       = 0x0000013BFF000000 # 315 (13B)

RSTICK_CENTER    = 0x0000000000000000
RSTICK_R         = 0x000FF00000000000 #   0 (000)
RSTICK_U_R       = 0x02DFF00000000000 #  45 (02D)
RSTICK_U         = 0x05AFF00000000000 #  90 (05A)
RSTICK_U_L       = 0x087FF00000000000 # 135 (087)
RSTICK_L         = 0x0B4FF00000000000 # 180 (0B4)
RSTICK_D_L       = 0x0E1FF00000000000 # 225 (0E1)
RSTICK_D         = 0x10EFF00000000000 # 270 (10E)
RSTICK_D_R       = 0x13BFF00000000000 # 315 (13B)

NO_INPUT       = BTN_NONE + DPAD_CENTER + LSTICK_CENTER + RSTICK_CENTER

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

# Compute x and y based on angle and intensity
def angle(angle, intensity):
    # y is negative because on the Y input, UP = 0 and DOWN = 255
    x =  int((math.cos(math.radians(angle)) * 0x7F) * intensity / 0xFF) + 0x80
    y = -int((math.sin(math.radians(angle)) * 0x7F) * intensity / 0xFF) + 0x80
    return x, y

def lstick_angle(angle, intensity):
    return (intensity + (angle << 8)) << 24

def rstick_angle(angle, intensity):
    return (intensity + (angle << 8)) << 44

# Precision wait
def p_waitNew(waitTime,t0):
    t1 = t0
    while (t1 - t0 < waitTime):
        t1 = time.perf_counter()

def p_wait(waitTime):
    p_waitNew(waitTime,time.perf_counter())
    
# Wait for data to be available on the serial port
def wait_for_data(timeout = 1.0, sleepTime = 0.1):
    t0 = time.perf_counter()
    t1 = t0
    inWaiting = ser.in_waiting
    while ((t1 - t0 < sleepTime) or (inWaiting == 0)):
        time.sleep(sleepTime)
        inWaiting = ser.in_waiting
        t1 = time.perf_counter()

# Read X bytes from the serial port (returns list)
def read_bytes(size):
    bytes_in = ser.read(size)
    return list(bytes_in)

# Read 1 byte from the serial port (returns int)
def read_byte():
    bytes_in = read_bytes(1)
    if len(bytes_in) != 0:
        byte_in = bytes_in[0]
    else:
        byte_in = 0
    return byte_in

# Discard all incoming bytes and read the last (latest) (returns int)
def read_byte_latest():
    inWaiting = ser.in_waiting
    if inWaiting == 0:
        inWaiting = 1
    bytes_in = read_bytes(inWaiting)
    if len(bytes_in) != 0:
        byte_in = bytes_in[0]
    else:
        byte_in = 0
    return byte_in

# Write bytes to the serial port
def write_bytes(bytes_out):
    ser.write(bytearray(bytes_out))
    return

# Write byte to the serial port
def write_byte(byte_out):
    write_bytes([byte_out])
    return

# Compute CRC8
# https://www.microchip.com/webdoc/AVRLibcReferenceManual/group__util__crc_1gab27eaaef6d7fd096bd7d57bf3f9ba083.html
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

# Send a raw packet and wait for a response (CRC will be added automatically)
def send_packet(packet=[0x00,0x00,0x08,0x80,0x80,0x80,0x80,0x00], debug=False):
    if not debug:
        bytes_out = []
        bytes_out.extend(packet)

        # Compute CRC
        crc = 0
        for d in packet:
            crc = crc8_ccitt(crc, d)
        bytes_out.append(crc)
        write_bytes(bytes_out)
        # print(bytes_out)

        # Wait for USB ACK or UPDATE NACK
        byte_in = read_byte()
        while(byte_in==0x38):
            byte_in = read_byte();
        commandSuccess = (byte_in == RESP_USB_ACK)
    else:
        commandSuccess = True
    return commandSuccess

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

# Send a formatted controller command to the MCU
def send_cmd(command,left_x,left_y,right_x,right_y):
    commandSuccess = send_packet(cmd_to_packet(command,left_x,left_y,right_x,right_y))
    return commandSuccess


#Test all buttons except for home and capture
def testbench_btn():
    send_cmd(BTN_A) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)
    send_cmd(BTN_B) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)
    send_cmd(BTN_X) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)
    send_cmd(BTN_Y) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)
    send_cmd(BTN_PLUS) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)
    send_cmd(BTN_MINUS) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)
    send_cmd(BTN_LCLICK) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)
    send_cmd(BTN_RCLICK) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)

# Test DPAD U / R / D / L
def testbench_dpad():
    send_cmd(DPAD_U) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)
    send_cmd(DPAD_R) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)
    send_cmd(DPAD_D) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)
    send_cmd(DPAD_L) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)

# Test DPAD Diagonals - Does not register on switch due to dpad buttons
def testbench_dpad_diag():
    send_cmd(DPAD_U_R) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)
    send_cmd(DPAD_D_R) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)
    send_cmd(DPAD_D_L) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)
    send_cmd(DPAD_U_L) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)

# Test Left Analog Stick
def testbench_lstick():
    #Test U/R/D/L
    send_cmd(BTN_LCLICK) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)
    send_cmd(LSTICK_U) ; p_wait(0.5)
    send_cmd(LSTICK_R) ; p_wait(0.5)
    send_cmd(LSTICK_D) ; p_wait(0.5)
    send_cmd(LSTICK_L) ; p_wait(0.5)
    send_cmd(LSTICK_U) ; p_wait(0.5)
    send_cmd(LSTICK_CENTER) ; p_wait(0.5)

    # 360 Circle @ Full Intensity
    for i in range(0,721):
        cmd = lstick_angle(i + 90, 0xFF)
        send_cmd(cmd)
        p_wait(0.001)
    send_cmd(LSTICK_CENTER) ; p_wait(0.5)

    # 360 Circle @ Partial Intensity
    for i in range(0,721):
        cmd = lstick_angle(i + 90, 0x80)
        send_cmd(cmd)
        p_wait(0.001)
    send_cmd(LSTICK_CENTER) ; p_wait(0.5)

# Test Right Analog Stick
def testbench_rstick():
    #Test U/R/D/L
    send_cmd(BTN_RCLICK) ; p_wait(0.5) ; send_cmd() ; p_wait(0.001)
    send_cmd(RSTICK_U) ; p_wait(0.5)
    send_cmd(RSTICK_R) ; p_wait(0.5)
    send_cmd(RSTICK_D) ; p_wait(0.5)
    send_cmd(RSTICK_L) ; p_wait(0.5)
    send_cmd(RSTICK_U) ; p_wait(0.5)
    send_cmd(RSTICK_CENTER) ; p_wait(0.5)

    # 360 Circle @ Full Intensity
    for i in range(0,721):
        cmd = rstick_angle(i + 90, 0xFF)
        send_cmd(cmd)
        p_wait(0.001)
    send_cmd(RSTICK_CENTER) ; p_wait(0.5)

    # 360 Circle @ Partial Intensity
    for i in range(0,721):
        cmd = rstick_angle(i + 90, 0x80)
        send_cmd(cmd)
        p_wait(0.001)
    send_cmd(RSTICK_CENTER) ; p_wait(0.5)

# Test Packet Speed
def testbench_packet_speed(count=100, debug=False):
    sum = 0
    min = 999
    max = 0
    avg = 0
    err = 0

    for i in range(0, count + 1):

        # Send packet and check time
        t0 = time.perf_counter()
        status = send_packet()
        t1 = time.perf_counter()

        # Count errors
        if not status:
            err += 1
            print('Packet Error!')

        # Compute times
        delta = t1 - t0
        if delta < min:
            min = delta
        if delta > max:
            max = delta
        sum = sum + (t1 - t0)

    avg = sum / i
    print('Min =', '{:.3f}'.format(min), 'Max =', '{:.3}'.format(max), 'Avg =', '{:.3f}'.format(avg), 'Errors =', err)

def testbench():
    testbench_btn()
    testbench_dpad()
    testbench_lstick()
    testbench_rstick()
    testbench_packet_speed()
    return

# Force MCU to sync
def force_sync():
    # Send 9x 0xFF's to fully flush out buffer on device
    # Device will send back 0xFF (RESP_SYNC_START) when it is ready to sync
    write_bytes([0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF])

    # Wait for serial data and read the last byte sent
    wait_for_data()
    byte_in = read_byte_latest()

    print(byte_in)
    while(byte_in==0x00) or (byte_in==0x88):
        print("INTERRUPTED1")
        write_bytes([0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF])
        byte_in = read_byte()
    print(byte_in)

    # Begin sync...
    inSync = False
    if byte_in == RESP_SYNC_START:
        write_byte(COMMAND_SYNC_1)
        byte_in = read_byte()
        while(byte_in==0x55) or (byte_in==0x88) or (byte_in==0xFF):
            print("INTERRUPTED2")
            byte_in = read_byte()
        print(byte_in);
        print("START")
        if byte_in == RESP_SYNC_1:
            write_byte(COMMAND_SYNC_2)
            byte_in = read_byte()
            while(byte_in==0x55):
                print("INTERRUPTED3")
                byte_in = read_byte()
            print(byte_in)
            if byte_in == RESP_SYNC_OK:
                write_byte(COMMAND_SYNC_DONE)
                inSync = True
    return inSync

# Start MCU syncing process
def sync():
    inSync = False

    # Try sending a packet
    #inSync = send_packet()
    if not inSync:
        print("forceSync")
        # Not in sync: force resync and send a packet
        inSync = force_sync()
        if inSync:
            print("forceSyncDONE")
            inSync = send_packet()
    return inSync

# -------------------------------------------------------------------------

def pressKeys(buttons):
    simpleKeys = buttons.split(";")
    command = 0x0
    for key in simpleKeys:
        if(key == "KEY_A"):
            command += BTN_A
        elif(key == "KEY_B"):
            command += BTN_B
        elif(key == "KEY_X"):
            command += BTN_X
        elif(key == "KEY_Y"):
            command += BTN_Y
        elif(key == "KEY_PLUS"):
            command += BTN_PLUS
        elif(key == "KEY_MINUS"):
            command += BTN_MINUS
        elif(key == "KEY_HOME"):
            command += BTN_HOME
        elif(key == "KEY_CAPTURE"):
            command += BTN_CAPTURE
        elif(key == "KEY_ZL"):
            command += BTN_ZL
        elif(key == "KEY_ZR"):
            command += BTN_ZR
        elif(key == "KEY_L"):
            command += BTN_L
        elif(key == "KEY_R"):
            command += BTN_R
        elif(key == "KEY_LSTICK"):
            command += BTN_LCLICK
        elif(key == "KEY_RSTICK"):
            command += BTN_RCLICK
        elif(key == "KEY_DLEFT"):
            command += DPAD_L
        elif(key == "KEY_DRIGHT"):
            command += DPAD_R
        elif(key == "KEY_DUP"):
            command += DPAD_U
        elif(key == "KEY_DDOWN"):
            command += DPAD_D

        elif(key == "KEY_SYNC"):
            command += BTN_L
            command += BTN_R
        elif(key != "NONE") and (key != "") and (key != " "):
            print("Unknown key:",key)
    return command

def toValidJoy(value):
    return 128-int(int(value)/256)

def toValidJox(value):
    return int(int(value)/256)+128

def revertJoy(value):
    return (128-value)*256

def revertJox(value):
    return (value-128)*256




current = set()

def on_press(key):
    current.add(key)

def on_release(key):
    if key in current:
        current.remove(key)
    if key == Key.esc:
        # Stop listener
        return False

# Collect events until released
listener = Listener(
        on_press=on_press,
        on_release=on_release)
listener.start()
def main():
    # Attempt to sync with the MCU
    if not sync():
        print('Could not sync!')
        return

    print("READY")
    notDone = True
    frameNo = 0
    record = False
    while notDone:
        start = time.perf_counter()
        command = NO_INPUT
        keys = ""
        lx = 128
        ly = 128
        rx = 128
        ry = 128
        for key in current:
            if(key == keyboard.KeyCode(char='l')):
                command += BTN_A
                keys+=";KEY_A"
            if(key == keyboard.KeyCode(char='k')):
                command += BTN_B
                keys+=";KEY_B"
            if(key == keyboard.KeyCode(char='j')):
                command += BTN_X
                keys+=";KEY_X"
            if(key == keyboard.KeyCode(char='i')):
                command += BTN_Y
                keys+=";KEY_Y"
            if(key == keyboard.KeyCode(char='3')):
                command += BTN_PLUS
                keys+=";KEY_PLUS"
            if(key == keyboard.KeyCode(char='1')):
                command += BTN_MINUS
                keys+=";KEY_MINUS"
            if(key == keyboard.KeyCode(char='h')):
                command += BTN_HOME
                keys+=";KEY_HOME"
            if(key == keyboard.KeyCode(char='c')):
                command += BTN_CAPTURE
                keys+=";KEY_CAPTURE"
            if(key == keyboard.KeyCode(char='q')):
                command += BTN_ZL
                keys+=";KEY_ZL"
            if(key == keyboard.KeyCode(char='u')):
                command += BTN_ZR
                keys+=";KEY_ZR"
            if(key == keyboard.KeyCode(char='e')):
                command += BTN_L
                keys+=";KEY_L"
            if(key == keyboard.KeyCode(char='o')):
                command += BTN_R
                keys+=";KEY_R"
            if(key == keyboard.KeyCode(char='x')):
                command += BTN_LCLICK
                keys+=";KEY_LSTICK"
            if(key == keyboard.KeyCode(char=',')):
                command += BTN_RCLICK
                keys+=";KEY_RSTICK"
            if(key == Key.left):
                command += DPAD_L
                keys+=";KEY_DLEFT"
            if(key == Key.right):
                command += DPAD_R
                keys+=";KEY_DRIGHT"
            if(key == Key.up):
                command += DPAD_U
                keys+=";KEY_DUP"
            if(key == Key.down):
                command += DPAD_D
                keys+=";KEY_DDOWN"

                
            if(key == keyboard.KeyCode(char='r')):
                record=True
                f=open("newScript0.txt","a+")
                
            if Key.esc in current:
                notDone=False
                break

            if(key == keyboard.KeyCode(char='t')):
                command += BTN_L
                command += BTN_R

            if(key == keyboard.KeyCode(char='w')):
                ly-=127
            if(key == keyboard.KeyCode(char='a')):
                lx-=127
            if(key == keyboard.KeyCode(char='s')):
                ly+=127
            if(key == keyboard.KeyCode(char='d')):
                lx+=127
            
        send_cmd(command,lx,ly,rx,ry)
        if(record):
            frameNo+=1
            if(keys == ""):
                keys = "NONE"
            else:
                keys = keys[1:]
            if(keys == "NONE") and (lx+ly+rx+ry == 128*4):
                print("EMPTY")
            else:
                f.write("%d %s %d;%d %d;%d\n" % (frameNo,keys,revertJox(lx),revertJoy(ly),revertJox(rx),revertJoy(ry)))
                
            
        p_waitNew(1/60,start)

    if 'f' in locals():
        f.close()
    frameNo = -3
    f = open("script1.txt","r")
    f1 = f.readlines();
    last = time.perf_counter()

    for line in f1:
        parts = line.split(" ")
        while frameNo != int(parts[0]):
            startTime = time.perf_counter()
            send_cmd(pressKeys(""),128,128,128,128)
            byte_in = read_byte();
            while(byte_in!=0x38):
                byte_in = read_byte();
            frameNo+=1

        #print(frameNo)

        com = pressKeys(parts[1])
        left = parts[2].split(";")
        right = parts[3].split(";")
        send_cmd(com,toValidJox(left[0]),toValidJoy(left[1]),toValidJox(right[0]),toValidJoy(right[1]))
    
        byte_in = read_byte();
        nb = 0
        while(byte_in!=0x38):
            byte_in = read_byte(); #   1/120
            nb += 1

        print (frameNo, nb, time.perf_counter() - last)
        last = time.perf_counter()
        frameNo+=1

    send_cmd(pressKeys(""),128,128,128,128)
    # testbench()
    # testbench_packet_speed(1000)


def countVSyncs():
    if not sync():
        print('Could not sync!')
        sys.exit()
    start = time.perf_counter();
    counter=0
    while(True):
        if(time.perf_counter() >= start+10):
            start=time.perf_counter();
            print(counter);
            counter=0
        byte_in = read_byte()
        if(byte_in==0x38):
            counter+=1
    

# ser = serial.Serial(port=args.port, baudrate=31250,timeout=1)
# ser = serial.Serial(port=args.port, baudrate=40000,timeout=1)
# ser = serial.Serial(port=args.port, baudrate=62500,timeout=1)
ser = serial.Serial(port=PORT, baudrate=19200,timeout=1)
main()
#countVSyncs()
ser.close()
