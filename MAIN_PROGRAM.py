from time import sleep
import time, threading, socket, os, sys, select, multiprocessing
import RPi.GPIO as GPIO

# PIN
ENABLE = [36, 24, 16] # Stepper [X, Y, Z]
DIR = [38, 26, 18] # Stepper [X, Y, Z]
STEP = [40, 32, 22] # Stepper [X, Y, Z]
LimitX = [23,21] # Limit X [min, max] active low
LimitY = [19,15] # Limit Y [min, max] active low
LimitZ = [13,11] # Limit Z [min, max] active low
LampuM, LampuK, LampuH = 8, 10, 12 # Lampu Merah active low
Emg = 3 # SW Emergency active low
cw, ccw, en = 33, 35, 37 # Spindle
inputA = 31 # Input Encoder

# Var Bresenham
x1, y1, z1 , x2, y2, z2, sb, dx, dy, dz, xs, ys, zs, p1, p2 = 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0

# Var Stepper
stepX, stepY, stepZ = 0,0,0 # step XYZ
enStep, runStep = 0,0 # Enable Stepper
feed00, feedRate, feedRateSet = 0.0001, 0.0001, 0.0001 # Feed Rate
CW, CCW = 0, 1 # Stepper CW, CCW
fr = 0

# PWM
set_point = 4800
error, last_error, sum_error, pid = 0.0, 0.0, 0.0, 0.0
dkp = 0.0015 #6.9 ##5.175
dki = 0.05 #1.104 ##1.381 
dkd = 0 #0.276
kp = dkp
ki = dki
kd = dkd

enDC = True
mState = 0
inRPM = 0.0
outPWM = 0.0
prevT = 0.0

# RPM
rpm, prev_rpm = 0.0, 0.0

#map
in_min = -4500
in_max = 4500
out_min = -99
out_max = 99

# Var komunikasi
lastCmd = "" # last String yg diterima
IP_PORT = 5005 # PORT
isConnected = False # Kondisi koneksi

# Var
allAuto, auto = "", "" # String Auto
manual = "" # String Manual
mode = 0 # Mode A/M
emg, flagEmg = 0, 0 # Emergency
rst = 0 # Reset
increment = 0 # G91 Incremental
line = 0 # Line input
start = 0 # Auto Start
prevTime = 0 # prev Timer
home, homing = 0, 0 # Homing
x1a, y1a, z1a = 0,0,0 # Prev Var XYZ
seq, proses, endogan = 0, 0, 0 # other
ori, oring = 0, 0 # Goto Origin Var
prevTimeHitung, hitung = 0, 0 # Menghitung waktu auto
ucing = 0
prevEmg = 0.0
hayam = 0

class koneksi (threading.Thread):
    def __init__(self, threadID, conn):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.conn = conn
    def run(self):
        global isConnected, lastCmd, conn
        while True:
            cmd = ""
            try:
                cmd = self.conn.recv(1024).strip()
            except:
                break
            if len(cmd) == 0:
                break
            cmd = cmd.decode("utf-8")
            lastCmd = cmd
            print(cmd)
        conn.close()
        print("Client disconnected. Waiting for next client...")
        isConnected = False
        if (isConnected == False):
            conn, addr = serverSocket.accept()
            print("Connected with client at " + addr[0])
            isConnected = True
            if (isConnected == True):
                conn.send(str.encode("CS"))
            thread1 = koneksi(1,conn)
            # necessary to terminate it at program termination:
            thread1.setDaemon(True)  
            thread1.start()

# Setting Koneksi
serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# close port when process exits:
serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
HOSTNAME = "" # Symbolic name meaning all available interfaces
try:
    serverSocket.bind((HOSTNAME, IP_PORT))
except socket.error as msg:
    print("Bind failed")
    sys.exit()

# Menunggu Client konek
serverSocket.listen(10)
print("Waiting for a connecting client...")
conn, addr = serverSocket.accept()
print("Connected with client at " + addr[0])
isConnected = True
if (isConnected == True):
    conn.send(str.encode("CS"))
thread1 = koneksi(1,conn)
thread1.setDaemon(True)  
thread1.start()

def setup():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    for x in range(3):
        # Stepper sebagai output
        GPIO.setup(ENABLE[x], GPIO.OUT)
        GPIO.output(ENABLE[x], GPIO.LOW)
        GPIO.setup(DIR[x], GPIO.OUT)
        GPIO.setup(STEP[x], GPIO.OUT)
        if (x <= 1):
            # Limit switch sebagai input
            GPIO.setup(LimitX[x], GPIO.IN)
            GPIO.setup(LimitY[x], GPIO.IN)
            GPIO.setup(LimitZ[x], GPIO.IN)
    # Lampu sebagai output
    GPIO.setup(LampuM, GPIO.OUT)
    GPIO.setup(LampuK, GPIO.OUT)
    GPIO.setup(LampuH, GPIO.OUT)
    # Emergency sebagai input
    GPIO.setup(Emg, GPIO.IN)
    # State Awal
    GPIO.output(LampuK, GPIO.HIGH)
    GPIO.output(LampuH, GPIO.HIGH)
    GPIO.output(LampuM, GPIO.HIGH)

def setupDC():
    global p_cw, p_ccw, en, cw, ccw, inputA, en, enDC
    # Setup Spindle
    GPIO.setup(en,GPIO.OUT)
    GPIO.output(en,True)
    GPIO.setup(ccw,GPIO.OUT)
    p_ccw=GPIO.PWM(ccw,2800) # frequensi 2800 = real 2000 hz
    p_ccw.start(0)
    GPIO.setup(cw,GPIO.OUT)
    p_cw=GPIO.PWM(cw,2800)
    p_cw.start(0)
    GPIO.setup(inputA, GPIO.IN)    # input mode
    GPIO.output(en,enDC)

def encoder(now_rpm):
    counter, enc_now, enc_prev, prev_time = 0, 0, 0, 0.0
    interval = 0.1
    ppr = 200
    while(1):
        enc_now = GPIO.input(31)
        if enc_now != enc_prev:
            counter +=1
        enc_prev = enc_now

        if (time.time() - prev_time > interval):
            prev_time = time.time()
            now_rpm.value = int(counter*600/ppr)
            counter = 0

def motorDC():
    global outPWM, in_max, in_min, out_max, out_min, inRPM, mState, rpm, error, set_point, last_error, pid, kp, ki, kd, p_cw, p_ccw, prevT
    if (time.time() - prevT >= 0.2):
        prevT = time.time()
        error = inRPM - rpm
        error = (error-in_min)*(out_max-out_min)/(in_max-in_min) + out_min
        pid = (kp * error) + (ki * (error + last_error)) + (kd * (error - last_error))
        last_error = error
        outPWM += pid  
        if (outPWM >=99):
            outPWM = 99
        elif (outPWM <= 0):
            outPWM = 0
        if (isConnected == True and mState != 0):
            conn.send(str.encode("SS=" + str(int(rpm))))
        #print("RPM : " + str(int(rpm)) + ", ERROR : " + str(int(error)) + ", PWM : " + str(int(outPWM)))
    if (mState == 1):
        p_cw.ChangeDutyCycle(outPWM)
        p_ccw.ChangeDutyCycle(0)
    elif (mState == 2):
        p_cw.ChangeDutyCycle(0)
        p_ccw.ChangeDutyCycle(outPWM)
    else:
        p_cw.ChangeDutyCycle(0)
        p_ccw.ChangeDutyCycle(0)

def Bresenham3D():
    global x1, y1, z1 , x2, y2, z2, sb, dx, dy, dz, xs, ys, zs, p1, p2, runStep, stepX, stepY, stepZ
    if (sb == 0):
        sb = 1
        dx = abs(x2 - x1) 
        dy = abs(y2 - y1) 
        dz = abs(z2 - z1)
    
        if (x2 > x1): 
            xs = 1
        else: 
            xs = -1
        if (y2 > y1): 
            ys = 1
        else: 
            ys = -1
        if (z2 > z1): 
            zs = 1
        else: 
            zs = -1
    
        # Driving axis is X-axis" 
        if (dx >= dy and dx >= dz):         
            p1 = 2 * dy - dx 
            p2 = 2 * dz - dx 
        # Driving axis is Y-axis" 
        elif (dy >= dx and dy >= dz):        
            p1 = 2 * dx - dy 
            p2 = 2 * dz - dy
        # Driving axis is Z-axis" 
        else:         
            p1 = 2 * dy - dz 
            p2 = 2 * dx - dz
                
    # Driving axis is X-axis" 
    if (dx >= dy and dx >= dz):         
        if (x1 != x2): 
            x1 += xs 
            if (p1 >= 0): 
                y1 += ys 
                p1 -= 2 * dx 
            if (p2 >= 0): 
                z1 += zs 
                p2 -= 2 * dx 
            p1 += 2 * dy 
            p2 += 2 * dz 
    # Driving axis is Y-axis" 
    elif (dy >= dx and dy >= dz):        
        if (y1 != y2): 
            y1 += ys 
            if (p1 >= 0): 
                x1 += xs 
                p1 -= 2 * dy 
            if (p2 >= 0): 
                z1 += zs 
                p2 -= 2 * dy 
            p1 += 2 * dx 
            p2 += 2 * dz 
    # Driving axis is Z-axis" 
    else:         
        if (z1 != z2): 
            z1 += zs 
            if (p1 >= 0): 
                y1 += ys 
                p1 -= 2 * dz 
            if (p2 >= 0): 
                x1 += xs 
                p2 -= 2 * dz 
            p1 += 2 * dy
            p2 += 2 * dx

    if (x1 == x2 and y1 == y2 and z1 == z2):
        sb = 0

def stepper():
    global x1, y1, z1, x2, y2, z2, stepX, stepY, stepZ, start, mode, oring
    #Cek kondisi limit
    if (oring == 0):
        if ((GPIO.input(LimitX[0]) == False and x1 < stepX) or (GPIO.input(LimitX[1]) == False and x1 > stepX)):
            stepX = x1
            x2 = x1
            print("Limit X kena")
            if (mode == 1 and start == 1): # Emergency jika menyentuh limit saat auto
                emrg()
        if ((GPIO.input(LimitY[0]) == False and y1 < stepY) or (GPIO.input(LimitY[1]) == False and y1 > stepY)):
            stepY = y1
            y2 = y1
            print("Limit Y kena")
            if (mode == 1 and start == 1): # Emergency jika menyentuh limit saat auto
                emrg()
        if ((GPIO.input(LimitZ[0]) == False and z1 < stepZ) or (GPIO.input(LimitZ[1]) == False and z1 > stepZ)):
            stepZ = z1
            z2 = z1
            print("Limit Z kena")
            if (mode == 1 and start == 1): # Emergency jika menyentuh limit saat auto
                emrg()
    # Stepper X 
    if (stepX != x1): 
        if (stepX < x1):
            state = CCW
        else:
            state = CW
        stepX = x1
        GPIO.output(DIR[0], state)
        GPIO.output(STEP[0], GPIO.HIGH)

    # Stepper Y
    if (stepY != y1):
        if (stepY < y1):
            state = CW
        else:
            state = CCW
        stepY = y1
        GPIO.output(DIR[1], state)
        GPIO.output(STEP[1], GPIO.HIGH)

    # Stepper Z
    if (stepZ != z1):
        if (stepZ < z1):
            state = CW
        else:
            state = CCW
        stepZ = z1
        GPIO.output(DIR[2], state)
        GPIO.output(STEP[2], GPIO.HIGH)

def stepperOFF():
    # Memberi sinyal LOW ke semua stepper
    for x in range (3):
        GPIO.output(STEP[x], GPIO.LOW)

def runOri():
    global oring, x1, y1, z1, home, homing, runStep, isConnected, x2, y2, z2, rst, stepX, stepY, stepZ, ori
    if (ori == 0):
        z1 += 1
        if (GPIO.input(LimitZ[1]) == False): # Saat Limit Z tersentuh
            ori = 1
            x2, y2, z2 = 0, 0, z1
    if (ori == 2):
        if (z2 == z1):
            ori = 0
            oring = 0
            runStep = 0
        else:
            z1 -= 1

def runHoming():
    global sb, x1, y1, z1, home, homing, runStep, isConnected, x2, y2, z2, rst, stepX, stepY, stepZ
    # Menjalankan siklus homing
    if (home == 0):
        z1 += 1
        if (GPIO.input(LimitZ[1]) == False): # Saat Limit Z tersentuh
            home = 1
    if (home == 1):
        if (GPIO.input(LimitX[1]) == True):
            x1 += 1
        if (GPIO.input(LimitY[1]) == True):
            y1 += 1
    if ((GPIO.input(LimitX[1]) == False) and (GPIO.input(LimitY[1]) == False) and (GPIO.input(LimitZ[1]) == False)): # Kondisi saat sampai home pos
        sb = 0
        home = 0
        homing = 0
        runStep = 0
        x2,y2,z2 = x1, y1, z1
        stepX, stepY, stepZ = x1, y1, z1
        if (rst == 1):
            #x1,y1,z1 = 0,0,0
            rst = 0
        if (isConnected == True):
            conn.send(str.encode("HP"))

def emrg():
    global oring, ori, home, homing, hayam, mState, flagEmg, start, runStep, homing, allAuto, home, homing, endogan, mode, enStep, seq, lastcmd, sb, feedRate, feed00, line
    # Reset variable saat Emergency
    mState = 0
    flagEmg = 1
    oring, ori, homing, home = 0,0,0,0
    start, runStep, home, homing, endogan, mode, enStep, line, seq, sb = 0,0,0,0,0,0,0,0,0,0
    feedRate = feed00
    allAuto = ""
    lastcmd = ""
    print("Emergency")
    if (hayam == 0):
        if (isConnected == True):
            conn.send(str.encode("EMG"))
        hayam = 1

def calcFR():
    global feedRate, feedRateSet
    feedRateSet = feedRateSet

def nManual():
    global mState, hayam, allAuto, dkp, dki, dkd, kp, ki, kd, mState, inRPM, oring, manual, x1, x2, y1, y2, z1, z2, mode, emg, runStep, homing, flagEmg, isConnected, start, feedRate, feedRateSet, feed00, rst, hitung, line, sb, endogan, increment
    # Input
    if (manual != ""):
        m = manual
        if (m[0] == "emg"): # emergency
            emrg()
        if (m[0] == "rst" and emg == 0 and flagEmg == 1): # reset
            homing = 1   
            flagEmg = 0
            rst = 1
            hayam = 0
            if (isConnected == True):
                conn.send(str.encode("RST"))
        if (m[0] == "clr"):
            allAuto = ""
        if (flagEmg == 0): 
            if (mode == 1):
                if (m[0] == "stop"): # stop
                    start, endogan, mode, line, sb = 0, 0, 0, 0, 0
                    x2, y2, z2 = x1, y1, z1
                    feedRate = feed00
                    runStep = 0
                    mState = 0    
                    if (isConnected == True):
                        conn.send(str.encode("STOP"))
            if (m[0] == "start" and allAuto != "" and runStep == 0): # start
                oring = 1
                mode, start = 1, 1
                runStep = 1   
                hitung = 0
                increment = 0
                line = 0
                if (isConnected == True):
                    conn.send(str.encode("EP"))
            if (mode == 0):  
                if (m[0][0] == "f"): # Feedrate ( mm/menit )
                    feedRateSet = float(60/(float(m[0][1:])*400)/2)
                    calcFR()
                    feedRate = feedRateSet
                    print ("delay: " + str(feedRate))
                if (m[0] == "df"): # Default Feedrate
                    feedRate = feed00
                    print ("Feedrate default!")
                if (m[0] == "dpid"): # Default PID
                    kp, ki, kd = dkp, dki, dkd
                    print ("PID default!")
                if (m[0][0:2] == "kp"): # P
                    kp = float(m[0][2:])
                    print ("KP" + str(kp))
                if (m[0][0:2] == "ki"): # I
                    ki = float(m[0][2:])
                    print ("KI" + str(ki))
                if (m[0][0:2] == "kd"): # D
                    kd = float(m[0][2:])
                    print ("KD" + str(kd))
                if (m[0] == "ds"): # Default Spindle
                    inRPM = 3000
                    print ("PWM default!")
                if (m[0] == "ccw" and mState == 0):
                    mState = 2
                    print("Motor CCW")
                if (m[0] == "cw" and mState == 0):
                    mState = 1
                    print("Motor CW")
                if (m[0] == "spin0"):
                    mState = 0
                    print("Motor Mati")
                elif (m[0][0] == "s" and m[0] != "set0" and m[0] != "stop" and m[0] != "start"): # Spindle RPM
                    inRPM = int(m[0][1:])
                    if (inRPM >= 4000):
                        inRPM = 4000
                    else:
                        inRPM == int(m[0][1:])
                    print("Input PWM : " + str(inRPM))
                if (runStep == 0):
                    if (m[0] == "set0"): # Setting 0
                        x1, y1, z1, x2, y2, z2 = 0, 0, 0, 0, 0, 0
                        runStep = 0
                        print ("Setting 0, OK!")
                    if (m[0] == "ori"): # Back to Origin (0)
                        oring = 1
                        runStep = 1
                        print ("Back to Origin!")
                    if (m[0] == "home"): # Homing
                        homing = 1
                        print ("Homing!")
                    if (m[0][0:2] == "x-"): # X--
                        x2 -= int(float(m[0][2:])*400)
                        runStep = 1
                    if (m[0][0:2] == "x+"): # X++
                        x2 += int(float(m[0][2:])*400)
                        runStep = 1
                    if (m[0][0:2] == "y-"): # Y--
                        y2 -= int(float(m[0][2:])*400)
                        runStep = 1
                    if (m[0][0:2] == "y+"): # Y++
                        y2 += int(float(m[0][2:])*400)
                        runStep = 1
                    if (m[0][0:2] == "z-"): # Z--
                        z2 -= int(float(m[0][2:])*400)
                        runStep = 1
                        print (z2)
                    if (m[0][0:2] == "z+"): # Z++
                        z2 += int(float(m[0][2:])*400) 
                        runStep = 1  
                        print (z2)        

def nAuto():
    global fr, auto, x1, x2, y1, y2, z1, z2, home, feedRate, feedRateSet, feed00, increment, mState, inRPM
    # Mode Auto
    if (auto != ""):
        n = auto
        for x in range (len(n)):
            if (n[x] == "G1" or n[x] == "G01"): # G01
                fr = 1
                print("G01")
            if (n[x] == "G0" or n[x] == "G00"): # G00
                fr = 0
                print("G00")
            if (n[x] == "M3"): # Motor CW
                mState = 1
                print("M3")
                print("Motor CW!")
            if (n[x] == "M4"): # Motor CCW
                mState = 2
                print("M4")
                print("Motor CCW!")
            if (n[x] == "M5"): # Motor Mati
                mState = 0
                print("M5")
                print("Motor MATI!")
            if (n[x] == "G90"): # Absolute
                increment = 0
                print("Absolut")
            if (n[x] == "G91"): # Incremental
                increment = 1
                print("Increment")
            if (n[x][0] == "F"): # Feedrate ( mm/menit )
                feedRateSet = float(60/(float(n[x][1:])*400)/2)
                calcFR()
                if (fr == 1):
                    feedRate = feedRateSet
                else:
                    feedRate = feed00
                print("Delay: " + str(feedRate))
            if (n[x][0] == "S"): # Spindle RPM 
                inRPM = float(n[0][1:])
                print("RPM : " + str(inRPM))
            if (n[x][0] == "X"): # GCODE X
                if (increment == 0):
                    x2 = int(float(n[x][1:])*400)
                else:
                    x2 += int(float(n[x][1:])*400)
            if (n[x][0] == "Y"): # GCODE Y
                if (increment == 0):
                    y2 = int(float(n[x][1:])*400)
                else:
                    y2 += int(float(n[x][1:])*400)
            if (n[x][0] == "Z"): # GCODE Z
                if (increment == 0):
                    z2 = int(float(n[x][1:])*400)
                else:
                    z2 += int(float(n[x][1:])*400)
        if (fr == 1):
            feedRate = feedRateSet
        else:
            feedRate = feed00

setup()
setupDC()

if __name__ == "__main__":
    now_rpm = multiprocessing.Value('i')
    p1 = multiprocessing.Process(target=encoder, args=(now_rpm, )) 
    # starting process 1 
    p1.start()
    while (1):
        if (lastCmd != ""):
            if (((lastCmd != "" and len(lastCmd.split()) >= 2) or (lastCmd[0] == "X" or lastCmd[0] == "Y" or lastCmd[0] == "Z" )) and lastCmd[0:2] != "kp" and start == 0): # Mengolah input Auto
                allAuto = allAuto + lastCmd
                lastCmd = ""
                if (allAuto.splitlines()[0].split()[0] == "%%"):
                    ucing = 1
                if (allAuto.splitlines()[len(allAuto.splitlines()) - 1].split()[0] == "M30"):
                    if (ucing == 1):
                        print("GCODE Auto Diterima!!!!!!!!!!!!!!!")
                    else:
                        print("GCODE Auto Diterima!")
                    print("Jumlah Baris: " + str(len(allAuto.splitlines())+1))
                    if (ucing == 1):
                        with open ("sidang.nc", "r") as myfile:
                            data=myfile.readlines()
                            allAuto = ''.join(data)
                    if (isConnected == True):
                            conn.send(str.encode("TS"))
            elif ((lastCmd != "" and len(lastCmd.split()) <= 1) or lastCmd[0:2] == "kp"): # Mengolah input Manual
                manual = lastCmd.split()
                lastCmd = ""
                nManual()
                manual = ""
        if (mode == 1 and start == 1 and flagEmg == 0 and runStep == 0 and homing == 0 and ((oring == 0) or (endogan == 1))): # Menjalankan mode Auto
            if (hitung == 0):
                prevTimeHitung = time.time()
                hitung = 1
            endogan = 1
            if (line <= len(allAuto.splitlines()) - 1): # Proses Auto
                auto = allAuto.splitlines()[line].split()
                nAuto()
                print(str(line+1) + ". "+ str(auto))
                if (isConnected == True):
                    conn.send(str.encode("LN" + str(line+1)))
                auto = ""
                line += 1
                runStep = 1
            else: # Mode Auto selesai
                allAuto = ""
                runStep, line, mode, start, endogan = 0, 0, 0, 0, 0
                feedRate = feed00
                mState = 0
                if (isConnected == True):
                    conn.send(str.encode("ES"))
                print ("Auto Kelar!, Waktu proses: " + str(time.time() - prevTimeHitung))
        if (GPIO.input(Emg) == False): # Cek kondisi Emergency
            if (time.time() - prevEmg > 0.5):
                emg = 1
                flagEmg = 1
        else:
            emg = 0
            prevEmg = time.time()
        if (flagEmg == 1): # Kondisi emergency
            GPIO.output(LampuM, GPIO.LOW) # Lampu Merah nyala
            GPIO.output(LampuK, GPIO.HIGH)
            GPIO.output(LampuH, GPIO.HIGH)
            emrg()
        elif(runStep == 1): # Kondisi Stepper bergerak
            GPIO.output(LampuM, GPIO.HIGH)
            GPIO.output(LampuK, GPIO.HIGH)
            GPIO.output(LampuH, GPIO.LOW) # Lampu Hijau Nyala
        else:
            GPIO.output(LampuM, GPIO.HIGH)
            GPIO.output(LampuK, GPIO.LOW) # Lampu kuning nyala
            GPIO.output(LampuH, GPIO.HIGH)
        if (isConnected == True and ((x1 != x1a and x1 % 400 == 0) or (y1 != y1a and y1 % 400 == 0) or (z1 != z1a and z1 % 400 == 0))): # Mengirim koordinat XYZ ke interface
            setring = "X=" + str(int(x1/400)) + "Y" + str(int(y1/400)) + "Z" + str(int(z1/400))
            conn.send(str.encode(setring))
        #Nilai Prev XYZ
        x1a, y1a, z1a = x1, y1, z1
        rpm = now_rpm.value
        motorDC()
        if ( homing == 1 ): # jika siklus homing
            runStep = 1
        if ( oring == 1 ): # jika siklus homing
            runStep = 1
        if ((runStep == 1)): # Proses menjalankan bresenham dan stepper
            if (seq == 0):
                if ( homing == 1 ):
                    runHoming() # Homing
                elif (oring == 1 and ori != 1):
                    runOri()
                elif (x2 == x1 and y2 == y1 and ori == 1):
                    z2 = 0
                    ori = 2
                else:
                    Bresenham3D() # Menjalankan Bresenham
                if (stepX != x1 or stepY != y1 or stepZ != z1):
                    prevTime = time.time()
                    seq = 1
                else:
                    runStep = 0
            if (seq == 1): # Menjalankan stepper
                if (time.time() - prevTime >= feedRate):
                    prevTime = time.time()
                    if (enStep == 0): # Stepper diberi HIGH
                        stepper()
                        enStep = 1
                    else: # Stepper diberi LOW
                        stepperOFF() 
                        seq, enStep = 0, 0