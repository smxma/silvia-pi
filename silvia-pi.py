#!/usr/bin/python

def scheduler(dummy,state):
  import time
  import sys
  import schedule
  from datetime import datetime

  sys.stdout = open("scheduler.log", "a", buffering=0)
  sys.stderr = open("scheduler.err.log", "a", buffering=0)

  print "Starting scheduler thread ..."

  last_wake = 0
  last_sleep = 0
  last_sched_switch = 0

  while True:

    if last_wake != state['wake_time'] or last_sleep != state['sleep_time'] or last_sched_switch != state['sched_enabled']:
      schedule.clear()

      if state['sched_enabled'] == True:
        schedule.every().day.at(state['sleep_time']).do(gotosleep,1,state)
        schedule.every().day.at(state['wake_time']).do(wakeup,1,state)

        nowtm = float(datetime.now().hour) + float(datetime.now().minute)/60.
        sleeptm = state['sleep_time'].split(":")
        sleeptm = float(sleeptm[0]) + float(sleeptm[1])/60.
        waketm = state['wake_time'].split(":")
        waketm = float(waketm[0]) + float(waketm[1])/60.

        if waketm < sleeptm:
          if nowtm >= waketm and nowtm < sleeptm:
            wakeup(1,state)
          else:
            gotosleep(1,state)
        elif waketm > sleeptm:
          if nowtm < waketm and nowtm >= sleeptm:
            gotosleep(1,state)
          else:
            wakeup(1,state)

      else:
        wakeup(1,state)

    last_wake = state['wake_time']
    last_sleep = state['sleep_time']
    last_sched_switch = state['sched_enabled']

    schedule.run_pending()

    time.sleep(1)

def wakeup(dummy,state):
  state['is_awake'] = True

def gotosleep(dummy,state):
  state['is_awake'] = False

def he_control_loop(dummy,state):
  print("he_control_loop execution ..")
  from time import sleep
  import RPi.GPIO as GPIO
  import config as conf
  import sys

  GPIO.setmode(GPIO.BOARD)
  GPIO.setwarnings(False)
  GPIO.setup(conf.he_pin, GPIO.OUT)
  GPIO.output(conf.he_pin,0)

  heating = False
  sys.stderr = open("he_control_loop.err.log", "a", buffering=0)
  try:
    while True:
      avgpid = state['avgpid']

      if state['is_awake'] == False :
        state['heating'] = False
        GPIO.output(conf.he_pin,0)
        sleep(1)
      else:
        if avgpid >= 100 :
          state['heating'] = True
          GPIO.output(conf.he_pin,1)
          sleep(1)
        elif avgpid > 0 and avgpid < 100:
          state['heating'] = True
          GPIO.output(conf.he_pin,1)
          sleep(avgpid/100.)
          GPIO.output(conf.he_pin,0)
          sleep(1-(avgpid/100.))
          state['heating'] = False
        else:
          GPIO.output(conf.he_pin,0)
          state['heating'] = False
          sleep(1)

  finally:
    GPIO.output(conf.he_pin,0)
    GPIO.cleanup()

def pid_loop(dummy,state):
  import sys
  from time import sleep, time
  from math import isnan
  from max31855 import MAX31855
  import RPi.GPIO as GPIO
  import PID as PID
  import config as conf
  sys.stdout = open("pid.log", "a", buffering=0)
  sys.stderr = open("pid.err.log", "a", buffering=0)

  def c_to_f(c):
    return c * 9.0 / 5.0 + 32.0

  sensor = MAX31855(cs_pin=conf.CS,clock_pin=conf.CLK, data_pin=conf.DO, board=GPIO.BOARD)

  pid = PID.PID(conf.Pc,conf.Ic,conf.Dc)
  pid.SetPoint = state['settemp']
  pid.setSampleTime(conf.sample_time*5)

  nanct=0
  i=0
  j=0
  pidhist = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
  avgpid = 0.
  temphist = [0.,0.,0.,0.,0.]
  avgtemp = 0.
  lastsettemp = state['settemp']
  lasttime = time()
  sleeptime = 0
  iscold = True
  iswarm = False
  lastcold = 0
  lastwarm = 0

  try:
    while True : # Loops 10x/second
      tempc = sensor.get_rj()
      if isnan(tempc) :
        nanct += 1
        if nanct > 100000 :
          sys.exit
        continue
      else:
        nanct = 0

      tempf = c_to_f(tempc)
      temphist[i%5] = tempf
      avgtemp = sum(temphist)/len(temphist)

      if avgtemp < 100 :
        lastcold = i

      if avgtemp > 200 :
        lastwarm = i

      if iscold and (i-lastcold)*conf.sample_time > 60*15 :
        pid = PID.PID(conf.Pw,conf.Iw,conf.Dw)
        pid.SetPoint = state['settemp']
        pid.setSampleTime(conf.sample_time*5)
        iscold = False

      if iswarm and (i-lastwarm)*conf.sample_time > 60*15 : 
        pid = PID.PID(conf.Pc,conf.Ic,conf.Dc)
        pid.SetPoint = state['settemp']
        pid.setSampleTime(conf.sample_time*5)
        iscold = True

      if state['settemp'] != lastsettemp :
        pid.SetPoint = state['settemp']
        lastsettemp = state['settemp']

      if i%10 == 0 :
        pid.update(avgtemp)
        pidout = pid.output
        pidhist[i/10%10] = pidout
        avgpid = sum(pidhist)/len(pidhist)

      state['i'] = i
      state['tempf'] = round(tempf, 2)
      state['tempC'] = round(tempc, 2)
      state['avgtemp'] = round(avgtemp,2)
      state['pidval'] = round(pidout,2)
      state['avgpid'] = round(avgpid,2)
      state['pterm'] = round(pid.PTerm,2)
      if iscold :
        state['iterm'] = round(pid.ITerm * conf.Ic,2)
        state['dterm'] = round(pid.DTerm * conf.Dc,2)
      else :
        state['iterm'] = round(pid.ITerm * conf.Iw,2)
        state['dterm'] = round(pid.DTerm * conf.Dw,2)
      state['iscold'] = iscold

      print time(), state

      sleeptime = lasttime+conf.sample_time-time()
      if sleeptime < 0 :
        sleeptime = 0
      sleep(sleeptime)
      i += 1
      lasttime = time()

  finally:
    pid.clear

def f_to_c(f):
  return ((f - 32) * 5.0/9.0)
  
def display_state_oled(dummy, state):
  import time
  import Adafruit_SSD1306
  from datetime import datetime
  from PIL import Image
  from PIL import ImageDraw
  from PIL import ImageFont
  import subprocess
  import sys

  sys.stderr = open("display_oled.err.log", "a", buffering=0)
  
  # Raspberry Pi pin configuration:
  RST = None     # on the PiOLED this pin isnt used
  # Note the following are only used with SPI:
  DC = 23
  SPI_PORT = 0
  SPI_DEVICE = 0
  
  # 128x64 display with hardware I2C:
  disp = Adafruit_SSD1306.SSD1306_128_64(rst=RST)
  
  # Initialize library.
  disp.begin()

  # Clear display.
  disp.clear()
  disp.display()

  # Create blank image for drawing.
  # Make sure to create image with mode '1' for 1-bit color.
  width = disp.width
  height = disp.height
  image = Image.new('1', (width, height))

  # Get drawing object to draw on image.
  draw = ImageDraw.Draw(image)

  # Draw a black filled box to clear the image.
  draw.rectangle((0,0,width,height), outline=0, fill=0)

  # Draw some shapes.
  # First define some constants to allow easy resizing of shapes.
  padding = -2
  top = padding
  bottom = height-padding
  # Move left to right keeping track of the current x position for drawing shapes.
  x = 0


  # Load default font.
  font = ImageFont.load_default()

  while True:

      # Draw a black filled box to clear the image.
      draw.rectangle((0,0,width,height), outline=0, fill=0)

      # Shell scripts for system monitoring from here : https://unix.stackexchange.com/questions/119126/command-to-display-memory-usage-disk-usage-and-cpu-load
      cmd = "hostname -I | cut -d\' \' -f1"
      IP = subprocess.check_output(cmd, shell = True )
      cmd = "top -bn1 | grep load | awk '{printf \"CPU Load: %.2f\", $(NF-2)}'"
      CPU = subprocess.check_output(cmd, shell = True )
      cmd = "free -m | awk 'NR==2{printf \"Mem: %s/%sMB %.2f%%\", $3,$2,$3*100/$2 }'"
      MemUsage = subprocess.check_output(cmd, shell = True )
      cmd = "df -h | awk '$NF==\"/\"{printf \"Disk: %d/%dGB %s\", $3,$2,$5}'"
      Disk = subprocess.check_output(cmd, shell = True )

      currentTempC = "Current temp: %s C" % state['tempC']
      brewTemp = "Brew temp: %.2f C" % f_to_c(state['settemp'])
      isHeating = "Heating state: %s" % state['heating']
      
      dt_string = datetime.now().strftime("%d/%m/%Y %H:%M:%S")

      # Write two lines of text.
      draw.text((x, top),       "IP: " + str(IP),  font=font, fill=255)
      draw.text((x, top+8),     str(CPU), font=font, fill=255)
      draw.text((x, top+16),    str(MemUsage),  font=font, fill=255)
      draw.text((x, top+25),    str(Disk),  font=font, fill=255)
      draw.text((x, top+32),    str(currentTempC),  font=font, fill=255)
      draw.text((x, top+40),    str(brewTemp),  font=font, fill=255)
      draw.text((x, top+48),    str(isHeating),  font=font, fill=255)
      draw.text((x, top+56),    str(dt_string),  font=font, fill=255)

      # Display image.
      disp.image(image)
      disp.display()
      time.sleep(.1)


def rest_server(dummy,state):
  from bottle import route, run, get, post, request, static_file, abort
  from subprocess import call
  from datetime import datetime
  import config as conf
  import os
  import sys

  basedir = os.path.dirname(__file__)
  wwwdir = basedir+'/www/'
  
  sys.stderr = open("rest_server.err.log", "a", buffering=0)

  @route('/')
  def docroot():
    return static_file('index.html',wwwdir)

  @route('/<filepath:path>')
  def servfile(filepath):
    return static_file(filepath,wwwdir)

  @route('/curtemp')
  def curtemp():
    return str(state['avgtemp'])

  @route('/curtempC')
  def curtempC():
    return str(f_to_c(state['avgtemp']))
  @route('/heatState')
  def heatState():
    return str(state['heating'])
  @get('/settemp')
  def settemp():
    return str(state['settemp'])

  @post('/settemp')
  def post_settemp():
    try:
      settemp = float(request.forms.get('settemp'))
      if settemp >= 200 and settemp <= 260 :
        state['settemp'] = settemp
        return str(settemp)
      else:
        print("Temp out of range 200-260")
        abort(400,'Set temp out of range 200-260.')
    except:
      print('Invalid number for set temp.')
      abort(400,'Invalid number for set temp.')

  @get('/is_awake')
  def get_is_awake():
    return str(state['is_awake'])

  @post('/scheduler')
  def set_sched():
    sched = request.forms.get('scheduler')
    if sched == "True":
      state['sched_enabled'] = True
    elif sched == "False":
      state['sched_enabled'] = False
      state['is_awake'] = True
    else:
      abort(400,'Invalid scheduler setting. Expecting True or False')

  @post('/setwake')
  def set_wake():
    wake = request.forms.get('wake')
    try:
      datetime.strptime(wake,'%H:%M')
    except:
      abort(400,'Invalid time format.')
    state['wake_time'] = wake
    return str(wake)

  @post('/setsleep')
  def set_sleep():
    sleep = request.forms.get('sleep')
    try:
      datetime.strptime(sleep,'%H:%M')
    except:
      abort(400,'Invalid time format.')
    state['sleep_time'] = sleep
    return str(sleep)

  @get('/allstats')
  def allstats():
    return dict(state)

  @route('/restart')
  def restart():
    call(["reboot"])
    return '';

  @route('/shutdown')
  def shutdown():
    call(["shutdown","-h","now"])
    return '';

  @get('/healthcheck')
  def healthcheck():
    return 'OK'

  run(host=conf.host,port=conf.port, server=conf.server)

if __name__ == '__main__':
  from multiprocessing import Process, Manager
  from time import sleep
  from urllib2 import urlopen
  import config as conf

  manager = Manager()
  pidstate = manager.dict()
  pidstate['is_awake'] = True
  pidstate['sched_enabled'] = conf.sched_enabled
  pidstate['sleep_time'] = conf.sleep_time
  pidstate['wake_time'] = conf.wake_time
  pidstate['i'] = 0
  pidstate['settemp'] = conf.set_temp
  pidstate['avgpid'] = 0.
  print "Starting Scheduler thread..."
  s = Process(target=scheduler,args=(1,pidstate))
  s.daemon = True
  s.start()
  
  print "Starting display OLED..."
  r = Process(target=display_state_oled,args=(1,pidstate))
  r.daemon = True
  r.start()

  print "Starting PID thread..."
  p = Process(target=pid_loop,args=(1,pidstate))
  p.daemon = True
  p.start()

  print "Starting HE Control thread..."
  h = Process(target=he_control_loop,args=(1,pidstate))
  h.daemon = True
  h.start()

  print "Starting REST Server thread..."
  r = Process(target=rest_server,args=(1,pidstate))
  r.daemon = True
  r.start()

  #Start Watchdog loop
  print "Starting Watchdog..."
  piderr = 0
  weberr = 0
  weberrflag = 0
  urlhc = 'http://localhost:'+str(conf.port)+'/healthcheck'

  lasti = pidstate['i']
  sleep(1)

  while p.is_alive() and h.is_alive() and r.is_alive() and s.is_alive():
    curi = pidstate['i']
    if curi == lasti :
      piderr = piderr + 1
    else :
      piderr = 0

    lasti = curi

    if piderr > 9 :
      print 'ERROR IN PID THREAD, RESTARTING'
      p.terminate()

    try:
      hc = urlopen(urlhc,timeout=2)
    except:
      weberrflag = 1
    else:
      if hc.getcode() != 200 :
        weberrflag = 1

    if weberrflag != 0 :
      weberr = weberr + 1

    if weberr > 9 :
      print 'ERROR IN WEB SERVER THREAD, RESTARTING'
      r.terminate()

    weberrflag = 0

    sleep(1)
