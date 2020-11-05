# Web streaming and save split under RPi.GPIO control
# Todo:
#		Upgrade to the full save routine with threading and chunking

import io
import picamera
from picamera import mmal, mmalobj as mo
import logging
import socketserver
import threading
from threading import Condition
from http import server
import datetime as dt
import RPi.GPIO as GPIO
import time
import pickle
import csv
import os
import sys

#####Clocks, streams, paths, chunking, GPIO
streamOn = False
firstFrameIdx = 0
trial_start = 0
GPUtimer = mo.MMALCamera()
trialNum = 0
sessionNum = 0
justOff = False
framesPerSave = 2000
dateStr = time.strftime("%Y%m%d")
savePathInit = '/media/usb/'
savePath = savePathInit + dateStr + '/'
if not os.path.exists(savePath):
	os.mkdir(savePath)
#GPIO stuff
GPIO.setwarnings(False)
GPIO.cleanup()
GPIO.setmode(GPIO.BCM)
on_pin = 25 #
GPIO.setup(on_pin,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)

#####Setting up HTML page appearance
PAGE="""\
<html>
<head>
<title>Picamera Stream</title>
</head>
<body>
<center><h1>Picamera Stream</h1></center>
<center><img src="stream.mjpg" width="640" height="480" /></center>
</body>
</html>
"""

#Handles direct camera communication and frame writes
class StreamingOutput(object):
    def __init__(self):
        self.frame = None
        self.buffer = io.BytesIO()
        self.condition = Condition()

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # New frame, copy the existing buffer's content and notify all
            # clients it's available
            self.buffer.truncate()
            with self.condition:
                self.frame = self.buffer.getvalue()
                self.condition.notify_all()
            self.buffer.seek(0)
        return self.buffer.write(buf)

#Does most of the things
class StreamingHandler(server.BaseHTTPRequestHandler):
	#Getting the clock value from the picamera GPU
	def get_millis():
		millis = (round(GPUtimer.control.params[mmal.MMAL_PARAMETER_SYSTEM_TIME]/1000))
		return millis
	#Submits image data to buffer stream, initiates chunk saving
	def do_GET(self):
		#Initialize variables for data, loop counters
		data = []
		metadata = []
		trialNum = 0
		nFrame = 0
		nChunk = 0
		trialOn = False
		global streamOn
		streamOn = True
		#Initialize server connection
		if self.path == '/':
			self.send_response(301)
			self.send_header('Location', '/index.html')
			self.end_headers()
		elif self.path == '/index.html':
			content = PAGE.encode('utf-8')
			self.send_response(200)
			self.send_header('Content-Type', 'text/html')
			self.send_header('Content-Length', len(content))
			self.end_headers()
			self.wfile.write(content)
		elif self.path == '/stream.mjpg':
			self.send_response(200)
			self.send_header('Age', 0)
			self.send_header('Cache-Control', 'no-cache, private')
			self.send_header('Pragma', 'no-cache')
			self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
			self.end_headers()
			#Serve image data
			try:
				while True:
					with output.condition:
						output.condition.wait()
						frame = output.frame
					self.wfile.write(b'--FRAME\r\n')
					self.send_header('Content-Type', 'image/jpeg')
					self.send_header('Content-Length', len(frame))
					self.end_headers()
					self.wfile.write(frame)
					self.wfile.write(b'\r\n')
					#Append frames and metadat to files until we get a whole chunk
					if GPIO.input(on_pin) and nFrame < framesPerSave:
						global trial_start, firstFrameIdx
						frame_millis = round(camera.frame.timestamp/1000) - trial_start
						frame_idx = camera.frame.index - firstFrameIdx
						#append metadata row
						metadata.append((frame_millis,frame_idx,trialNum))
						#append frame to list
						data.append(frame)
						trialOn = True
						nFrame += 1
					#Save a chunk
					elif GPIO.input(on_pin) and nFrame == framesPerSave:
						#Initiate save thread
						saver = saveThread(trialNum,nChunk,data,metadata)
						saver.start()
						#Prepare to save next chunk
						nChunk += 1
						nFrame = 0
						metadata = []
						data = []
					#Save the last chunk of this trial
					elif ~GPIO.input(on_pin) and trialOn:
						#Initiate save thread
						saver = saveThread(trialNum,nChunk,data,metadata)
						saver.start()
						#flash the off sign so we know we're not saving stream
						camera.annotate_text_size = 6
						camera.annotate_text = 'off'
						#Prepare loop variables for next trial
						trialNum += 1
						trialOn = False
						nChunk = 0
						nFrame = 0
						metadata = []
						data = []
			except Exception as e:
				camera.close()
				server.server_close()
				GPIO.cleanup()
				logging.warning(
					'Removed streaming client %s: %s',
					self.client_address, str(e))
		else:
			self.send_error(404)
			self.end_headers()

class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
	allow_reuse_address = True
	daemon_threads = True

#Instantiation spawns a thread to save camera data and metadata
class saveThread(threading.Thread):
	def __init__(self,trialNum,nChunk,data,metadata):
		threading.Thread.__init__(self)
		self.sessionNum = sessionNum
		self.trialNum = trialNum
		self.nChunk = nChunk
		self.data = data
		self.metadata = metadata
	def run(self):
		t = time.time()
		dateStr = time.strftime("%Y%m%d")
		timeStr = time.strftime("%H%M%S")
		fName = savePath + animalID + '_' + dateStr + '_' + timeStr + 'picam_s' +\
		str(self.sessionNum)+ '_t' + str(self.trialNum) + '_c' + str(self.nChunk)
		#filenames
		metFname = fName + '.csv'
		dataFname = fName + '.data'
		#Metadata save
		with open(metFname,'w',newline = '') as metout:
			wr = csv.writer(metout)
			wr.writerows(self.metadata)
		metout.close()
		#Pickle hexadecimal image data
		fData = open(dataFname,'wb')
		pickle.dump(self.data,fData)
		fData.close()
		elapsed = time.time() - t
		print('Wrote files, writing took' + str(elapsed))

#Setting response to interrupt
def interrupt_on(on_pin):
	millis = round(GPUtimer.control.params[mmal.MMAL_PARAMETER_SYSTEM_TIME]/1000)
	#Only adjust global variable states if user client has attached to server
	if streamOn:
		global trial_start, firstFrameIdx
		trial_start = millis
		firstFrameIdx = camera.frame.index
		camera.annotate_text = ''
		print('Trial start interrupt detected by picam')
        
#Allowing savepath to be passed in from another program
if (len(sys.argv)>1):
    savePath = sys.argv[1]
    animalID = sys.argv[2]
    sessionNum = sys.argv[3]

######Main
with picamera.PiCamera(resolution='320x256', framerate=80) as camera:
	output = StreamingOutput()
	#Getting consistent camera levels
	camera.rotation = 180
	time.sleep(2) #let the levels settle
	camera.shutter_speed = camera.exposure_speed
	camera.exposure_mode = 'off'
	g = camera.awb_gains
	camera.awb_mode = 'off'
	camera.awb_gains = g
	#Setting up camera clock, outputs, and initial image annotation
	camera.clock_mode = 'raw'
	camera.start_recording(output, format='mjpeg')
	camera.annotate_background = picamera.Color('black')
	camera.annotate_text_size = 6
	camera.annotate_text = dt.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
	try:
		address = ('', 8000)
		server = StreamingServer(address, StreamingHandler)
		GPIO.add_event_detect(on_pin,GPIO.RISING,callback=interrupt_on)
		server.serve_forever()
	except KeyboardInterrupt:
		print('Keyboard interrupt ended stream at address',address)
		camera.close()
		server.server_close()
		GPIO.cleanup()
		pass
	finally:
		camera.stop_recording()
