'''dtsc_app.py
Joey Broussard
PNI
20191106

Interfaces the dtsc class with an HTML/JS based web GUI

(1) need to use eventlet, otherwise .run() defaults to gevent() which is SLOW
(2) monkey_path() wraps some functions to call eventlet equivalents
   in particule time.sleep() is redirected to coresponding eventlet() call

'''

from flask import Flask, abort, render_template, send_file, request
from flask_socketio import SocketIO, emit
from flaskext.markdown import Markdown

import os, time
from datetime import datetime
from threading import Thread
import eventlet
import json
import sys
import signal
import subprocess
from settings import APP_ROOT

from dtsc import dtsc
from settings import APP_ROOT

#see: https://github.com/miguelgrinberg/Flask-SocketIO/issues/192
eventlet.monkey_patch()

#eventlet.debug.hub_prevent_multiple_readers(False)
app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
app.debug = True
app.config['DATA_FOLDER'] = 'data/'

Markdown(app, extensions=['fenced_code'])

#Getting socket for state transfer and declaring namespace
socketio = SocketIO(app, async_mode='eventlet')
namespace = ''

#Initializing objects attributes
savepath = '/media/usb/' #location of mounted usb drive
thread = None #second thread used by background_thread()
ser = None
mydtsc = None
myanalysis = None

def background_thread():
    """Example of how to send server generated events to clients."""
    #Constantly update the server output stream with current 
    #object state
    while True:
        time.sleep(.7)
        response = MakeServerResponse()     

        jsonResponse = json.dumps(response)
        socketio.emit('serverUpdate', jsonResponse, namespace=namespace)
        
def MakeServerResponse():
    #Import dtsc object state to push to server
    now = datetime.now()
    dateStr = now.strftime("%m/%d/%y")
    timeStr = now.strftime("%H:%M:%S.%f")
    
    response = {}
    response['currentdate'] = dateStr
    response['currenttime'] = timeStr

    response['savepath'] = mydtsc.savepath
    response['animalID'] = mydtsc.animalID

    response['filePath'] = mydtsc.trial['filePath']
    response['fileName'] = mydtsc.trial['fileName']

    response['trialRunning'] = mydtsc.trialRunning
    response['sessionNumber'] = mydtsc.trial['sessionNumber']
    response['sessionDur'] = mydtsc.trial['sessionDur']

    response['trialNumber'] = mydtsc.trial['trialNumber']
    response['numTrial'] = mydtsc.trial['numTrial']

    response['useMotor'] = mydtsc.trial['useMotor']

    return response
    
@app.route('/')
def index():
    global thread
    if thread is None:
        print('starting background thread')
        thread = Thread(target=background_thread)
        thread.daemon  = True; #as a daemon the thread will stop when *this stops
        thread.start()
    theRet = render_template('indexE2.html', dtsc=mydtsc)
    return theRet

@app.route('/form2')
def form2():
    return render_template('form_sandbox.html')


#from: http://stackoverflow.com/questions/23718236/python-flask-browsing-through-directory-with-files
@app.route('/', defaults={'req_path': ''})
@app.route('/<path:req_path>')
def dir_listing(req_path):

    print('\n')
    print('req_path:', req_path)
    
    # Joining the base and the requested path
    abs_path = os.path.join(APP_ROOT, req_path)

    print('abs_path:', abs_path)

    # Return 404 if path doesn't exist
    if not os.path.exists(abs_path):
        return abort(404)

    # Check if path is a file and serve
    if os.path.isfile(abs_path):
        return send_file(abs_path)

    # Show directory contents
    if os.path.isdir(abs_path):
        print('IS DIRECTORY:', abs_path)
    files = os.listdir(abs_path)
    return render_template('files.html', path=req_path.replace('data/','') + '/', files=files)

@app.route('/grafica')
def index_grafica():
    return render_template('grafica2.html')

@socketio.on('connectArduino', namespace=namespace) #
def connectArduino(message):
    emit('my response', {'data': message['data']})
    print('connectArduino', message['data'])

@socketio.on('startarduinoButtonID', namespace=namespace) #
def startarduinoButton(message):
    print('startarduinoButtonID')
    mydtsc.startSession()
    
@socketio.on('stoparduinoButtonID', namespace=namespace) #
def stoparduinoButtonID(message):
    print('stoparduinoButtonID')
    mydtsc.stopSession()
    
@socketio.on('printArduinoStateID', namespace=namespace) #
def printArduinoStateID(message):
    mydtsc.GetArduinoState()

@socketio.on('emptySerialID', namespace=namespace) #
def printArduinoStateID(message):
    mydtsc.emptySerial()

@socketio.on('checkserialportID', namespace=namespace) #
def checkserialportID(message):
    exists, str = mydtsc.checkserialport()
    if exists:
        emit('serialdata', {'data': "OK: " + str})
    else:
        emit('serialdata', {'data': "ERROR: " + str})

@socketio.on('setSerialPortID', namespace=namespace) #
def setSerialPort(message):
    portStr = message['data']
    ok = mydtsc.setserialport(portStr)
    if ok:
        emit('serialdata', {'data': "OK: " + portStr})
    else:
        emit('serialdata', {'data': "ERROR: " + portStr})

@socketio.on('arduinoVersionID', namespace=namespace) #
def arduinoVersionID(message):
    mydtsc.checkarduinoversion()

@socketio.on('my event', namespace=namespace) #responds to echo
def test_message(message):
    emit('my response', {'data': message['data']})

@socketio.on('my broadcast event', namespace=namespace)
def test_message(message):
    emit('my response', {'data': message['data']}, broadcast=True)

@socketio.on('connect', namespace=namespace)
def test_connect():
    emit('my response', {'data': 'Connected'})

@socketio.on('disconnect', namespace=namespace)
def test_disconnect():
    print('*** dtsc_app -- Client disconnected')

@socketio.on('trialform', namespace=namespace)
def trialform(message):
    '''message is trailFormDict from dtsc object'''
    print('\n=== dtsc_app.trialform:', message)
    numTrial = message['numTrial']
    trialDur = message['trialDur']

    interTrialIntervalLow = message['interTrialIntervalLow']
    interTrialIntervalHigh = message['interTrialIntervalHigh']
    preCSdur = message['preCSdur']
    CSdur = message['CSdur']
    USdur = message['USdur']
    percentUS = message['percentUS']
    percentCS = message['percentCS']

    useMotor = message['useMotor'] #{motorOn, motorLocked, motorFree}
    motorSpeed = message['motorSpeed']
    #
    
    emit('serialdata', {'data': "=== Session Form ==="})
    
    mydtsc.settrial('numTrial', numTrial)
    time.sleep(0.01)
    mydtsc.settrial('trialDur', trialDur)
    time.sleep(0.01)

    mydtsc.settrial('interTrialIntervalLow', interTrialIntervalLow)
    time.sleep(0.01)
    mydtsc.settrial('interTrialIntervalHigh', interTrialIntervalHigh)
    time.sleep(0.01)
    mydtsc.settrial('preCSdur', preCSdur)
    time.sleep(0.01)
    mydtsc.settrial('CSdur', CSdur)
    time.sleep(0.01)
    mydtsc.settrial('USdur', USdur)
    time.sleep(0.01)
    mydtsc.settrial('percentUS', percentUS)
    time.sleep(0.01)
    mydtsc.settrial('percentCS', percentCS)
    time.sleep(0.01)

    mydtsc.settrial('motorSpeed', motorSpeed)
    time.sleep(0.01)
    mydtsc.settrial('useMotor', useMotor)
    time.sleep(0.01)

    mydtsc.updatetrial() #update total dur
    
    mydtsc.emptySerial()
    
    print('trialform() useMotor=', useMotor)
    
    emit('serialdata', {'data': "=== Session Form Done ==="})

@socketio.on('animalform', namespace=namespace)
def animalform(message):
    print('animalform:', message)
    animalID = message['animalID']
    mydtsc.animalID = animalID
    #mytreadmill.settrial('dur', dur)
    emit('my response', {'data': "animal id is now '" + animalID + "'"})


def sig_handler(signal,frame):
    print('Signal handler called by dtsc_app.py')
    mydtsc.__del__
    print('Process kill applied to piCamera subprocess')
    os.system("pkill -9 -f puffCamera2_0.py")
    sys.exit(0)

if __name__ == '__main__':
    try:
        #Make kill method for all processes and threads
        signal.signal(signal.SIGINT, sig_handler)

        #Initializing the dtsc object
        mydtsc = dtsc()
        #dataRoot = os.path.join(savepath) + '/'
        #mydtsc.setsavepath(dataRoot)
        mydtsc.bAttachSocket(socketio)

        #Reporting server state on connection
        print('starting server')
        #host settings in 'this' namespace location if desired here
        socketio.run(app, host='169.254.67.160', port=5010, use_reloader=True)
        print('finished')
    except:
        print('...exiting')
        raise
