# -*- coding: utf-8 -*-
"""
Created on Tue Dec 31 09:14:13 2019

@author: wanglab
"""

import imageio
#import imageio-ffmpeg
import pickle
import numpy as np
import cv2
import os
import pandas as pd

#%% getting file lists
#Path and files to make movies for
path = r'C:\Users\PNI User\Desktop\JB27_20200104_session21\rigData'
movIdx = np.arange(0,110,10)#[53]#np.arange(0,120,10)

files = sorted([(d,f) for d,_,fs in os.walk(path) for f in fs if f.endswith('.data')])
im_files = [os.path.join(*i) for i in files]
csv_files = [os.path.join(d, os.path.splitext(f)[0] + '.csv') for d,f in files]
#Get names and display available files
names = [os.path.splitext(os.path.split(f)[-1])[0] for f in im_files]
print('Available datasets:')
for idx,n in enumerate(names):
    print('\t{}\t{}'.format(idx,n))

#%% function block
def mjpg2array(filename):   
    #Open and unpickle the bytes file
    filehand = open(filename,'rb')
    stream = []
    while 1:
        try:
            stream.append(pickle.load(filehand))
        except EOFError:
            break
    filehand.close()
    #Removing the extra list packaging, getting nImages points
    stream = stream[0]
    nIm = len(stream)
    
    #Reading through the binary file to parse image data
    idx = 0 #for buidling the np image array
    for img in stream:
        #grab frame start and end on hex stream
        a = img.find(b'\xff\xd8')
        b = img.find(b'\xff\xd9')
        if a != -1 and b != -1:
            jpg = img[a:b+2]
            #stream = stream[b+2:]
            data = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
            
            if idx==0:
                #For plot of line scan
                sizeWH = data.shape
                #Full output array
                mov = np.empty([int(sizeWH[0]),int(sizeWH[1]),nIm],dtype=np.uint8)
                mov[:,:,0] = data
            else:
                mov[:,:,idx] = data
            idx += 1
            #cv2.imshow('i', data)
            #if cv2.waitKey(1)==27:
                #exit(0)
        elif a==-1 or b==-1:
            pass
    
    # permute the data to put time element first
    imArray = np.transpose(mov,(2,0,1))
    return imArray

#%% selecting and opening files to make into movies
for idx in range(len(movIdx)):
    #parse bytes to np array
    imArray = mjpg2array(im_files[movIdx[idx]])
    #Read the time logs
    csvfilehand = open(csv_files[movIdx[idx]])
    newData = pd.read_csv(csvfilehand,header=None, names = ['time','frame','trial'])
    time = newData['time'].values
    csvfilehand.close()
    #Make a subarray corresponding to highlighted trace period
    csTime = 2000
    csusInt = 300
    usDur = 50
    pad = [100,300]#ms pad before and after us termination
    timeEnds = [-pad[0]+csTime,pad[1]+csTime+csusInt]
    goodFrames = (time>timeEnds[0]) & (time<timeEnds[1])
    #Stamps for CS and US
    csStamp = np.zeros(np.shape(imArray[0]))
    csStamp[0:10,0:10] = 255
    usStamp = np.ones(np.shape(imArray[0]))
    usStamp[0:10,0:10] = 0
    csEnds = [csTime,csTime+csusInt+usDur]
    usEnds = [csTime+csusInt,csTime+csusInt+usDur]
    for a in np.where((time>csEnds[0]) & (time<csEnds[1])):
        imArray[a] = imArray[a]*usStamp + csStamp
    for b in np.where((time>usEnds[0]) & (time<usEnds[1])):
        imArray[b] = imArray[b]*usStamp
    #write to mp4
    imSubArray = imArray[goodFrames]
    if not os.path.exists(os.path.join(path,'sampleMovs')):
        os.makedirs(os.path.join(path,'sampleMovs'))
    im_fileOut = os.path.join(path,'sampleMovs',names[movIdx[idx]]+'.mp4')
    imageio.mimwrite(im_fileOut,imSubArray,fps=10,quality=9)
