# -*- coding: utf-8 -*-
"""
Created on Mon Nov 25 10:49:15 2019

@author: wanglab
"""

#%% import libraries
import numpy as np
import pickle
import cv2
import matplotlib.pyplot as pl
from matplotlib import cm
import pandas as pd
import os
import csv
#from scipy.ndimage.filters import uniform_filter1d as unfilt1D #for 1D temporal filtering

#%% Mask out the ROI to analyze, plot
# select eye ROI
def pickROI(imArray):
    pl.imshow(imArray.mean(axis=0), cmap=pl.cm.Greys_r)
    pts = pl.ginput(timeout=-1, n=-1)
    pl.close()
    
    # convert points to mask
    pts = np.asarray(pts, dtype=np.int32)
    roi = np.zeros(imArray[0].shape, dtype=np.int32)
    roi = cv2.fillConvexPoly(roi, pts, (1,1,1), lineType=cv2.LINE_AA)
    roi = roi.astype(np.float)
    return roi

#%% initializing the image stack and parsing the image stack
#unpickling the bytes stream from mjpg and converting to np array shape [nIm,wid,height]
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



#%% Choose a directory
#All camera and timestamp files
pathMaster = r'C:\Users\PNI User\Desktop\Sessions2Add'
subDirs = next(os.walk(pathMaster))[1]
print('Available datasets:')
for idx,n in enumerate(subDirs):
    print('\t{}\t{}'.format(idx,n))

#%%
#grabing the files and metadata from the chosen directory for analysis
subDirIdx = [1,2,3,4]#[52]#np.arange(53,61)

#Loop over all directories in pathMaster directory
for thisSubDir in subDirIdx:
    path = os.path.join(pathMaster,subDirs[thisSubDir],'rigData')
    files = sorted([(d,f) for d,_,fs in os.walk(path) for f in fs if f.endswith('.data')])
    im_files = [os.path.join(*i) for i in files]
    csv_files = [os.path.join(d, os.path.splitext(f)[0] + '.csv') for d,f in files]
    
    #Set local and summary save directories
    localDir = os.path.join(path,'Summary')
    if not os.path.exists(localDir):
            os.makedirs(localDir)
    headDir = os.path.join(pathMaster,'Summary')
    if not os.path.exists(headDir):
        os.makedirs(headDir)
    
    #Trial structure and metadata
    txt_files = sorted([(d,f) for d,_,fs in os.walk(path) for f in fs if f.endswith('.txt')])
    txt_files = [os.path.join(*i) for i in txt_files]
    txtfilehand = open(txt_files[0])
    metadata = pd.read_csv(txtfilehand,skiprows=1) #column heads as millis, events, value
    #Trial types
    trialTypes = ('CS','US','CS_US')
    trialTypes = metadata['event'][metadata['event'].isin(trialTypes)].values
    #ITSI inter-start intervals
    ISI = np.diff(metadata['millis'][metadata['event']=='startTrial'])
    ISI = np.insert(ISI,0,0) #ISI for first trial is 0
    #Grab first row headers, parse to dictionary
    with open(txt_files[0]) as txtfilehand:
        reader=csv.reader(txtfilehand)
        headers = next(reader)
    headers = dict(x.split('=') for x in headers[0].split(';') if '=' in x)
    txtfilehand.close()
    
    #Extracting subject, date, session from directory header
    sessionInfo = subDirs[thisSubDir].split('_')
    animalID = sessionInfo[0]
    date = sessionInfo[1]
    session = sessionInfo[2]
    
    #%% pick roi
    imPickArray = mjpg2array(im_files[0])
    #If ROI already selected, use saved version
    if os.path.exists(os.path.join(localDir,'_'.join(sessionInfo)+'roi.py.npy')):
        roi = np.load(os.path.join(localDir,'_'.join(sessionInfo)+'roi.py.npy'))
    else:
        pl.close('all')
        roi = pickROI(imPickArray)
        
    #%% getting data for each analyzed trial
    data = pd.DataFrame()
    for idx in range(len(files)):
        #Read the time logs
        csvfilehand = open(csv_files[idx])
        newData = pd.read_csv(csvfilehand,header=None, names = ['time','frame','trial'])
        csvfilehand.close()
#        #Remove trials with large gaps
#        if newData['time'].diff().max()>60:
#            print('_'.join(sessionInfo)+' '+str(idx) + ' lagged')
#            continue
        #Correct timestamps on trials that double (or triple) log preceeding trials
        count = 0
        while np.max(newData['time'])>6000:
            newData['time'] = newData['time'] - ISI[idx-count] + 7.4
            if np.max(newData['time'])>6000:
                count+=1
        
        #Read image data, process
        imArray = mjpg2array(im_files[idx])
        tr = imArray.reshape([len(imArray), -1]) @ roi.reshape(np.product(roi.shape))
#        #Clean up sawtooth artifact if present
#        trFilt = unfilt1D(tr,8)
#        bg = tr - trFilt
#        bgRm = bg
#        bgRm[bg>3*np.std(bg)] = 0
#        tr = tr - bg
        #add session and trial info to the dataframe
        newData.insert(0,'session',session)
        newData.insert(0,'date',date)
        newData.insert(0,'animalID',animalID)
        newData['trialType'] = trialTypes[idx]
        #add eyetrace to the new dataframe and append to full dataframe
        newData['eyetrace'] = tr
        frames = [data,newData]
        data = pd.concat(frames)
        print('_'.join(sessionInfo)+' '+str(idx))
    
    #clean up any duplicated values, re-do indexing
    #data = data.drop_duplicates(subset=['frame','eyetrace'])
    
    #%% pull pi Master-specific metadata
    csTime = float(headers['preCSdur'])#number of millis at which cs starts
    csusInt = float(headers['CS_USinterval'])#length of cs
    camFreq = 150#frames per second for picamera #TIMES 2 FOR MR. NYQUIST!!!!!
    pad = [500,800]#ms pad before and after cs
    timeBins = np.arange(-pad[0]+csTime,pad[1]+csTime+csusInt,1)
    #Need parser for CS v. US trials
    
    #%% making some summary plots
    #Initializing directory to save figures
    localDir = os.path.join(path,'Summary')
    if not os.path.exists(localDir):
            os.makedirs(localDir)
            
    #this is a single trial with its time series data
    uniTrials = data.trial.unique()
    colors = cm.jet(np.linspace(0,1,len(uniTrials)))
    
    #for holding clips of the eyetrace
    slices = np.zeros((len(uniTrials),len(timeBins)))
    #loop and plot individual trials
    pl.figure()
    for idx in range(len(uniTrials)):
        thisT = np.array(data['time'][data['trial']==uniTrials[idx]],dtype=float)
        thisEye = np.array(data['eyetrace'][data['trial']==uniTrials[idx]],dtype=float)
        if any(thisT>0):    
            interpEye = np.interp(timeBins,thisT[thisT>0],thisEye[thisT>0])
        else:
            interpEye = np.empty((len(timeBins)))
            interpEye[:] = np.nan
            print(idx)
        slices[idx] = interpEye
        pl.plot(timeBins-csTime,interpEye,linewidth=0.3,color=colors[idx])
    n,x = pl.ylim()
    pl.vlines(csusInt, n, x, linestyle='--', color='black')
    pl.vlines(0, n, x, linestyle='--', color='black')
    pl.xlabel('time from CS onset (ms)')
    pl.ylabel('Eyelid (a.u.)')
    pl.title('_'.join(sessionInfo))
    pl.tight_layout()
    pl.savefig(os.path.join(localDir,'_'.join(sessionInfo)+'traces.jpg'))
    
    #Show an average of the trial type of choice with std bars
    trialKinds = np.array(['CS_US','CS','US'])
    colors = np.array(['blue','red','green'])
    pl.figure()
    idx = 0
    legStr = []
    for kind in trialKinds:
        if not sum(trialTypes==kind)==0:
            subSlices = slices[trialTypes[uniTrials]==kind]
            mean = np.nanmean(subSlices,axis=0)
            err = np.nanstd(subSlices,axis=0)
            pl.plot(timeBins-csTime,mean,color=colors[trialKinds==kind][0])
            pl.fill_between(timeBins-csTime, mean-err, mean+err, alpha=.1, color=colors[trialKinds==kind][0], lw=0)
            legStr.append(kind)
            idx+=1
    n,x = pl.ylim()
    pl.vlines(csusInt, n, x, linestyle='--', color='black')
    pl.vlines(0, n, x, linestyle='--', color='black')
    pl.xlabel('time from CS onset (ms)')
    pl.ylabel('Average eyelid position (a.u.)')
    pl.title('_'.join(sessionInfo))
    pl.legend(legStr)
    pl.tight_layout()
    pl.savefig(os.path.join(localDir,'_'.join(sessionInfo)+'avgTraces.jpg'))
    
    #Make a heatmap of the time slices
    xAxis = [timeBins[0]-csTime,timeBins[-1]-csTime]
    pl.figure()
    imgHand = pl.imshow(slices[np.argsort(trialTypes[uniTrials])],\
                        cmap='Greys_r',extent=[xAxis[0],xAxis[1],len(uniTrials-1),0],aspect = 'auto')
    pl.colorbar()
    n,x = pl.ylim()
    pl.vlines(csusInt, n, x, linestyle='--', color='red')
    pl.vlines(0, n, x, linestyle='--', color='red')
    pl.xlabel('time from CS onset (ms)')
    pl.ylabel('trial number')
    pl.title('_'.join(sessionInfo))
    pl.tight_layout()
    pl.savefig(os.path.join(localDir,'_'.join(sessionInfo)+'tracesImg.jpg'))
    
    #If analyzing multiple files, close these figures
    #if len(subDirIdx)>1:
        #pl.close('all')
        
    #%% Save dataframes in this directory and in the head directory
    #Saving to local directory
    data.to_hdf(os.path.join(localDir,'_'.join(sessionInfo)+'data.h5'),key = 'df') #camera dataframe
    metadata.to_hdf(os.path.join(localDir,'_'.join(sessionInfo)+'meta.h5'),key = 'df') #metadata dataframe
    np.save(os.path.join(localDir,'_'.join(sessionInfo)+'roi.py'),roi)
    
    #Saving to summary directory
    data.to_hdf(os.path.join(headDir,'_'.join(sessionInfo)+'data.h5'),key = 'df') #camera dataframe
    metadata.to_hdf(os.path.join(headDir,'_'.join(sessionInfo)+'meta.h5'),key = 'df') #metadata dataframe
    np.save(os.path.join(headDir,'_'.join(sessionInfo)+'traces.npy'),slices)
    
    #Saving trialTypes data to the 2P folder for Ca imaging pipeline
    path2P = os.path.join(pathMaster,subDirs[thisSubDir],'2P data')
    trialTypes = np.array(trialTypes)
    np.savez(os.path.join(path2P,'_'.join(sessionInfo)+'trialTypes'),trialTypes)