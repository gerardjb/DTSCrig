# -*- coding: utf-8 -*-
"""
Created on Thu Jan  2 17:30:16 2020

@author: wanglab
"""
#%% Libraries
import pandas as pd
import numpy as np
import matplotlib.pyplot as pl
import os
from matplotlib import cm

#%% List the available files
path = r'C:\Users\PNI User\Desktop\20200302 - Eyeblink J39-54\AllAnimals'
#Get all the file names
files = sorted([(d,f) for d,_,fs in os.walk(path) for f in fs if f.endswith('.npy')])
slice_files = [os.path.join(*i) for i in files]
meta_files = [os.path.join(d, os.path.splitext(f)[0].replace('traces','meta.h5')) for d,f in files]
sessionInfo = [x.split('\\')[-1].replace('traces.npy','') for x in slice_files]

#%% Construct lists containing np array for normalized slices, trial types, and sessions
#Get time bins
csTime = 2000#number of millis at which cs starts
csusInt = 300#length of cs
camFreq = 150#frames per second for picamera #TIMES 2 FOR MR. NYQUIST!!!!!
pad = [500,800]#ms pad before and after cs
timeBins = np.arange(-pad[0]+csTime,pad[1]+csTime+csusInt,1)
xAxis = [timeBins[0]-csTime,timeBins[-1]-csTime]
normIdx = timeBins<(csTime)

#Normalize each trace to its min in the pre-US period
normSlicesAg = []
trialTypesAg = []
sessionAg = []
animalNum = 0
animalKeys = {}
for sf,mf,si in zip(slice_files,meta_files,sessionInfo):
    #read session info
    thisSession = si.split('_')
    animalID = thisSession[0]
    date = thisSession[1]
    session = thisSession[2].replace('session','')
    
    #Check for new animalID
    if animalID in animalKeys:
        animalIdx = animalKeys[animalID]
    else:
        animalKeys[animalID] = animalNum
        animalIdx = animalNum
        normSlicesAg.insert(animalIdx,[])
        trialTypesAg.insert(animalIdx,[])
        sessionAg.insert(animalIdx,[])
        animalNum += 1
    
    #open metadata to get trial types
    metadata = pd.read_hdf(mf,'df')
    trialTypes = ('CS','US','CS_US')
    trialTypes = metadata['event'][metadata['event'].isin(trialTypes)].values
    trialTypesAg[animalIdx].extend(trialTypes)
    
    #Make and append a session identifier
    session = ''.join(c for c in session if c.isdigit())
    session2add = int(session)*np.ones(len(trialTypes))
    sessionAg[animalIdx].extend(session2add)
    
    #normalize slices to pre-US period min
    slices = np.load(sf)
    normSlice = np.zeros(np.shape(slices))
    avgSlice = np.mean(slices,axis=0)
    for a in range(np.shape(slices)[0]):
        normSlice[a] = (slices[a] - np.mean(slices[a,normIdx]))/\
        (np.max(avgSlice) - np.min(slices[a,normIdx]))
    normSlicesAg[animalIdx].extend(normSlice)


#%% Make plots, prepare to aggregate all CR trials together
#Prep array to hold percCR on a per session basis
#Get animal with largest number of sessions, then initialize NaN array
maxSession = max([max(p) for p in sessionAg])
allPercCR = np.empty((len(animalKeys),int(maxSession)))
allPercCR[:] = np.nan
allFracCR = np.empty((len(animalKeys),int(maxSession)))
allFracCR[:] = np.nan

for animal in animalKeys:
    animalIdx = animalKeys[animal]
    thisNslices = np.stack(normSlicesAg[animalIdx])
    thisSessAg = np.array(sessionAg[animalIdx],dtype=int)
    thisCSUStrials = np.array(trialTypesAg[animalIdx])=='CS_US'
    thisCStrials = np.array(trialTypesAg[animalIdx])=='CS'
    
    #Trial alignment figure
    allfig,allax = pl.subplots()
    imHand = allax.imshow(thisNslices[thisCSUStrials],\
                       extent=[xAxis[0],xAxis[1],sum(thisCSUStrials)-1,0],\
                       aspect='auto',cmap = 'Greys_r')
    cbar = pl.colorbar(imHand)
    imHand.set_clim(0.0,1.2)
    #Add graph labels
    cbar.set_label('Normalized eyelid position')
    allax.set_xlabel('time from CS onset (ms)')
    allax.set_ylabel('trial number')
    allax.set_title(animal + 'CS_US trials')
    #Add vertical lines at stims
    n,x = allax.get_ylim()
    allax.vlines(csusInt, n, x, linestyle='--', color=[1,0.5,0.5])
    allax.vlines(0, n, x, linestyle='--', color=[1,0.5,0.5])
    #add on horizontal line to demarcate sessions
    thisSessCSUS = thisSessAg[thisCSUStrials]
    thisSessCS = thisSessAg[thisCStrials]
    n,x = allax.get_xlim()
    cumTrials = 0
    colors = cm.jet(np.linspace(0,1,len(np.unique(thisSessCSUS))))
    coloridx = 0
    
    #Make figure to hold average traces, percent sig CR trials
    sumFig,sumax = pl.subplots()
    perFig,perax = pl.subplots()
    fracFig,fracax = pl.subplots()
    
    for i in np.unique(thisSessCSUS):
        #Setting up trial slices to work with
        CSUStrialThisSession = sum(thisSessCSUS==i)
        CStrialThisSession = sum(thisSessCS==i)
        theseSlices  = thisNslices[thisCSUStrials][thisSessCSUS==i]
        
        #Avg traces plot
        avgTrace = np.mean(theseSlices,axis=0)
        sumax.plot(timeBins-csTime,(avgTrace - np.min(avgTrace[0:100]))/\
                   (np.max(avgTrace) - np.min(avgTrace[0:100])),\
                   color= colors[coloridx])
        
        #Adding session number and lines to demarcate session borders to heatmap
        cumTrials += CSUStrialThisSession
        allax.hlines(cumTrials,n,x,linestyle='--',color=[0.5,0.5,1])
        allax.text(x,cumTrials,str(i),horizontalalignment='right',color='r')
        
        #Percent trials with CR, and average fraction CR per session
        CRthold = 0.15
        percCRthisSession = sum([np.interp(300,timeBins-csTime,a)>CRthold for a in theseSlices])/CSUStrialThisSession
        fracCRthisSession = sum([np.interp(300,timeBins-csTime,a) for a in theseSlices])/CSUStrialThisSession
        perax.plot(i,percCRthisSession,'o',color = colors[coloridx])
        fracax.plot(i,fracCRthisSession,'o',color = colors[coloridx])
        #Add this value into the aggregated totals for all animals
        allPercCR[animalIdx,i-1] = percCRthisSession
        allFracCR[animalIdx,i-1] = fracCRthisSession
        
        coloridx+=1
    #Finalizing heatmap ylim
    allax.set_ylim(cumTrials,0)
    
    #Labels for averaged traces plot
    sumax.legend(np.unique(thisSessCSUS),title='Session')
    n,x = sumax.get_ylim()
    sumax.vlines(csusInt,n,x,linestyle='--',color = 'k')
    sumax.vlines(0,n,x,linestyle='--',color = 'k')
    sumax.set_title(animal + 'CS_US trials')
    sumax.set_xlabel('time from CS onset (ms)')
    sumax.set_ylabel('Normalized average eyelid position')
    
    #labels for percent CR 
    perax.set_xlabel('Session number')
    perax.set_ylabel('percent trials with CR')
    perax.set_title(animal + 'Percent trials with CRs >' + str(CRthold))
    fracax.set_xlabel('Session number')
    fracax.set_ylabel('average CR amplitude')
    fracax.set_title(animal + 'average CR amplitude normalized to UR')
#%%Making plot with the average percCR per animal and error
pl.figure()
pl.plot(np.arange(1,maxSession+1),np.transpose(allPercCR),color='grey',linewidth=0.5)
pl.errorbar(np.arange(1,maxSession+1),np.nanmean(allPercCR,axis=0),\
            yerr=np.nanstd(allPercCR,axis=0),\
            color='black',linewidth=2)
pl.title('All animals, percent trials with CR>'+str(CRthold))
pl.xlabel('Session number')
pl.ylabel('percent trials with\n above threshold CR')

