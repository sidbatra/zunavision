/*****************************************************************************
** ZunaVision
** Copyright (c) 2008 
**
** FILENAME:    trackCentral.h
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Manages the overall content tracking process
*****************************************************************************/
#pragma once

#include "base.h"
#include "video.h"
#include "contentTracker.h"
#include "renderingEngine.h"

using namespace std;


class trackCentral
{
private:
    contentTracker _contentTracker;         //Encapsulates all content region tracking functionalities
    renderingEngine _renderingEngine;       //Encapsulates all content rendering functionatilies

public:
    trackCentral();               //Default constructor

    void manageTracking(video &cVideo); //Manages the overall content tracking process
    void maintainVolume(video &cVideo , unsigned v); //Maintains the volume size needed for time based processing
    void outputVolume(video &cVideo , unsigned v);
    void wrapUpFrame(video &cVideo , unsigned v ,unsigned i); //Wraps up the main role of the frame by saving the meta information and optionally rendering it
    
    bool isValidForTracking(video &cVideo , unsigned v , unsigned c , unsigned f);    //Checks if the given content can be tracked in the given frame

};
