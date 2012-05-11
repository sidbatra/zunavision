/*****************************************************************************
** ZunaVision
** Copyright (c) 2008 
**
** FILENAME:    trackCentral.cpp
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Manages the overall content tracking process
*****************************************************************************/

#include "trackCentral.h"

trackCentral::trackCentral()
{
    //Does nothing as of now
}


//Manages the overall content tracking process
void trackCentral::manageTracking(video &cVideo)
{       
    
    cVideo.writeInitial();   //Write initial video & content meta data


    //Iterate over each volume
    for(unsigned v=0 ; v<cVideo._videoVolumes.size() ; v++)
    {
        cout<<"\n\n Processing volume "<<v;

        //While volume has frames in it
        while( cVideo._videoVolumes[v]._stubCounter <= constants::VOLUME_MARGIN )
        {
            unsigned f = cVideo._frames[v][constants::VOLUME_MARGIN * 2]._index;

            cout<<"\n Processing frame "<<f;

            //Iterate over and track all the valid content regions
            for( unsigned c=0 ; c<cVideo._content.size() ; c++)
            {
                //If content region is valid and trackable
                if( isValidForTracking(cVideo ,v ,  c , f) )
                {          
                   _contentTracker.trackContent(cVideo,v,c); //Tracks the content region and updates the meta information in the video                             
                }
            }

            //Wrap up the finished frames
            if( cVideo._videoVolumes[v]._totalFrames >= cVideo._frames[v].size() )
                outputVolume(cVideo,v);    
            
            maintainVolume(cVideo,v); 

            cVideo._videoVolumes[v].computeNextFrameIndex(cVideo._reverse); //Compute next index in volume
                        
        }//while


    }//v

   cVideo.writeFinal(); //Write final video data and close filestream 

   cVideo.freeMemory(); //Free memory used by the video objects

}

//Maintains the volume size needed for time based processing
void trackCentral::maintainVolume(video &cVideo , unsigned v)
{
    cVideo._frames[v][0].freeMemory();
    cVideo._frames[v].erase( cVideo._frames[v].begin() );

     //Add fresh frames to maintain constant size of volume
    if( !cVideo._videoVolumes[v]._isFinished )
        cVideo._frames[v].push_back( cVideo.readFrame(cVideo._videoVolumes[v]._currentIndexInPart ) );
    else
    {            
        //Add stub frame since actual frames are over
        frame tempFrame;
        tempFrame._index = -1;
        cVideo._frames[v].push_back( tempFrame);
    }
}

//Wraps up the ready frames
void trackCentral::outputVolume(video &cVideo , unsigned v)
{
    //Wrap up initial frames left out because of volume requirements
    if( cVideo._videoVolumes[v]._totalFrames == cVideo._frames[v].size() )
    {
        for( unsigned i=0 ; i<constants::VOLUME_MARGIN ; i++ )
            wrapUpFrame(cVideo,v,i);
    }

    wrapUpFrame(cVideo,v,constants::VOLUME_MARGIN);

    //Wrap up final frames
    if( cVideo._videoVolumes[v]._stubCounter == constants::VOLUME_MARGIN )
    {
        for( unsigned i=constants::VOLUME_MARGIN+1 ; i<constants::VOLUME_MARGIN * 2 + 1 ; i++)
            wrapUpFrame(cVideo,v,i);
    }
}

//Wraps up the main role of the frame by saving the meta information and optionally rendering it
void trackCentral::wrapUpFrame(video &cVideo , unsigned v , unsigned i)
{
    cVideo.writeFrame(v,i);
                
    if( cVideo._render )
    {
        _renderingEngine.renderFrame(cVideo,v,i);
        cVideo.saveFrame(v,i);
    }
}

//Checks if the given content can be tracked in the given frame
bool trackCentral::isValidForTracking(video &cVideo , unsigned v, unsigned c , unsigned f)
{
    bool isTrackable = false;
  
    if( cVideo._reverse )
    {        
        if(  cVideo._content[c]._startingFrame >= f && (int)f > cVideo._content[c]._endFrameBackward + 1 && cVideo._videoVolumes[v].isWithinVolume(cVideo._content[c]._startingFrame) )
            isTrackable = true;
    }
    else
    {
        if( cVideo._content[c]._startingFrame <= f && (int)f < cVideo._content[c]._endFrameForward -1 && cVideo._videoVolumes[v].isWithinVolume(cVideo._content[c]._startingFrame) )
            isTrackable = true;
    }
  
    return isTrackable;
}



