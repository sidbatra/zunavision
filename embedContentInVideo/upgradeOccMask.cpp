/*****************************************************************************
** ZunaVision
** Copyright (c) 2008 
**
** FILENAME:    upgradeOccMask.cpp
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Removes the surronding grey pixels from all older occ masks
*****************************************************************************/

#include "base.h"
#include "video.h"

using namespace std;

#define NUM_REQUIRED_PARAMETERS 2

//Function prototype
void upgradeMask(IplImage **occMask,vector<svlPoint2d> targetPolygon);

//Commandline instructions for application
void usage()
{
    cerr << "USAGE: ./upgradeOccMask [OPTIONS] <meta data directory> <meta data file>";
	cerr << "OPTIONS:" << endl
    << " -v            :: verbose "<<endl
    << endl;
	 	 
}

//Entry point for the application
int main(int argc, char *argv[])
{
	//If no foldername is supplied flag error
	if( argc < NUM_REQUIRED_PARAMETERS )
	{
		usage();
		return -1;
	}

	//Pointer to the arguments after the EXE name
	char **args = argv + 1;
                
	//Read all the options if any
    while (--argc > NUM_REQUIRED_PARAMETERS) 
	{	    
        if(!strcmp(*args, "-v")) 
        {constants::VERBOSE = true;}
        else
        {cerr << "ERROR: unrecognized option " << *args << endl;return -1;}						
		args++;
	}//WHILE


	//Setup local variables 
    string metaDataDirectory = string(args[0]); //Directory where all the meta information is placed
    string metaDataFilename = string(args[1]); //Filename for the video metadata file
   
    video cVideo;   
    frame frameObject;
        
    cVideo.read(metaDataFilename , "", "" , metaDataDirectory,false); //Read all the meta data information
        
    int index = 0;

    //Reads every frame from the meta data file & folder
    while(true)
    {         
         frameObject = cVideo.readFrameSeq(index);

         if( frameObject._index == -1 )
             break;

         if( constants::VERBOSE )
            cout<<"\n Modifying mask for frame "<<frameObject._index;
        
         for( unsigned i=0 ; i<frameObject._contentTracks.size() ; i++)
             if( frameObject._contentTracks[i]._occlusionMask != NULL )
             {              
                 upgradeMask(&(frameObject._contentTracks[i]._occlusionMask),frameObject._contentTracks[i]._polygon);
                 frameObject.replaceOccMask(string(metaDataDirectory),i);
             }

        
        index++;
        frameObject.freeMemory();
    }

	//Default return value
    return 0;
}


//Upgrades the occlusion mask
void upgradeMask(IplImage **occMask,vector<svlPoint2d> targetPolygon)
{
    
    //Compute bounding rectangle to the content
    double minX , maxX , minY , maxY;
    visionUtilities::findEnclosingRectangle(targetPolygon,minX,minY,maxX,maxY);
    minX -= constants::CONTENT_MARGIN ; minY -= constants::CONTENT_MARGIN;
    maxX += constants::CONTENT_MARGIN ; maxY += constants::CONTENT_MARGIN;

    //Compute discrete height width of the region in the image which is to be replaced with the content
    int discreteX = (int)floor(minX);
    int discreteY = (int)floor(minY);
    int discreteWidth  = (int)(ceill(maxX) - floor(minX) + 1) + discreteX;
    int discreteHeight = (int)(ceill(maxY) - floor(minY) + 1) + discreteY;
               
    for( int y=0 ; y<(*occMask)->height ; y++)
    {
        for( int x=0 ; x<(*occMask)->width ; x++)
        {  
            if( !( y >= discreteY && y< discreteHeight && x >= discreteX && x < discreteWidth) )
                CV_IMAGE_ELEM(*occMask,unsigned char,y,x) = 0;
        }
    }    
    
}

