/*****************************************************************************
** ZunaVision
** Copyright (c) 2008 
**
** FILENAME:    convertMetadata.cpp
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Converts video meta data from a previous version to the current version
*****************************************************************************/

#include <cstdlib>
#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>

#include "video.h"

using namespace std;

#define NUM_REQUIRED_PARAMETERS 4


//Commandline instructions for application
void usage()
{
    cerr << "USAGE: ./convertMetadata [OPTIONS] <video name> <video format> <Source metadata> <Output metaData>"<<endl;
    cerr << "OPTIONS:" << endl;
    cerr << "-fps <num>  :: FPS of the video, applied to all start frames"<< endl
	<< endl;
	 	 
}

void readOldFormat(string filename ,video &cVideo , double fps);


//Entry point for the application
int main(int argc, char *argv[])
{
	//If no filename is supplied flag error
	if( argc < NUM_REQUIRED_PARAMETERS )
	{
		usage();
		return -1;
	}

	//Pointer to the arguments after the EXE name
	char **args = argv + 1;
	double fps = 1.0;

	//Read all the options if any
    while (--argc > NUM_REQUIRED_PARAMETERS) 
	{	
        
		if(!strcmp(*args, "-fps"))
                	{fps = atof(*(++args)); argc--;}	
	   	else
        	{
		        cerr << "ERROR: unrecognized option " << *args << endl;
		        usage();
	  		return -1;
        	}
						
		args++;
	}//WHILE


	//Setup local variables 
    string videoName = string(args[0]); //Name of the video
    string videoFormat = string(args[1]); //Compression foramt of the actual video
    string sourceFilename = string(args[2]); //Input previous version of the metadata
    string targetFilename = string(args[3]); //Output converted new version of the metadata


    video cVideo;

    readOldFormat(sourceFilename , cVideo , fps);
    cVideo._name = videoName;
    cVideo._format = videoFormat;
    
    
    cVideo.write(targetFilename);
        
	//Default return value
    return 0;
}


//Reads the meta data from an older version 
void readOldFormat(string filename , video &cVideo , double fps)
{
    
	//Vectors for holding tracking input
	vector<svlPoint2d> polygon;
	int totalTracks = 0;
    char ext[5] , contentName[100] , contentURL[1000];
 
	//Open input stream for tracks
	ifstream inputStream(filename.c_str());
	
	inputStream>>totalTracks>>ext;
    cVideo._extension = string(ext);

    //Initialize vectors to hold the old data in a new format 
    cVideo._frames.clear();
    cVideo._frames.push_back(vector<frame>());
    cVideo._frames[0] = vector<frame>(totalTracks);
    cVideo._content = vector<content>(totalTracks);
    

	//Read each of the content entries
	for( int i=0 , f=0; i<totalTracks ; i++)
	{
        
        frame currentFrame;
        content currentContent;
        contentTracks currentContentTracks;

		//Get total points & starting frame
	int points = 0 , contentType = 0 , renderType = 0 , trackType =0 , occType = 0 , fadeType = -99;
	double startFrame = 0.0 , reverseEndFrame = 0.0 , endFrame = 0.0 ;

	inputStream>>points>>reverseEndFrame>>startFrame>>endFrame>>contentName>>contentURL>>ext>>contentType>>renderType>>trackType>>occType>>fadeType;

	startFrame *= fps;
        currentContent._startingFrame = (int)startFrame;

	if ( endFrame == -1 )
	        currentContent._endFrameForward = (int)endFrame;		
	else
	        currentContent._endFrameForward = (int)(endFrame * fps);

	if( reverseEndFrame == -1 )
	        currentContent._endFrameBackward = (int)reverseEndFrame;
	else
	        currentContent._endFrameBackward = (int)(reverseEndFrame * fps);
        
        currentContent._contentType = contentType;
        currentContent._trackingType = trackType;
        currentContent._occlusionType = occType;
        currentContent._renderingType = renderType;
	currentContent._fadeType = fadeType;
        currentContent._name = string(contentName);
        currentContent._extension = string(ext);
        
    
        currentContentTracks._polygon.clear();

		//Read each point of the current content
		for( int j=0 ; j<points;  j++)
		{
			double x =0 , y=0;
			inputStream>>x>>y;
			currentContentTracks._polygon.push_back(svlPoint2d(x,y));
     
		}

        currentContentTracks._contentID = i;
        currentFrame._contentTracks.push_back(currentContentTracks);
        currentFrame._index = (int)startFrame;


        cVideo._content[i] = currentContent;

        //Handle case where multiple content tracks are labeled onto one frame
        if( f > 0 && cVideo._frames[0][f-1]._index == currentFrame._index )
            cVideo._frames[0][f-1]._contentTracks.push_back(currentContentTracks);            
        else
            cVideo._frames[0][f++] = currentFrame;

    }//content i

    inputStream.close();
}

