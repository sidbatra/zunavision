/*****************************************************************************
** ZunaVision
** Copyright (c) 2008 
**
** FILENAME:    trackContent.cpp
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Tracks the content regions in the video based on the meta data information
*****************************************************************************/

#include "base.h"
#include "trackCentral.h"


using namespace std;

#define NUM_REQUIRED_PARAMETERS 3


//Commandline instructions for application
void usage()
{
    cerr << "USAGE: ./trackContent [OPTIONS] <base folder> <content folder> <meta data file>";
	cerr << "OPTIONS:" << endl
    << "  -o <string>       :: output filename for the meta data" << endl
    << "  -md <string>      :: output foldername for the supporting meta data (images etc)" << endl
    << "  -scale <double>   :: scale at which the frames are tracked (Default : 1.0)" << endl
    << "  -render           :: calls the rendering engine to render and save the final files" << endl
    << "  -reverse          :: tracks the content regions in reverse mode" << endl
    << "  -flashMode        :: renders all meta data in the flash player format" << endl
    << " -v                 :: verbose "<<endl

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
    string outputFilename = "";
    string metaDataFolder = "";
    bool render = false;
    bool flashMode = false;
    bool isReverse = false;
    double scale = constants::DEFAULT_TRACK_SCALE;

	//Read all the options if any
    while (--argc > NUM_REQUIRED_PARAMETERS) 
	{	
        if(!strcmp(*args, "-o")) 
		{outputFilename = string(*(++args)); argc--;}
        else if(!strcmp(*args, "-md")) 
		{metaDataFolder = string(*(++args)); argc--;}
        else if(!strcmp(*args, "-scale")) 
		{scale = atof(*(++args)); argc--;}
        else if(!strcmp(*args, "-render")) 
		{render = true; }
        else if(!strcmp(*args, "-flashMode")) 
		{flashMode = true; }
        else if(!strcmp(*args, "-reverse")) 
        {isReverse = true; }
        else if(!strcmp(*args, "-v")) 
        {constants::VERBOSE = true;}
    	else
        {cerr << "ERROR: unrecognized option " << *args << endl;return -1;}						
		args++;
	}//WHILE


	//Setup local variables 
    string mainDirectory = string(args[0]); //Directory where all the videos and results are kept
    string contentDirectory = string(args[1]); //Directory where the content files are stored
    string metaDataFilename = string(args[2]); //Filename for the video metadata file
   
    video cVideo;
    
    cVideo._outputFilename = outputFilename;
    cVideo._render = render;
    cVideo._flashMode = flashMode;
    cVideo._reverse = isReverse;
    cVideo._scale = scale;
    cVideo.read(metaDataFilename , mainDirectory, contentDirectory , metaDataFolder); //Read all the meta data information
    
    trackCentral cTrackCentral;
    cTrackCentral.manageTracking(cVideo);

	//Default return value
    return 0;
}


