/*****************************************************************************
** ZunaVision
** Copyright (c) 2008 
**
** FILENAME:    renderContent.cpp
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Render the final frames of the video based on the meta data generated by trackContent
*****************************************************************************/

#include "base.h"
#include "renderingEngine.h"


using namespace std;

#define NUM_REQUIRED_PARAMETERS 3


//Commandline instructions for application
void usage()
{
    cerr << "USAGE: ./renderContent [OPTIONS] <base folder> <content directory> <meta data file>";
	cerr << "OPTIONS:" << endl
    << "  -md <string> :: input foldername for the supporting meta data (images etc)" << endl
    << " -blanks       :: renders the blank frames which have no tracked information "<<endl
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
    
    string metaDataFolder = "";
    bool renderBlanks = false;
    
	//Read all the options if any
    while (--argc > NUM_REQUIRED_PARAMETERS) 
	{	    
        if(!strcmp(*args, "-md")) 
		{metaDataFolder = string(*(++args)); argc--;}
        else if(!strcmp(*args, "-blanks")) 
        {renderBlanks = true;}
        else if(!strcmp(*args, "-v")) 
        {constants::VERBOSE = true;}
        else
        {cerr << "ERROR: unrecognized option " << *args << endl;return -1;}						
		args++;
	}//WHILE


	//Setup local variables 
    string mainDirectory = string(args[0]); //Directory where all the videos and results are kept
    string contentDirectory = string(args[1]); //Directory where content is kept
    string metaDataFilename = string(args[2]); //Filename for the video metadata file
   
    video cVideo;   
    renderingEngine cRenderingEngine;   



    
    cVideo.read(metaDataFilename , mainDirectory, contentDirectory , metaDataFolder,false); //Read all the meta data information

    
    //Renders every frame read from the meta data file
    for( unsigned i=0 ; i<cVideo._filenames.size() ; i++)
    {        
            

        //Renders frame only if meta data is availible
        if( cVideo.isFrameAvailible(i) )
        {
                
            cVideo._frames.push_back(vector<frame>());
            cVideo._frames[0].push_back(cVideo.readFrame(i,true));    //Load actual images into memory
                
            //Render and save frame based on meta data information
            cRenderingEngine.renderFrame(cVideo,0,0);
            cVideo.saveFrame(0,0);
            
            //Free memory
            cVideo._frames[0][0].freeMemory();
            cVideo._frames.clear();
        }
        else if(renderBlanks)
        {
            //Save frame without any surface
            utilities::copyFile(cVideo.fullInputFilename(i),cVideo.fullOutputFilename(i));            
        }
    }

	//Default return value
    return 0;
}

