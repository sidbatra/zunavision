/*****************************************************************************
** ZunaVision
** Copyright (c) 2008 
**
** FILENAME:    plugRepeatFrames.cpp
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Plugs in the proper copies of the frames removed due to repeat frame analysis
*****************************************************************************/

#include <cstdlib>
#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include <sys/types.h>
#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
#include "dirent.h"
#else
#include <dirent.h>
#endif

#include "visionUtilities.h"


using namespace std;

#define NUM_REQUIRED_PARAMETERS 3


//Commandline instructions for application
void usage()
{
    cerr << "USAGE: ./plugRepeatFrames [OPTIONS] <source folder> <processed folder> <extension>";
	cerr << "OPTIONS:" << endl
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
    	//else
        //{
        cerr << "ERROR: unrecognized option " << *args << endl;
	    return -1;
        //}						
		args++;
	}//WHILE


	//Setup local variables 
    string sourceDirectory = string(args[0]); //Directory where all the video frames are stored
    string outputDirectory = string(args[1]); //Directory where all the processed video frames are stored
    string extension = string(args[2]); //Extension of the video frames
   
    //Create the output directory
    string repeatDirectory = sourceDirectory + "//repeatFrames";
    
    vector<string> filenames = utilities::loadFilenames(repeatDirectory,extension);
    
    //Iterate through all the repeat filenames 
    for( unsigned i=0 ; i<filenames.size() ; i++)
    {
        cout<<"\n Plugging frame "<<i;

        //Extract numeric frame ID from repeat filename
        string temp = filenames[i].substr(0 , filenames[i].size() - 4);
        int frameID = atoi(temp.c_str());
        string sourceID = utilities::padZeroes( utilities::toString(frameID-1) , (unsigned)temp.size());
        string targetID = utilities::padZeroes( utilities::toString(frameID) , (unsigned)temp.size());
        
        string sourceFilename = outputDirectory + "//" +  sourceID + extension;

        string targetFilename = outputDirectory + "//" +  targetID + extension;

        utilities::copyFile(sourceFilename,targetFilename);
        
    }

	//Default return value
    return 0;
}


