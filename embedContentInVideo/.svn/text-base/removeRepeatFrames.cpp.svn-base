/*****************************************************************************
** ZunaVision
** Copyright (c) 2008 
**
** FILENAME:    removeRepeatFrames.cpp
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Removes all the repeat frames from a video and stashes them in a sub folder
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

#define NUM_REQUIRED_PARAMETERS 2


//Commandline instructions for application
void usage()
{
    cerr << "USAGE: ./removeRepeatFrames [OPTIONS] <source folder> <extension>";
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
    string extension = string(args[1]); //Extension of the video frames
   
    //Create the output directory
    string outputDirectory = sourceDirectory + "//repeatFrames";
    utilities::createDirectory(outputDirectory.c_str());

    vector<string> filenames = utilities::loadFilenames(sourceDirectory,extension);

    //Load the first frame
    string filename = sourceDirectory + "//" + filenames[0];
    IplImage *previousFrame = cvLoadImage(filename.c_str(),CV_LOAD_IMAGE_COLOR);
    cvCvtColor(previousFrame,previousFrame,CV_RGB2Lab);

    //Iterate through all the filenames looking for repeat frames
    for( unsigned i=1 ; i<filenames.size() ; i++)
    {
        cout<<"\n Testing frame "<<i;

        string filename = sourceDirectory + "//" + filenames[i];
        
        IplImage *currentFrame = cvLoadImage(filename.c_str(),CV_LOAD_IMAGE_COLOR);
        cvCvtColor(currentFrame,currentFrame,CV_RGB2Lab);
        
        if( visionUtilities::isRepeatFrame(previousFrame,currentFrame) )
        {
            cout<<" Repeat frame found";

            string currentFile = sourceDirectory + "//" + filenames[i];
            string targetFile = outputDirectory + "//" + filenames[i];
            utilities::moveFile(currentFile.c_str() , targetFile.c_str());
        }

        //Update previous image and free memory
        cvReleaseImage(&previousFrame);
        previousFrame = cvCloneImage(currentFrame);
        cvReleaseImage(&currentFrame);
    }
    

	//Default return value
    return 0;
}


