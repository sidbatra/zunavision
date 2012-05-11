/*****************************************************************************
** MOVIE PROJECT
** Copyright (c) 2008
**
** FILENAME:    video2images.cpp
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
**  Extracts frames from given video to folder
**
*****************************************************************************/

#include <cstdlib>
#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>
#include <vector>
#include <algorithm>
#include <sstream>
#include <map>

#include <sys/types.h>
#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
#include "dirent.h"
#else
#include <dirent.h>
#endif

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "utilities.h"

using namespace std;


void usage()
{
    cerr << "USAGE: ./video2images [OPTIONS] <input video> <output dir>" << endl;
    cerr << "OPTIONS:" << endl
	     << "  -size <num>               :: scales width to match the given number, maintains aspect ratio" << endl
	 
	 << endl;
}



int main(int argc, char *argv[])
{
    // read commandline parameters
    const int NUM_REQUIRED_PARAMETERS = 2;

    const char *inputFilename = NULL;
    const char *outputDirectory = NULL;
    double width = -1;
	
    char **args = argv + 1;
    while (--argc > NUM_REQUIRED_PARAMETERS) 
	{				
    	if (!strcmp(*args, "-size")) 
        {
    	    width = atoi(*(++args)); argc -= 1;
    	}
	    args++;
    }

    if (argc != NUM_REQUIRED_PARAMETERS) {
	usage();
	return -1;
    }

    inputFilename = args[0];
    outputDirectory = args[1];
    const int MIN_FILENAME_LENGTH = 7;

    cout<<"\n\n Extracting frames to folder"<<outputDirectory;
    
    //Open stream to video
    CvCapture* capture = cvCaptureFromAVI(inputFilename);

    //Extract properties of given video
    cvQueryFrame(capture); 

    int frameH    = (int) cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT);
    int frameW    = (int) cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH);
    int fps       = (int) cvGetCaptureProperty(capture, CV_CAP_PROP_FPS);
    int numFrames = (int) cvGetCaptureProperty(capture,  CV_CAP_PROP_FRAME_COUNT);
    double scale = 1.0;



    if( width != -1 )
    {
        if( frameW < width )
            scale = (width + 0.0 ) / frameW;
    }

    cout<<"\n\nH W FPS NUM "<<frameH<<" "<<frameW<<" "<<fps<<" "<<numFrames;    
	cout<<"\n\nScale "<<scale<<" "<<frameW;    


    int index = 0;

    //Save each frame to target folder
    while(true)
    {
        //Grab frame from video
        IplImage* temp = cvQueryFrame(capture); 

        if( temp == NULL )
            break;

        string name = utilities::toString(index++);

        //Addp adding zeroes
        while( name.size() < MIN_FILENAME_LENGTH )
            name = "0" + name ;
            
          name = string(outputDirectory) + "//" + name + ".jpg";

          cout<<"\nSaving image "<<name.c_str();
          
          if( scale != 1 )
              utilities::resizeInPlace(&temp,scale);
          //Save video frame
          cvSaveImage(name.c_str(),temp);

          if( scale != 1 )
          cvReleaseImage(&temp);

        
    }

    //Free stream to the AVI file
    cvReleaseCapture(&capture);
        
    return 0;
}


