/*****************************************************************************
** ZunaVision
** Copyright (c) 2008 
**
** FILENAME:    video2images.cpp
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Decompiles videos to images
*****************************************************************************/

#include "base.h"

using namespace std;


void usage()
{
    cerr << "USAGE: ./video2images [OPTIONS] <input video> <output dir>" << endl;
    cerr << "OPTIONS:" << endl
        << "  -ext <string>    :: Extension of the output video frames (Default : .jpg)" << endl
        << " -v                :: verbose "<<endl
	 
	 << endl;
}


int main(int argc, char *argv[])
{
    // read commandline parameters
    const int NUM_REQUIRED_PARAMETERS = 2;

    //Setup argument holding variables
    const char *inputFilename = NULL;
    const char *outputDirectory = NULL;
    const char *extension = ".jpg";
    
	//Read command line options
    char **args = argv + 1;
    while (--argc > NUM_REQUIRED_PARAMETERS) 
	{				
    	if (!strcmp(*args, "-ext")) 
        {extension = *(++args); argc --;}
        else if(!strcmp(*args, "-v")) 
        {constants::VERBOSE = true;}

	    args++;
    }

    if (argc != NUM_REQUIRED_PARAMETERS) 
    {usage();return -1;}

    inputFilename = args[0];
    outputDirectory = args[1];

    const int MIN_FILENAME_LENGTH = 7;
    int index = 0;


   cout<<"\n\n Extracting video "<<inputFilename<<" to folder "<<outputDirectory<<"\n";
    
    //Open stream to video
    CvCapture* capture = cvCaptureFromAVI(inputFilename);
   

    //Save each frame to target folder
    while(true)
    {
        //Grab frame from video
        IplImage* temp = cvQueryFrame(capture); 

        if( temp == NULL )
            break;

        //Display video stats
        if( constants::VERBOSE && index == 0 )
        {
                int frameH    = (int) cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT);
                int frameW    = (int) cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH);
                int fps       = (int) cvGetCaptureProperty(capture, CV_CAP_PROP_FPS);
                int numFrames = (int) cvGetCaptureProperty(capture,  CV_CAP_PROP_FRAME_COUNT);
                
		cout<<"\n ** Video Stats ** \n";
                cout<<"\n\nHeight WWidth FPS NUM "<<frameH<<" "<<frameW<<" "<<fps<<" "<<numFrames; 
        }

        
        string name = utilities::padZeroes(utilities::toString(index++),MIN_FILENAME_LENGTH);
        
        name = string(outputDirectory) + "//" + name + string(extension);
        
        if ( constants::VERBOSE )
            cout<<"\nSaving image "<<name.c_str();
        
        //Save video frame
        cvSaveImage(name.c_str(),temp);        
    }

    //Free stream to the AVI file
    cvReleaseCapture(&capture);

    if ( constants::VERBOSE )
    	cout<<"\n Closing stream to video \n";
        
    return 0;
}


