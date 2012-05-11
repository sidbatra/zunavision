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
    cerr << "USAGE: ./images2video [OPTIONS] <input dir> <output video>" << endl;
    cerr << "OPTIONS:" << endl
        << "  -ext               :: extension of the video frame images (Default:.jpg)" << endl
	<< "  -fps               :: frames per second for the final video (Default:29)" << endl
	<< " -type <num>	 :: extension of the final video (Default: DivX)"<<endl
	<<" -scale <float>	 :: scale of the frames in the final video (Default: 1)"<<endl
    << " -v              :: verbose "<<endl
	 << endl;
}

int main(int argc, char *argv[])
{
    const int NUM_REQUIRED_PARAMETERS = 2;

    //Setup variables to read command line arguments
    const char *inputDirectory = NULL;
    const char *outputFilename = NULL;
    char *extension = ".jpg";
    float fps = 29.0;
    int type = 0;
    double scale = 1;

    //Read the command line options
    char **args = argv + 1;
    while (--argc > NUM_REQUIRED_PARAMETERS) 
	{    				
	    if (!strcmp(*args, "-fps")) 
        {fps = (float)atof(*(++args)); argc -= 1;}
	 else if (!strcmp(*args, "-type")) 
        {type = atoi(*(++args)); argc -= 1;}
	 else if (!strcmp(*args, "-scale")) 
        {scale = atof(*(++args)); argc -= 1;}
	    else if (!strcmp(*args, "-ext")) 
        {extension = *(++args); argc -= 1;}
         else if(!strcmp(*args, "-v")) 
        {constants::VERBOSE = true;}
        else 
        {
	        cerr << "ERROR: unrecognized option " << *args << endl;
	        return -1;
	    }
	    args++;
    }

    if (argc != NUM_REQUIRED_PARAMETERS) 
    {usage();return -1;}

    //Load required arguments
    inputDirectory = args[0];
    outputFilename = args[1];

    
    CvVideoWriter *videoWriter = NULL;

    cout<<"\n Initializing video stream \n";

    //Create sorted list of frame filenames
    vector<string> filenames = utilities::loadFilenames(string(inputDirectory),extension);
    
    for(unsigned i=0 ; i<filenames.size() ; i++) 
    {
	    //Load image
	    string filename = string(inputDirectory) + "//" + filenames[i];

        if( constants::VERBOSE )
            cout<<"\n Writing to video frame - "<<filename.c_str();
	    
        IplImage *frame = cvLoadImage(filename.c_str());

	if( scale != 1 )
		utilities::resizeInPlace(&frame,scale);
		
	    //Initialize video writer with info from first frame
	    if (videoWriter == NULL) 
        {
            #if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
	            videoWriter = cvCreateVideoWriter(outputFilename, -1, fps, cvSize(frame->width, frame->height));
            #else
		if( type == 0 )
	            videoWriter = cvCreateVideoWriter(outputFilename,CV_FOURCC('D','I','V','X'), fps, cvSize(frame->width, frame->height));
		else if(type == 1)
	            videoWriter = cvCreateVideoWriter(outputFilename,CV_FOURCC('F','L','V','1'), fps, cvSize(frame->width, frame->height));
            #endif
	    }
	
	    if (videoWriter == NULL) 
        {cerr << "ERROR: could not open AVI file for writing " << outputFilename << endl;exit(-1);}
	
    	cvWriteFrame(videoWriter, frame);
	    cvReleaseImage(&frame);
    }
    
    cvReleaseVideoWriter(&videoWriter);

    cout<<"\n Close video stream \n";
    
    return 0;
}

