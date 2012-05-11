/*****************************************************************************
** STAIR VISION PROJECT
** Copyright (c) 2007, Stanford University
**
** FILENAME:    images2video.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu> , Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
**  Application for creating video from images in a directory.
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

#include <sys/types.h>
#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
#include "dirent.h"
#else
#include <dirent.h>
#endif

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

//#include "svlCompatibility.h"
//#include "svlCore.h"
//#include "svlUtils.h"

using namespace std;

#define WINDOW_NAME "Video"

void usage()
{
    cerr << "USAGE: ./images2video [OPTIONS] <input dir> <output video>" << endl;
    cerr << "OPTIONS:" << endl
	 << "  -blanking <n>      :: number of initial blank frames" << endl
	 << "  -flip              :: flip frames horizontally" << endl
	 << "  -x                 :: display frames during encoding" << endl
	 << "  -ext               :: extension of the video frame images" << endl
	 << "  -fps               :: frames per second for the final video (default:29)" << endl
	 << endl;
}

int main(int argc, char *argv[])
{
    // read commandline parameters
    const int NUM_REQUIRED_PARAMETERS = 2;

    const char *inputDirectory = NULL;
    const char *outputFilename = NULL;
	char *extension = ".jpg";
    bool bDisplayImage = false;
    int blankingCount = 0;
    bool bFlipFrames = false;
	float fps = 29.0;

    char **args = argv + 1;
    while (--argc > NUM_REQUIRED_PARAMETERS) 
	{
				
	if (!strcmp(*args, "-x")) {
	    bDisplayImage = true;
	} else if (!strcmp(*args, "-blanking")) {
	    blankingCount = atoi(*(++args)); argc -= 1;
	}
	else if (!strcmp(*args, "-fps")) {
	    fps = atof(*(++args)); argc -= 1;
	
	}
	else if (!strcmp(*args, "-ext")) {
	    extension = *(++args); argc -= 1;
	}else if (!strcmp(*args, "-flip")) {
	    bFlipFrames = true;
	} else {
	    cerr << "ERROR: unrecognized option " << *args << endl;
	    return -1;
	}
	args++;
    }

    if (argc != NUM_REQUIRED_PARAMETERS) {
	usage();
	return -1;
    }

    inputDirectory = args[0];
    outputFilename = args[1];

    if (bDisplayImage) {
	cvNamedWindow(WINDOW_NAME, 1);
    }

    CvVideoWriter *videoWriter = NULL;

    // create sorted list of image filenames
    vector<string> imageFilenames;
    DIR *dir = opendir(inputDirectory);
    assert(dir != NULL);
    struct dirent *e = readdir(dir);
    while (e != NULL) {
	if (strstr(e->d_name, extension) != NULL) {
	    imageFilenames.push_back(string(e->d_name));	    
	}
	e = readdir(dir);
    }
    closedir(dir);
    sort(imageFilenames.begin(), imageFilenames.end());


    for (vector<string>::const_iterator it = imageFilenames.begin();
	 it != imageFilenames.end(); ++it) {
	// load image
	string filename = string(inputDirectory).append(string("/")).append(*it);
	printf("\n%s", filename.c_str());
	IplImage *frame = cvLoadImage(filename.c_str());
	if (bFlipFrames) 
	    cvFlip(frame, NULL, 1);

	// show image
	if (bDisplayImage) {
	    cvShowImage(WINDOW_NAME, frame);
	    cvWaitKey(30);
	}
	// encode image
	if (videoWriter == NULL) {
#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
	    videoWriter = cvCreateVideoWriter(outputFilename, -1, fps, cvSize(frame->width, frame->height));
#else
	    videoWriter = cvCreateVideoWriter(outputFilename,CV_FOURCC('D','I','V','X'), fps, cvSize(frame->width, frame->height));
#endif
	}
	
	if (videoWriter == NULL) {
	    cerr << "ERROR: could not open AVI file for writing " << outputFilename << endl;
	    exit(-1);
	}
	
	if (blankingCount > 0) {
	    IplImage *blankFrame = cvCloneImage(frame);
	    cvZero(blankFrame);
	    while (blankingCount-- > 0) {
		cvWriteFrame(videoWriter, blankFrame);
	    }
	    cvReleaseImage(&blankFrame);
	}

	cvWriteFrame(videoWriter, frame);
	cvReleaseImage(&frame);
    }
   
    // free memory
    if (bDisplayImage) {
        cvDestroyWindow(WINDOW_NAME);
    }

    cvReleaseVideoWriter(&videoWriter);
    
    return 0;
}

