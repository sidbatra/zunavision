/*****************************************************************************
** MOVIE PROJECT
** Copyright (c) 2008 
**
** FILENAME:    frameSampler.h
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Predicts which frames in the video stream are more conducive to ads
*****************************************************************************/


#include <stdio.h>
#include <cstdlib>
#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <string>
#include <vector>

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include <sys/types.h>
#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
#include "dirent.h"
#else
#include <dirent.h>
#endif


#include "svlSIFT.h"
#include "utilities.h"

using namespace std;

class frameSampler
{

private:
    //Data members

public:

    //Constuctor logic
    frameSampler();

    //Methods
    vector<int> sampleFrames(vector<string> frameNames);

private:

    void rankUsingSIFT( vector<string> frameNames , vector<int> &frameNameIndices);


};

