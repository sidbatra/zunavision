/*****************************************************************************
** MOVIE PROJECT
** Copyright (c) 2008 
**
** FILENAME:    frameSampler.h
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Predicts which frames in the video stream are more conducive to ads
*****************************************************************************/


#include "base.h"
#include "sift.h"


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

