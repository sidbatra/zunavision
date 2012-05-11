/*****************************************************************************
** MOVIE PROJECT
** Copyright (c) 2008 
**
** FILENAME:    frameSampler.cpp
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Implementation of functions in frameSampler.cpp
*****************************************************************************/

#include "frameSampler.h"


//************** Constructor Logic **********

frameSampler::frameSampler()
{
    //Does nothing as of now
}


//************** Methods ***************

//Samples and sorts based on different metrics predicting how well a frame might 
//host planar content
vector<int> frameSampler::sampleFrames(vector<string> frameNames)
{
    const int FRAME_RATE = 10; //Initially sample every Nth frame

    vector<string> sampledFrames;
    vector<int> sampledFrameIndices;

    for( unsigned i=0 ; i<frameNames.size() ; i+= FRAME_RATE)
    {
        sampledFrames.push_back(frameNames[i]);
        sampledFrameIndices.push_back(i);
    }

    //rankUsingSIFT(sampledFrames,sampledFrameIndices);


    //Return the sorted sampled frames
    return sampledFrameIndices;
}


//Ranks sampled frames based on SIFT features
void frameSampler::rankUsingSIFT( vector<string> frameNames , vector<int> &frameNameIndices)
{
    vector<int> totalSIFTMatches(frameNames.size());

    sift svlSIFTObject;

    //Compute total SIFT features for each frame
    for( int i=0 ; i<(int)frameNames.size() ; i++)
    {
        //Load the sampled frame
        IplImage *temp = cvLoadImage(frameNames[i].c_str(),CV_LOAD_IMAGE_COLOR);

        //Store total SIFT features on frame
        totalSIFTMatches[i] = 0;// svlSIFTObject.getTotalSIFTFeatures(temp);

        //Free memory
        cvReleaseImage(&temp);
    }

     //Sort the frames indices based on total SIFT matches
        for( unsigned i=0 ; i<frameNameIndices.size() ; i++)
        {
  	            for(unsigned j=i+1 ; j<frameNameIndices.size() ; j++)
	            {
		            if(totalSIFTMatches[i] > totalSIFTMatches[j])
		            {
			            int temp = totalSIFTMatches[i];
			            totalSIFTMatches[i] = totalSIFTMatches[j];
			            totalSIFTMatches[j] = temp;
                        
                        temp = frameNameIndices[i];
			            frameNameIndices[i] = frameNameIndices[j];
			            frameNameIndices[j] = temp;

		            }
	            }//j
        }//i
}
