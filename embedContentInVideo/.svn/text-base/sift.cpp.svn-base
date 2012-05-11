/*****************************************************************************
** ZunaVision
** Copyright (c) 2008 
**
** FILENAME:    sift.cpp
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Manages computation matching and sampling of SIFT features
*****************************************************************************/

#include "sift.h"

int CUSTOM_KEY_POOL = -1;                   //Custom index to manage memory being allocated to SIFT features of each frame

//Default constructor
sift::sift()
{

}


//Samples SIFT features based upon different correspondence metrics
CvMat* sift::sampleSIFTFeatures(vector<vector<vector<svlPoint2d> > > &correspondences, contentTracks &currentTracks ,int siftIndex , int sampleIndex)
{	
    
    //************************** Weights ******************************

    //Weights for different particles
    //Quality , Velocity / inframe distance , Distance from centre
    const double w_type[][3] = 
    { 
        {0.2998 , 0.7 , 0.0002} , 
        {0.7 , 0.3 , 0.0} , 
        {0.2 , 0.8 , 0.0} ,
        {0.39998 , 0.6 , 0.00002} ,
        {0.45 , 0.45 , 0.00002},
        {0.2 , 0.4 , 0.002}           
    };
    
    //const double ts_type[] = {.25 , .75 , 1};
    const double ts_type[] = {.2 , .6 , .85};//{.2 , .6 , 1};

    //*******************************************************************
   
    vector<vector<svlPoint2d> > matchPairs = correspondences[0];
        
    vector<double> finalWeights(matchPairs.size());
    double weights[3];
    unsigned totalSamples = (int)(matchPairs.size() * ts_type[sampleIndex]);
        
    //Selection of parameters based on requested type value
    for( int i=0 ; i<3 ; i++)
        weights[i] = w_type[siftIndex][i];
    
    //Increase the total number of samples if the sampling reduces the number below the mnimum number of SIFT features
     if( totalSamples < constants::MINIMUM_SIFT_FEATURES )
        totalSamples = constants::MINIMUM_SIFT_FEATURES;

     

    //Compute weights for each correspondence pairing
	for( unsigned j=0 ; j<matchPairs.size() ; j++)
	{
        double final = weights[0] * _featureSpaceRatios[j] + weights[2] * _imageDistances[j];

        //Use distance from median in old SIFT metric not in the new one
        if( currentTracks._H != NULL )
            final += weights[1] * _actualDistances[j];
        else
            final += weights[1] * fabs(_actualDistances[j] - _actualDistanceMedian);
                      
        finalWeights[j] = final;
                
    }


    //Sort the match pairs based on the value of the final weight
	for( unsigned i=0 ; i<matchPairs.size() ; i++)
	{
		  	for(unsigned j=i+1 ; j<matchPairs.size() ; j++)
			{
				if(finalWeights[i] > finalWeights[j])
				{
					double temp = finalWeights[i];
					finalWeights[i] = finalWeights[j];
					finalWeights[j] = temp;

                    matchPairs[i].swap( matchPairs[j] );

                     temp = _featureSpaceRatios[i];
					_featureSpaceRatios[i] = _featureSpaceRatios[j];
					_featureSpaceRatios[j] = temp;

                     temp = _actualDistances[i];
					_actualDistances[i] = _actualDistances[j];
					_actualDistances[j] = temp;

                    temp = _imageDistances[i];
					_imageDistances[i] = _imageDistances[j];
					_imageDistances[j] = temp;
                    
				}
			}//j
	}//i
	


	//Initialize matrices based on the number of samples
	vector<CvMat*> matrices(2);
	matrices[0] = cvCreateMat( 2, totalSamples, CV_64FC1);
	matrices[1] = cvCreateMat( 2, totalSamples, CV_64FC1);


    //Create matrices using which affine transform is computed
	for( unsigned j=0 ; j<totalSamples ; j++ )
	{
		CV_MAT_ELEM(*matrices[0] ,double, 0,j) = matchPairs[j][0].x;
		CV_MAT_ELEM(*matrices[0] ,double, 1,j) = matchPairs[j][0].y;
		
		CV_MAT_ELEM(*matrices[1] ,double, 0,j) = matchPairs[j][1].x;
		CV_MAT_ELEM(*matrices[1] ,double, 1,j) = matchPairs[j][1].y;		
	}

    CvMat*  A = visionUtilities::estimateHomography(matrices);
   
    cvReleaseMat(&matrices[0]);
    cvReleaseMat(&matrices[1]);

    correspondences[0] = matchPairs; 

    return A;
	
	////********************************************************************************************
	
 //   svlPoint2d centreOfRegion = visionUtilities::findCentreOfRegion(currentTracks._polygon);
 //   IplImage *temp = cvCloneImage(cFrame._colorImage);
	//
	////for( int j=median - deviation  ; j<= median + deviation ; j++ )
	//for( unsigned j=0 ; j<matchPairs.size() ; j++ )
 //   //for( int j=0 ; j<totalSamples; j++ )
	//{	
	//	
 //      // printf(" %f ",actualDistances[j]);
 //       //printf(" %f ",featureSpaceRatios[j]);


	//	
 //   	 double r = rand() % 256;
	//	 double g = rand() % 256;
	//	 double b = rand() % 256;
	//	
	////	 cvLine(_combinedImage,cvPoint(matchPairs[j][0].x ,matchPairs[j][0].y),cvPoint(matchPairs[j][1].x  + colShift, matchPairs[j][1].y  + rowShift),cvScalar(b,g,r),1 );
	//	// cvLine(temp,cvPoint(matchPairs[j][0].x ,matchPairs[j][0].y),cvPoint(matchPairs[j][1].x , matchPairs[j][1].y),cvScalar(b,g,r),1 );
 //        cvLine(temp,cvPoint(matchPairs[j][0].x ,matchPairs[j][0].y),cvPoint(centreOfRegion.x ,centreOfRegion.y),cvScalar(b,g,r),1 );
 //        cvDrawCircle(temp,cvPoint((matchPairs[j][0].x+matchPairs[j][1].x)/2,(matchPairs[j][0].y+matchPairs[j][1].y)/2),sqrt(_actualDistances[j]),cvScalar(b,g,r));
	//  
	//}

 //   utilities::display(temp,"All correspondences");
	//
 //   cvReleaseImage(&temp);
 //   temp = cvCloneImage(cFrame._colorImage);
 //   
	////for( int j=median - deviation  ; j<= median + deviation ; j++ )
	////for( int j=0 ; j<matchPairs.size() ; j++ )
 //   for( unsigned j=0 ; j<totalSamples ; j++ )
	//{	
 //       //cout<<" "<<finalWeights[j];
 //       //printf(" %f ",actualDistances[j]);
 //       //printf(" %f ",featureSpaceRatios[j]);

	//	 double r = rand() % 256;
	//	 double g = rand() % 256;
	//     double b = rand() % 256;
	//	
	//	//cvLine(_combinedImage,cvPoint(matchPairs[j][0].x ,matchPairs[j][0].y),cvPoint(matchPairs[j][1].x  + colShift, matchPairs[j][1].y  + rowShift),cvScalar(b,g,r),1 );
	//	//cvLine(temp,cvPoint(matchPairs[j][0].x ,matchPairs[j][0].y),cvPoint(matchPairs[j][1].x , matchPairs[j][1].y),cvScalar(b,g,r),1 );
 //       cvLine(temp,cvPoint(matchPairs[j][0].x ,matchPairs[j][0].y),cvPoint(centreOfRegion.x , centreOfRegion.y),cvScalar(b,g,r),1 );
 //       cvDrawCircle(temp,cvPoint((matchPairs[j][0].x+matchPairs[j][1].x)/2,(matchPairs[j][0].y+matchPairs[j][1].y)/2),sqrt(_actualDistances[j]),cvScalar(b,g,r));
	//  
	//}

 //   cvDrawCircle(temp,cvPoint(centreOfRegion.x,centreOfRegion.y),5,cvScalar(0,0,255),5);
 //   //string name = _outFolder + "/" + utilities::toString(_currentFrame) + ".jpg";
 //   //cvSaveImage(name.c_str(),temp);
	//utilities::display(temp,"Selected correspondences");
	//
 //   cvReleaseImage(&temp);
 //      return A;
	//********************************************************************************************

 
}


//Compute metrics to enable sampling by various particle
int sift::computeMetricsForSampling( vector< vector< vector< svlPoint2d > > > &correspondences , contentTracks &tracks)
{

    //Incase SIFT computation is to be avoided, example in the case of a known particle
    if( correspondences.size() == 0 )
        return 0;
     
    //As of now time information is not being used in SIFT features
    vector< vector<svlPoint2d > > matchPairs = correspondences[0];

    //Not enough SIFT features are present (possibly something wrong with the frames like transitionary effects)
    if( matchPairs.size() < constants::MINIMUM_SIFT_FEATURES )
         return -1;

    //Init containers
    _imageDistances = vector<double>(matchPairs.size());
    _actualDistances = vector<double>(matchPairs.size());

    //Make local copies of stats from the current tracks
    svlPoint2d centreOfRegion =  visionUtilities::findCentreOfRegion(tracks._polygon);

    for( unsigned j=0 ; j<matchPairs.size() ; j++)
    {
        if( tracks._H != NULL )
        {
            //For future frames base distance on the previous velocity estimates
            double Hx = matchPairs[j][0].x;
            double Hy = matchPairs[j][0].y;

            //Estimate position of SIFT match based on previous velocity
            visionUtilities::projectCoordinate(tracks._H,Hx,Hy);
            
            double tempX = matchPairs[j][1].x - Hx ;
            double tempY = matchPairs[j][1].y - Hy ;

            _actualDistances[j] = tempX * tempX + tempY * tempY;
        }
        else
        {         
            //for first frame reply on old distane metric
            double tempX = matchPairs[j][1].x - matchPairs[j][0].x;
            double tempY = matchPairs[j][1].y - matchPairs[j][0].y;
            
            _actualDistances[j] = tempX*tempX + tempY*tempY;

        }
		
		
    }//match pairs j


    //Sort to estimate median only if old SIFT distance metric is being used for first frame
    if( tracks._H == NULL )
    {
         ////Sort the match pairs based on the actual distance being moved
        for( unsigned i=0 ; i< matchPairs.size() ; i++)
        {
	  	        for(unsigned j=i+1 ; j< matchPairs.size() ; j++)
		        {
			        if(_actualDistances[i] > _actualDistances[j])
			        {
				        double temp = _actualDistances[i];
				        _actualDistances[i] = _actualDistances[j];
				        _actualDistances[j] = temp;
                        
                        temp = _featureSpaceRatios[i];
				        _featureSpaceRatios[i] = _featureSpaceRatios[j];
				        _featureSpaceRatios[j] = temp;

				        matchPairs[i].swap( matchPairs[j] );
			        }
		        }//j
        }//i

    }

    //Populate the median
    _actualDistanceMedian = _actualDistances[ _actualDistances.size() / 2 ];
    
    for( unsigned j=0 ; j<matchPairs.size() ; j++)
    {     
        double tempX = matchPairs[j][0].x - centreOfRegion.x;
        double tempY = matchPairs[j][0].y - centreOfRegion.y;
        _imageDistances[j] = tempX*tempX + tempY*tempY;
		
    }

     correspondences[0] = matchPairs; 

    return 1;
}


//Computes correspondences of the central frame to all the others
vector< vector<vector<svlPoint2d> > > sift::computeCorrespondences( vector<frame> &frames)
{
    unsigned centreIndex = constants::VOLUME_MARGIN * 2;
    vector< vector<vector<svlPoint2d> > > correspondences;
    vector<vector<svlPoint2d> > matchPairs;
    _featureSpaceRatios.clear();

    //for( unsigned i=0 ; i<frames.size() ; i++)
    //{
    //    if( i != centreIndex)
    //    {
    matchPairs = findMatches(frames[centreIndex]._siftFeatures , frames[centreIndex+1]._siftFeatures , constants::SIFT_THRESHOLD , _featureSpaceRatios);
    correspondences.push_back(matchPairs);
    //    }
    //}

   
   return correspondences;
    
}



//Computes the correspondences between two frames
vector<vector<svlPoint2d> > sift::findMatches(Keypoint keys1, Keypoint keys2 , double threshold , vector<double> &ratios)
{
    Keypoint k, match;
	int count = 0;
	vector<vector<svlPoint2d> > result;

    if( constants::VERBOSE )
	    cout<<"\nMessage from internal SIFT code \n\n";

    //Find best match for each keypoint
    for (k=keys1; k != NULL; k = k->next)
	{
	  double ratio= 0.0;
	
      match = this->checkForMatch(k, keys2,threshold , &ratio);  
	  
      if (match != NULL) 
	  {
		count++;
				
		vector<svlPoint2d> temp(2);
		temp[0] = svlPoint2d(k->col , k->row);
		temp[1] = svlPoint2d(match->col , match->row);

		result.push_back(temp);
		ratios.push_back(ratio);
      }
    }
	
    if( constants::VERBOSE )
    {
        cout<<"Found "<<count<<" matches.\n";
	    cout<<"\nEnd of Message from internal SIFT code \n\n";
    }

	return result;
}

//Checks a euclidean distance based metric to find the best match for the given keypoint
Keypoint sift::checkForMatch(Keypoint key, Keypoint klist , double threshold , double *ratio)
{
    int dsq, distsq1 = 100000000, distsq2 = 100000000;
    Keypoint k, minkey = NULL;

    //Find the two best matches
    for (k = klist; k != NULL; k = k->next)
	{

      dsq = distSquared(key, k);

      if (dsq < distsq1) 
	  {
		distsq2 = distsq1;
		distsq1 = dsq;
		minkey = k;
      } 
	  else if (dsq < distsq2) 
	  {
		distsq2 = dsq;
      }
    }

	//*ratio = distsq1;
    *ratio = distsq1 / (distsq2 + 0.0);
    
    if (10 * 10 * distsq1 < (int)(threshold * threshold * distsq2))
      return minkey;
    else 
	  return NULL;
}



//Return squared distance in feature space between two keypoint descriptors
int sift::distSquared(Keypoint k1, Keypoint k2)
{
    int distsq = 0;
    unsigned char *pk1, *pk2;

    pk1 = k1->descrip;
    pk2 = k2->descrip;

    for (int i = 0; i < constants::SIFT_FV_SIZE ; i++) 
	{
	    int dif = (int) *pk1++ - (int) *pk2++;
        distsq += dif * dif;
    }
	
    return distsq;
}


//Computes the SIFT features for the given frame
void sift::computeSIFTFeatures(frame &cFrame)
{
    //Return if frame is a stub or has already been processed
    if( cFrame._index == -1 || cFrame._siftFeatures != NULL )
        return ;

    //Compute index for memory pool used by SIFT for this frame
    CUSTOM_KEY_POOL = (cFrame._index % (constants::VOLUME_MARGIN * 7) ) +  constants::SIFT_PREFIX_FOR_KEY_POOL ;

    //Convert IplImages to Lowe's data types
    Image imageObject = convertToImageObject(cFrame._grayScaleImage);
	
	//Extract SIFT features
    cFrame._siftFeatures = GetKeypoints(imageObject);

    //Free up memory
    delete [] *(imageObject->pixels);
    delete [] (imageObject->pixels);
}


//Convert the image from an openCV image object to the Image object required by Lowe's implementation
Image sift::convertToImageObject(IplImage *inputImage)
{
	//Read image data a char pointer
	unsigned char * imageData = reinterpret_cast<unsigned char *>(inputImage->imageData);
		
	//Step for image to go to the next row
	int widthStep = inputImage->widthStep ;

	//Number of channels in the image
	int numberOfChannels = inputImage->nChannels;

	//Create anImage object with the correct dimensions
	Image result = CreateImage(inputImage->height , inputImage->width);
		
	//Iterate over each pixel
	for( int i=0 , c=0; i<inputImage->height ; i++)
    {
				
		for( int j=0 ; j<inputImage->width * numberOfChannels ; j+= numberOfChannels)
		{
			//Convert to grayscale
			result->pixels[i][c++] = (float)((.299 * imageData[j+2] + .587 * imageData[j+1] + .114 * imageData[j]) / 255.0);
			
		}
		
		//Increment in char pointer to go the next row
		imageData += widthStep;
			
		//Set column counter to 0 for new row
		c = 0;
	}
	
	return result;
}

