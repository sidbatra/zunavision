/*****************************************************************************
** STAIR VISION LIBRARY
** Copyright (c) 2007, Stanford University
**
** FILENAME:    svlSIFT.cpp
** AUTHOR(S):   Siddharth Batra <sidbatra@stanford.edu>
** DESCRIPTION: Implements the functions in svlSIFT.h
**
*****************************************************************************/
#pragma once


#include "svlSIFT.h"


extern int KEY_POOL;
void FreeStoragePool(int);

//*********************** Static members *************
int svlSIFT::_norepeats = 0;


using namespace std;

//Code default constructor
svlSIFT::svlSIFT()
{
	//Does nothing as of now
}


vector<vector<svlPoint2d> > svlSIFT::findMatches(Keypoint keys1, Keypoint keys2 , double threshold , vector<double> &ratios)
{
    Keypoint k, match;

	int count = 0;

	vector<vector<svlPoint2d> > result;

	printf("\nMessage from internal SIFT code \n\n");
	
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
	
    printf("Found %d matches.\n", count);

	printf("\nEnd of Message from internal SIFT code \n\n");

	return result;
}




Keypoint svlSIFT::checkForMatch(Keypoint key, Keypoint klist , double threshold , double *ratio)
{
    int dsq, distsq1 = 100000000, distsq2 = 100000000;
    Keypoint k, minkey = NULL;

    /* Find the two closest matches, and put their squared distances in
       distsq1 and distsq2.
    */
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
int svlSIFT::distSquared(Keypoint k1, Keypoint k2)
{
    int i, dif, distsq = 0;
    unsigned char *pk1, *pk2;

    pk1 = k1->descrip;
    pk2 = k2->descrip;

    for (i = 0; i < 128; i++) 
	{
	dif = (int) *pk1++ - (int) *pk2++;
      distsq += dif * dif;
    }
	
    return distsq;
}

//Retrieves the total SIFT features on the image
int svlSIFT::getTotalSIFTFeatures(IplImage *image)
{
    int totalFeatures = 0;

    //Convert IplImages to Lowe's data types
	Image firstImage = convertToImageObject(image);
	
	//Extract SIFT features
	_imageOneFeatures = GetKeypoints(firstImage);

    while(_imageOneFeatures->next != NULL)
    {
        totalFeatures++;
        _imageOneFeatures = _imageOneFeatures->next;
    }
	
    //Free up memory
    delete [] *(firstImage->pixels);
    delete [] (firstImage->pixels);  

    FreeStoragePool(2);

    return totalFeatures;
}


//Matches the two given images using the SIFT features and returns 1 if the frame is repeated
int svlSIFT::matchImagesFast(IplImage* imageOne,IplImage* imageTwo , double threshold,CvMat *previousH,svlPoint2d centreOfRegion )
{ 
    int isRepeatedFrame = 0;
	
	//Convert IplImages to Lowe's data types
	Image firstImage = convertToImageObject(imageOne);
	Image secondImage = convertToImageObject(imageTwo);

	//Extract SIFT features
	_imageOneFeatures = GetKeypoints(firstImage);
	_imageTwoFeatures = GetKeypoints(secondImage);

    //Free up memory
    delete [] *(firstImage->pixels);
    delete [] *(secondImage->pixels);
    delete [] (firstImage->pixels);
    delete []  (secondImage->pixels);

	//Remove all previous matches
	_matchPairs.clear();
    _featureSpaceRatios.clear();
    _imageDistances.clear();
    _actualDistances.clear();
    _actualDistanceMedian = -1;
        

	    
	//Finds the matching pairs and distances between them
	_matchPairs = this->findMatches(_imageOneFeatures, _imageTwoFeatures , threshold , _featureSpaceRatios);

    if( (int)_matchPairs.size() < constants::MINIMUM_SIFT_FEATURES )
         return -1;
    
    vector<double> distance;

	////Compute distance moved by each SIFT match
	for( unsigned j=0 ; j<_matchPairs.size() ; j++)
	{
        //for first frame reply on old distane metric
        double tempX = _matchPairs[j][1].x - _matchPairs[j][0].x;
        double tempY = _matchPairs[j][1].y - _matchPairs[j][0].y;
        
        distance.push_back( tempX*tempX + tempY*tempY );
    }//match pairs j

    
    sort(distance.begin(),distance.end());

    
	//Compute average distance moved by SIFT pixels from frame i to i+1
    double medianDistance = distance[distance.size() / 2];

    
    //Removes repeated frames based on old SIFT distance metric
    if( medianDistance < constants::THRESHOLD_FOR_REPEAT_FRAME && !svlSIFT::_norepeats)
         isRepeatedFrame = 1;

    FreeStoragePool(2);
    


    //Collect sampling stats only if the frame is not repeated
    if( !isRepeatedFrame )
    {

        _imageDistances = vector<double>(_matchPairs.size());
        _actualDistances = vector<double>(_matchPairs.size());


        ///Compute distance moved by each SIFT match
	    for( unsigned j=0 ; j<_matchPairs.size() ; j++)
	    {
            if( previousH != NULL )
            {
                //For future frames base distance on the previous velocity estimates
                double Hx = _matchPairs[j][0].x;
                double Hy = _matchPairs[j][0].y;

                //Estimate position of SIFT match based on previous velocity
                visionUtilities::projectCoordinate(previousH,&Hx,&Hy);
                
                double tempX = _matchPairs[j][1].x - Hx ;
                double tempY = _matchPairs[j][1].y - Hy ;

                _actualDistances[j] = tempX * tempX + tempY * tempY;
            }
            else
            {         
                //for first frame reply on old distane metric
                double tempX = _matchPairs[j][1].x - _matchPairs[j][0].x;
                double tempY = _matchPairs[j][1].y - _matchPairs[j][0].y;
                
                _actualDistances[j] = tempX*tempX + tempY*tempY;

            }
    		
    		
	    }//match pairs j


        //Sort to estimate median only if old SIFT distance metric is beign used for first frame
        if( previousH == NULL )
        {
             ////Sort the match pairs based on the actual distance being moved
	        for( unsigned i=0 ; i<_matchPairs.size() ; i++)
	        {
		  	        for(unsigned j=i+1 ; j<_matchPairs.size() ; j++)
			        {
				        if(_actualDistances[i] > _actualDistances[j])
				        {
					        double temp = _actualDistances[i];
					        _actualDistances[i] = _actualDistances[j];
					        _actualDistances[j] = temp;
                            
                            temp = _featureSpaceRatios[i];
					        _featureSpaceRatios[i] = _featureSpaceRatios[j];
					        _featureSpaceRatios[j] = temp;

					        _matchPairs[i].swap( _matchPairs[j] );
				        }
			        }//j
	        }//i
        }

        
	    for( unsigned j=0 ; j<_matchPairs.size() ; j++)
	    {     
            double tempX = _matchPairs[j][0].x - centreOfRegion.x;
            double tempY = _matchPairs[j][0].y - centreOfRegion.y;
            _imageDistances[j] = tempX*tempX + tempY*tempY;
    		
	    }

        //Store the median for future use
        _actualDistanceMedian = _actualDistances[_matchPairs.size() /2 ];

    }//if not repeated

    return isRepeatedFrame;
        
}


//Matches the two given images using the SIFT features
vector<CvMat*> svlSIFT::sampleSIFTFeaturesForAffineEstimate(int *totalMatches  , CvMat *previousH , int typeIndex , int ratioIndex )
{
    
    vector<CvMat*> matrices(2);
    vector<double> finalWeights(_matchPairs.size());
    double weights[3];
    int totalSamples = -1;
        
    //Weights for different particles
    const double w_type[][3] = 
    { 
        {0.2998 , 0.7 , 0.0002} , 
        {0.7 , 0.3 , 0.0} , 
        {0.2 , 0.8 , 0.0} ,
        {0.39998 , 0.6 , 0.00002} ,
        {0.45 , 0.45 , 0.00002},
        {0.2 , 0.4 , 0.002}  
         //,{0.01 , 1.0 , 0.00002}
    };
    //{Neigbourhood affinity , Good quality matches , matches obeying previous velocity constraint , something in between , 
    //all equal weights}
    const double ts_type[] = {.2, .6 , 1};//{.25 , .75 , 1};// { 31 , 91 };
    


    //Selection of parameters based on requested type value
    for( int i=0 ; i<3 ; i++)
        weights[i] = w_type[typeIndex][i];

    totalSamples = (int)(_matchPairs.size() * ts_type[ratioIndex]);
    
    //Reduce the total number of samples if the matches found in the image weren't enough
     if( totalSamples < constants::MINIMUM_SIFT_FEATURES )
        totalSamples = constants::MINIMUM_SIFT_FEATURES;


    //Compute weights for each correspondence pairing
	for( unsigned j=0 ; j<_matchPairs.size() ; j++)
	{
        double final = weights[0] * _featureSpaceRatios[j] + weights[2] * _imageDistances[j];

        //Use distance from median in old SIFT metric not in the new one
        if( previousH != NULL )
            final += weights[1] * _actualDistances[j];
        else
            final += weights[1] * fabs(_actualDistances[j] - _actualDistanceMedian);
                      
        finalWeights[j] = final;
                
    }


    //Sort the match pairs based on the actual distance being moved
	for( unsigned i=0 ; i<_matchPairs.size() ; i++)
	{
		  	for(unsigned j=i+1 ; j<_matchPairs.size() ; j++)
			{
				if(finalWeights[i] > finalWeights[j])
				{
					double temp = finalWeights[i];
					finalWeights[i] = finalWeights[j];
					finalWeights[j] = temp;

                    _matchPairs[i].swap( _matchPairs[j] );

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
	


	//Initialize matrices based on expected number of samples
	
	matrices[0] = cvCreateMat( 2, totalSamples, CV_64FC1);
	matrices[1] = cvCreateMat( 2, totalSamples, CV_64FC1);


    //Create matrices using which affine transform is computed
	for( int j=0, col=0  ; j<totalSamples; j++ , col++)
	{
		CV_MAT_ELEM(*matrices[0] ,double, 0,col) = _matchPairs[j][0].x;
		CV_MAT_ELEM(*matrices[0] ,double, 1,col) = _matchPairs[j][0].y;
		//CV_MAT_ELEM(*matrices[0] ,double, 2,col) = 1.0;

		CV_MAT_ELEM(*matrices[1] ,double, 0,col) = _matchPairs[j][1].x;
		CV_MAT_ELEM(*matrices[1] ,double, 1,col) = _matchPairs[j][1].y;
		//CV_MAT_ELEM(*matrices[1] ,double, 2,col) = 1.0;
	}

    *totalMatches = totalSamples;
	
    return matrices;
	
   // cvCvtColor(imageOne,imageOne,CV_Lab2RGB);
   // cvCvtColor(imageTwo,imageTwo,CV_Lab2RGB);
	////********************************************************************************************
	//Generate a vertical combination of the two input images
//	combineImagesVertically(imageOne,imageTwo);
//
//    
//   // IplImage *temp = cvCloneImage(imageTwo);
//     //IplImage *temp = cvCloneImage(_combinedImage);
//	
//	//row col shifts needed by second image upon verticalcombination
//	int rowShift = 0;
//    int colShift = imageOne->width;
//
//    //printf("\n\n\n");
//
//	//for( int j=median - deviation  ; j<= median + deviation ; j++ )
//	//for( int j=0 ; j<_matchPairs.size() ; j++ )
//    for( int j=0 ; j<totalSamples; j++ )
//	{	
//		
//       // printf(" %f ",actualDistances[j]);
//        //printf(" %f ",featureSpaceRatios[j]);
//
//
//		
//    	 double r = rand() % 256;
//		 double g = rand() % 256;
//		 double b = rand() % 256;
//		
//		 cvLine(_combinedImage,cvPoint(_matchPairs[j][0].x ,_matchPairs[j][0].y),cvPoint(_matchPairs[j][1].x  + colShift, _matchPairs[j][1].y  + rowShift),cvScalar(b,g,r),1 );
//		// cvLine(temp,cvPoint(_matchPairs[j][0].x ,_matchPairs[j][0].y),cvPoint(_matchPairs[j][1].x , _matchPairs[j][1].y),cvScalar(b,g,r),1 );
//   //      cvLine(temp,cvPoint(_matchPairs[j][0].x ,_matchPairs[j][0].y),cvPoint(centreOfRegion.x ,centreOfRegion.y),cvScalar(b,g,r),1 );
//	  
//	}
//
//    
//	//
//	cvNamedWindow("Debug Windows - Track points", 1);
////	cvShowImage("Debug Windows - Track points" , temp);
//  cvShowImage("Debug Windows - Track points" , _combinedImage);
//	cvWaitKey(-1);
//  cvDestroyAllWindows();
// //   
//     //cvReleaseImage(&temp);
//     //IplImage *temp = cvCloneImage(imageTwo);
//
// //   combineImagesVertically(imageOne,imageTwo);
//
//	//
//	//row col shifts needed by second image upon verticalcombination
//	//int rowShift = 0;
//	//int colShift = firstImage->cols;
//     //printf("\n\n\n");
//
//	//for( int j=median - deviation  ; j<= median + deviation ; j++ )
//	//for( int j=0 ; j<_matchPairs.size() ; j++ )
//    //for( int j=0 ; j<totalSamples ; j++ )
//	//{	
//     //   printf(" %f ",actualDistances[j]);
//        //printf(" %f ",featureSpaceRatios[j]);
//
//		// double r = rand() % 256;
//		// double g = rand() % 256;
//	//	 double b = rand() % 256;
//		
//		// cvLine(_combinedImage,cvPoint(_matchPairs[j][0].x ,_matchPairs[j][0].y),cvPoint(_matchPairs[j][1].x  + colShift, _matchPairs[j][1].y  + rowShift),cvScalar(b,g,r),1 );
//		//cvLine(temp,cvPoint(_matchPairs[j][0].x ,_matchPairs[j][0].y),cvPoint(_matchPairs[j][1].x , _matchPairs[j][1].y),cvScalar(b,g,r),1 );
//     //    cvLine(temp,cvPoint(_matchPairs[j][0].x ,_matchPairs[j][0].y),cvPoint(centreOfRegion.x , centreOfRegion.y),cvScalar(b,g,r),1 );
//    //     cvDrawCircle(temp,cvPoint((_matchPairs[j][0].x+_matchPairs[j][1].x)/2,(_matchPairs[j][0].y+_matchPairs[j][1].y)/2),sqrt(actualDistances[j]),cvScalar(b,g,r));
//	  
//	//}
//
//  //  string name = _outFolder + "/" + utilities::toString(_currentFrame) + ".jpg";
//  // cvSaveImage(name.c_str(),temp);
//	
//	//cvNamedWindow("Debug Windows - Track points", 1);
//	//cvShowImage("Debug Windows - Track points" , temp);
// //   //cvShowImage("Debug Windows - Track points" , _combinedImage);
//	//cvWaitKey(-1);  
//	//cvDestroyAllWindows();
//
//    //cvReleaseImage(&temp);
//	////********************************************************************************************
//
//	//  FreeStoragePool(2);
//
//
//   return matrices;
}


//Combines the two given images horizontally
void svlSIFT::combineImagesHorizontally(IplImage* imageOne, IplImage* imageTwo)
{
	//Compute total rows
	int rows = imageOne->height + imageTwo->height;

	//Compute cols by taking the larger of the two
	int columns = max( imageOne->width , imageTwo->width);

	//Init IplImage data member
	_combinedImage = cvCreateImage(cvSize(columns,rows),IPL_DEPTH_8U,3);

	//Feed the contents of the first image
	for( int i=0 ; i<imageOne->height ; i++)
	{
		for( int j=0 ; j<imageOne->width ; j++ )
		{
			cvSet2D(_combinedImage,i,j,cvGet2D(imageOne,i,j));
		}
	}

	//Feed the contents of the second image
	for( int i=0 ; i<imageTwo->height ; i++)
	{
		for( int j=0 ; j<imageTwo->width ; j++ )
		{
			cvSet2D(_combinedImage,i + imageOne->height,j,cvGet2D(imageOne,i,j));
			
		}
	}

	

	
}

//Combines the two given images vertically and puts the result in the _combinedImage datamember
void svlSIFT::combineImagesVertically(IplImage* imageOne, IplImage* imageTwo)
{


	//Compute total columns
	int columns = imageOne->width + imageTwo->width;

	//Compute rows by taking the larger of the two
	int rows = max( imageOne->height , imageTwo->height);

	//Init IplImage data member
	//_combinedImage = cvCreateImage(cvSize(columns,rows),IPL_DEPTH_8U,3);
	_combinedImage = cvCreateImage(cvSize(columns,rows),IPL_DEPTH_8U,1);

	//Feed the contents of the first image
	for( int i=0 ; i<imageOne->height ; i++)
	{
		for( int j=0 ; j<imageOne->width ; j++ )
		{
			cvSet2D(_combinedImage,i,j,cvGet2D(imageOne,i,j));
			 
		}
	}

	//Feed the contents of the second image
	for( int i=0 ; i<imageTwo->height ; i++)
	{
		for( int j=0 ; j<imageTwo->width ; j++ )
		{
			cvSet2D(_combinedImage,i,j + imageOne->width ,cvGet2D(imageTwo,i,j));
		}
	}


}


//Convert the image from an openCV image object to the Image object required by Lowe's implementation
Image svlSIFT::convertToImageObject(IplImage *inputImage)
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


//Converts David Lowe's Image object to an OpenCV IplImage*
IplImage* svlSIFT::convertToIplImage(Image imageObject , IplImage *inputImage)
{

	//Read image data a char pointer
	unsigned char * imageData = reinterpret_cast<unsigned char *>(inputImage->imageData);

	//Step for image to go to the next row
	int widthStep = inputImage->widthStep ;

	//Number of channels in the image
	int numberOfChannels = inputImage->nChannels;

	//Iterate over each pixel
	for( int i=0 , c=0; i<inputImage->height ; i++ )
	{

		for( int j=0 ; j<inputImage->width * numberOfChannels ; j+= numberOfChannels )
		{
			//Convert the pixel info to grayscale and store in the pixel matrix
		    imageData[j+2] = (unsigned char)(imageObject->pixels[i][c] * 255.0);
		    imageData[j+1] = (unsigned char)(imageObject->pixels[i][c] * 255.0);
		    imageData[j] = (unsigned char)(imageObject->pixels[i][c] * 255.0);

			//Move to next column
			c++;
		}

		//Increment in char pointer to go the next row
		imageData += widthStep;

		//Set column counter to zero for next row
		c=0;
	}

	return inputImage;

}

