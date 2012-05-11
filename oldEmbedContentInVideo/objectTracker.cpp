/*****************************************************************************
** MOVIE PROJECT
** Copyright (c) 2008
**
** FILENAME:    objectTracker.cpp
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Function definitions for the objectTracker class

*****************************************************************************/

#include "objectTracker.h"


//*********************** Constants **********************************

//**************** Misc debug *****************
const int PREFIX = 18;
const int OCCLUDED = 255;
const int FREE = 0;
const int ADD = 59;
//const int SAVE_OBJ_STATS = 0;
//const int SAVE_LOCATION_STATS = 0;


//************* Occlusion outlier detection *************


const CvScalar INITIAL_VARIANCE =  cvScalar(10,3,3); //Initial variances for each pixel, over each channel

//***************** Annealing ******************
const int TOTAL_SMOOTHING_ITERATIONS = 4; //Total level of smoothing
const unsigned SMOOTHING_ITERATIONS[TOTAL_SMOOTHING_ITERATIONS] = {4,3,2,1};  //Values for each level of smoothing
const double SMOOTHING_WEIGHTS[TOTAL_SMOOTHING_ITERATIONS] = {0.05,0.05,0.2,0.7};//{0.25,0.25,0.25,0.25}; //Signifies how important each level of blurring is
const int OCC_SMOOTHING_INDEX = 3; //Index in SMOOTHING_ITERATIONS for 



//****************** NCC *******************
const int FALLOFF_VALUE = -5;



//************************* Static Variables *************************


int objectTracker::_selectedParticle = -1; //Holds which particle has been selected

//#DEBUG#
//string objectTracker::_tempOut;
//#DEBUG#

//***************************** Constructor Logic ****************************

objectTracker::objectTracker()
{	
    _currentFrame = 1;
	_occlusionMode = 0;
	_previousOcclusionMode  = 0;
    _cleanImage = NULL;
    _cleanTransform = NULL;
    _previousH = NULL;
    _preCutSceneFrame = NULL;
    _preCutSceneFrameColor = NULL;
    _IDashOccColor = NULL;
    
}




//****************************** Methods **********************************


//Tracks the given polygon from one frame to the next based on estimation of the camera homography and
//an optimization based projective transform estimation from 2D matches
//Returns 1 if the next frame is very different to the current one to move to next content
int objectTracker::trackObjectsWithHomography(IplImage* imageOne, IplImage* imageTwo, vector<svlPoint2d> &objects
                                               ,vector<svlPoint2d> &affineObjects
                                               , IplImage *previousFrame , IplImage *currentFrame)
{	
    int totalMatches = 0; //Holds total matches or point correspondences between the two frames
    double siftThreshold = 6; 	//Threshold for matching SIFT features from one frame to the other
    double minX = 0.0 , minY = 0.0 , maxX = 0.0 , maxY = 0.0;
    	
	//Compute enclosing rectangle with a margin
	findMinMax(objects,0,minX,minY,maxX,maxY);

    
    int isRepeated = _siftObject.matchImagesFast(imageOne,imageTwo,siftThreshold,_previousH,
        svlPoint2d((maxX+minX)/2,(maxY+minY)/2)) ;
  
    
    if(0)// visionUtilities::estimateCutScene(imageOne,imageTwo) )
    {     

        cvCvtColor(previousFrame,previousFrame,CV_Lab2RGB);

        savePreCutScene(imageOne,previousFrame);

     
        cout<<"\nCUT SCENE FOUND";

        return 1;
    }
    else if( isRepeated == 1 || isRepeated == -1 )//If repeated frame is found skip optimization step 
    {
        cout<<"\n REPEATED FRAME FOUND || MINIMUM SIFT FEATURES NOT FOUND";

        //Indicates repeated particle
        objectTracker::_selectedParticle = -2;
        return 0;
    }

    
      //Setup optimization related stats common to all particles
      _currentFrame++;
      _previousOcclusionMode = _occlusionMode;

      //#DEBUG# #Saves the values of the objective function resulting from different particles# 
      //string name = svlSIFT::_outFolder + "//particles.txt";
      //ofstream outp(name.c_str(),ios::app);

	
      //Compute the transformation

       CvMat *H_Star = NULL;
       CvMat *A_Star = NULL;
		

            //******************* Compute affine homography between the two frames ******************

            //Variables to hold results from the best optimization partcile thus far
            
            double J_Star = 9999.0;
            int isFirstParticle = 1;
            int selectedParticle = -1;
            
            //const double VELOCITY_WEIGHT = 0.3;
            
            int testTotal = 0;
            vector<CvMat *> testMatches = _siftObject.sampleSIFTFeaturesForAffineEstimate(&testTotal , _previousH 
                , constants::TEST_SIFT_INDEX,constants::TEST_SAMPLE_INDEX );
            A_Star = formAffineMatrix(testMatches,testTotal);


            //save pre cut scenes if you choose to return to .. return 2 model.. try & avoid it
            //Checks if the pixel is possibly out of bounds
            if( !visionUtilities::checkOutofBounds(imageOne,objects,A_Star) )
            {   
                int siftType = 0;
                int sampleType = 0;

                //Iterate over each particle .. allowing use of initial esitmate which results in the best results
                for( int p=0 ; p< constants::TOTAL_PARTICLES ; p++)
                {

                   cout<<"\n\n Particle "<<p;
                   _currentParticle = p;

                   if( p >= constants::SIFT_PARTICLES )
                       sampleType = 1;
                   
                   siftType = p % constants::SIFT_PARTICLES;
                   

                   CvMat *A = NULL;

                   if( p < constants::TOTAL_PARTICLES - 3 )
                   {continue;
                       printf("\n SIFT %d SAMPLE %d",siftType,sampleType);

                        //Returns point correspondences as a vector of matrices
                        vector<CvMat *> matches = _siftObject.sampleSIFTFeaturesForAffineEstimate(&totalMatches , _previousH 
                            ,siftType , sampleType );
                        A = formAffineMatrix(matches,totalMatches);
                   }
                   else if(p == constants::TOTAL_PARTICLES - 3 )  //ALL SIFT SAMPLES
                   {continue;
                       sampleType = 2;
                       printf("\n ALL FEATURES SIFT %d SAMPLE %d",siftType,sampleType);

                        //Returns point correspondences as a vector of matrices
                        vector<CvMat *> matches = _siftObject.sampleSIFTFeaturesForAffineEstimate(&totalMatches , _previousH 
                            ,siftType , sampleType);
                        A = formAffineMatrix(matches,totalMatches);

                   }
                   else if(p == constants::TOTAL_PARTICLES - 2 )  //VELOCITY
                   {continue;
                        cout<<"\n VELOCITY";

                        if( _previousH == NULL)
                            continue;

                        A = cvCloneMat(_previousH);
                   }
                   else 
                    {                        
                        cout<<"\n IDENTITY";

                        A = cvCreateMat(3,3,CV_64FC1);

                        for( int y=0 ; y<3 ; y++)
                            for( int x=0 ; x<3 ; x++)
                                CV_MAT_ELEM(*A,double,y,x) = constants::IDENTITY[y*3+x];
                        
                    }
               

                    ////Mixes the affine estimate and the previous velocity
                    //if( p % 2 == 1 && p!= TOTAL_PARTICLES -1 )
                    //{
                    //    //Skip this step if the first frame is being processed
                    //    if( _previousH == NULL )
                    //            continue;

                    //    //Donot weigh the last row, since
                    //       for( int y=0 ; y<3 ; y++)
	                   //    	for( int x=0 ; x<3 ; x++)
                    //           {
                    //               if( y != 2 )
                    //                    cvmSet(A,y,x,cvmGet(A,y,x) * (1-VELOCITY_WEIGHT) + cvmGet(_previousH,y,x) * VELOCITY_WEIGHT );
                    //               else
                    //                   cvmSet(A,y,x,cvmGet(_previousH,y,x));
                    //           }
                    //    
                    //}//if mix


                    double J_reference = 0.0;  

			        //Estimate the projective transform for each individual polygon
			        //on the assumption that each polygon lies on the same plane
			        //Init the projection matrix to NULL
                    //This is done under constrained iterations for speed
		              CvMat* H = estimateProjectiveTransform(A , imageOne , imageTwo , objects , previousFrame , currentFrame 
                        , isFirstParticle,&J_reference,1);
                      cvReleaseMat(&H);
                
                        
                      //If optimization is better than the previous ideal case
                      //replace H_Star
                      if( J_reference < J_Star )
                      {
                          cout<<"\n ********** Particle selected *************\n";
                            
                          //###DEBUG MUST REMOVE THIS FIRST PARTICLE***********************
                          if( isFirstParticle )
                          {
                          cvReleaseMat(&A_Star);
                      
                          A_Star = cvCloneMat(A);
                          J_Star = J_reference;

                          selectedParticle = p;
                          }
                      }
                    
                      
                      //if( _currentFrame != 2 )
                      //{
                      //     outp<<J_reference;

                      //    if( p!=TOTAL_PARTICLES-1)
                      //        outp<<" ";
                      //    else
                      //        outp<<"\n";
                      //}

                      //Ensures pixel model is not initialized more than once
                     isFirstParticle = 0;
                       
                     cvReleaseMat(&A);

                } //particle p

               // outp.close();

                cout<<"\n\n Finalizing stats for best particle "<<selectedParticle;
                objectTracker::_selectedParticle = selectedParticle;

                //###DEBUG MUST REMOVE THIS CLONIGNG***********************
                //Run the final optimization for the selected particle without any iteration constraints
                H_Star = cvCloneMat(A_Star);//estimateProjectiveTransform(A_Star , imageOne , imageTwo , objects , previousFrame , currentFrame 
                        //, isFirstParticle,&J_Star,0);

                //Finalize the optimization
                finalizeOptimizationStats(H_Star,imageTwo,previousFrame,currentFrame);

            }//If region is not out of bounds
            else
            {
                H_Star = cvCloneMat(A_Star);
            }
		
				
            ofstream out("test.txt",ios::app);
			//Find new location for each point within the object
			for( unsigned j=0 ; j<objects.size() ; j++)
	 		{	
				//Compute the corresponding coordinate for the given
				//polygon coordinate in the next frame and project it 
				//using the full projective estimation matrix H

				printf("\n\n%e %e ",objects[j].x,objects[j].y);
				
				projectCoordinate(H_Star,&(objects[j].x),&(objects[j].y));	                

				printf("After H  %e %e",objects[j].x,objects[j].y);

                out<<objects[j].x<<" "<<objects[j].y<<" ";


				//Compute the corresponding coordinate for the given
				//polygon coordinate in the next frame and project it 
				//using the only the affine matrix A
				//printf("\n%e %e ",affineObjects[i][j].x,affineObjects[i][j].y);
				
				projectCoordinate(A_Star,&(affineObjects[j].x),&(affineObjects[j].y));

				//printf("After A %e %e ",affineObjects[i][j].x,affineObjects[i][j].y);
				
            }//coordinates of polygon
            out<<"\n";
            out.close();
            
        
        if( _previousH != NULL )
            cvReleaseMat(&_previousH);

        //Save velocity for estimates in the next frame
        _previousH = cvCloneMat(H_Star);
        

        
        //Free memory
         cvReleaseMat(&H_Star);
         cvReleaseMat(&A_Star);
		
     return 0;
	
}


//Computes a pseduo inverse of the correspondences to estimate the affine transformation matrix
CvMat* objectTracker::formAffineMatrix(vector<CvMat*> matches, int totalMatches)
{

    CvMat *A = cvCreateMat(3,3,CV_64FC1);
    cvFindHomography(matches[0] ,matches[1],A);
    return A;
    
	//CvMat *ADash = cvCreateMat( totalMatches , 3, CV_64FC1);
	//CvMat *AAT = cvCreateMat(3,3,CV_64FC1);
	//CvMat *inv = cvCreateMat(3,3,CV_64FC1);
	//CvMat *step3 = cvCreateMat(totalMatches,3,CV_64FC1);
	//CvMat *A = cvCreateMat(3,3,CV_64FC1);
	//
	////H * F1 = F2
	////Pseudo inverse
	////H = F2 * F1' * inv(F1 * F1')

	//cvmTranspose(matches[0],ADash);
	//cvmMul(matches[0],ADash,AAT);
	//cvmInvert(AAT,inv);
	//cvmMul(ADash,inv,step3);
	//cvmMul(matches[1],step3,A);

	//cvReleaseMat(&(matches[0]));
	//cvReleaseMat(&(matches[1]));
	//cvReleaseMat(&ADash);
	//cvReleaseMat(&AAT);
	//cvReleaseMat(&inv);
	//cvReleaseMat(&step3);

 //   return A;
}

//Saves the state prior to a cut scene
void objectTracker::savePreCutScene(IplImage *previousFrame , IplImage *previousFrameColor)
{
    //Free memory
    if( _preCutSceneFrame != NULL )
        cvReleaseImage(&_preCutSceneFrame);

    if( _preCutSceneFrameColor != NULL )
        cvReleaseImage(&_preCutSceneFrameColor);
    
    //Make copies
    _preCutSceneFrame = cvCloneImage(previousFrame);
    _preCutSceneFrameColor = cvCloneImage(previousFrameColor);
    
}

//Used to check if the current region is back into view
int objectTracker::estimateCutScene(IplImage *two)
{   
    return visionUtilities::estimateCutScene( _preCutSceneFrame , two);
}


//Estimates the projetive transform for the given polygon based on the affine transform of the camera
//and the assumption that all the points lie on the same plane
CvMat* objectTracker::estimateProjectiveTransform(CvMat *A, IplImage *imageOne, IplImage *imageTwo
                                                  , std::vector<svlPoint2d> polygon 
                                                  , IplImage *previousFrame , IplImage *currentFrame 
                                                  , int isFirstParticle , double *J_reference , int constrainIterations)
{
    //PIXEL MODEL IS NOT BEIGN UPDATED UNDER OCCLUSION>> KEEP THAT IN MIND
	
	//Set initial value of the projection matrix as the affine matrix
	CvMat *H = cvCloneMat(A);
	
	//Make copies for blurring etc naming consistent with math notation
	IplImage *IDash = cvCloneImage(imageTwo);
   
    if( _IDashOccColor != NULL )
        cvReleaseImage(&_IDashOccColor);

     _IDashOccColor = cvCloneImage(currentFrame);
    
        
    //Compute index of the initial smoothing level
	int index = 0;
 
    //Smoothen the current image (IDash)
    for( unsigned i=0 ; i<SMOOTHING_ITERATIONS[index] ; i++)
        cvSmooth(IDash,IDash,CV_GAUSSIAN,constants::SMOOTHING_MASK_SIZE,constants::SMOOTHING_MASK_SIZE);        
    

    //Smoothen a colored copy of the current frame for occlusion detection
    for( unsigned i=0 ; i<SMOOTHING_ITERATIONS[OCC_SMOOTHING_INDEX] ; i++)
        cvSmooth(_IDashOccColor,_IDashOccColor,CV_GAUSSIAN,constants::SMOOTHING_MASK_SIZE,constants::SMOOTHING_MASK_SIZE);
    	

	//Setup the model on the first frame
    //runs so that the model is not rebuilt
	if( _currentFrame == 2 && isFirstParticle == 1)
	{
		//Computes the Edge pixels to be tracked
		computeEdgePixels(cvCloneImage(imageOne), cvCloneImage(previousFrame) , polygon ,0);
	}
    

   //**************** Setting up occlusion mode aids ************
    
    //Clean up vectors & setup defaults
	_isOccluded.clear();

    for( unsigned i=0 ; i< _regionPixels.size() ; i++)
		_isOccluded.push_back(0);
	
    //***********************************************************

    //Removes pixels which may be occluded from the tracking process for the current frame
    if( estimateOccludedPixels(A,_IDashOccColor)  )
   {
       cout<<"\n Occlusion is greater than tractable\n";

        populateModelNCCStats(index);
        *J_reference = J(IDash,H,index); 

       	//Free memory
	    cvReleaseImage(&IDash);
        cvReleaseImage(&_IDashOccColor);

        return H;
   }
    
    
    //Buffer model statis used for NCC
    populateModelNCCStats(index);
	
    
	//Attempt to minimize the objective function for the max number of iterations
	for( int i=0 ; i<constants::MAX_ITERATIONS ; i++)
	{ 
		//Objective function value from the previous iteration
		double J_i_minus_1 = J(IDash,H,index); 
      

		//printf("\n ***************** \n %d JInitial = %e %d \n ******************",i,J_i_minus_1,_currentFrame);
	
		//Holds all the slopes as a vector
		double delJ_vector[9];

		
		for( int y=0 ; y<3 ; y++)
		{
			for( int x=0 ; x<3 ; x++)
			{				
				int entry = y * 3 + x;

				//Compute and store the slope
				delJ_vector[entry] = delJ(IDash,H,y,x,index);				
								
			}//x
		}//y



		//************Compute Hessian of the objective function************
			
		//Init matrix to hold the hessian
		CvMat *Hessian = cvCreateMat(9,9,CV_64FC1);
		
		//Compute value at each index (j,k)
		for( int j=0 ; j<9 ; j++)
		{
			for( int k=j ; k<9 ; k++)
			{
				//Convert hessian indicies to indices in the projective matrices
				int jy = j / 3 , jx = j % 3;
				int ky = k / 3 , kx = k % 3;
				
				CvMat *HDash  = cvCloneMat(H);
				CvMat *HDashj = cvCloneMat(H);
				CvMat *HDashk = cvCloneMat(H);
				
				CV_MAT_ELEM(*HDash,double,jy,jx) = CV_MAT_ELEM(*HDash,double,jy,jx) + constants::DELTA_HJ;
				CV_MAT_ELEM(*HDash,double,ky,kx) = CV_MAT_ELEM(*HDash,double,ky,kx) + constants::DELTA_HJ;

				CV_MAT_ELEM(*HDashj,double,jy,jx) = CV_MAT_ELEM(*HDashj,double,jy,jx) + constants::DELTA_HJ;

				CV_MAT_ELEM(*HDashk,double,ky,kx) = CV_MAT_ELEM(*HDashk,double,ky,kx) + constants::DELTA_HJ;

				//Compute Hjk via numerical differentation
				
				double Jjk  = J(IDash,HDash,index);
				double Jj   = J(IDash,HDashj,index);
				double Jk   = J(IDash,HDashk,index);
				
				double Hjk = Jjk - Jj - Jk + J_i_minus_1;

				Hjk /= (constants::DELTA_HJ * constants::DELTA_HJ);


                CV_MAT_ELEM(*Hessian,double,j,k) = Hjk;
                CV_MAT_ELEM(*Hessian,double,k,j) = Hjk;

				cvReleaseMat(&HDash);
				cvReleaseMat(&HDashj);
				cvReleaseMat(&HDashk);

			}
		}

        

		CvMat *HessianInv = cvCreateMat(9,9,CV_64FC1);

		//Invert the hessian
		cvmInvert(Hessian,HessianInv);

        cvReleaseMat(&Hessian);
		


		//**************************** Compute steps before line search *************

		//Holds all the slopes as a vector
		double delJ_final[9];

		for( int j=0 ; j<9 ; j++)
		{
			double temp = 0.0;

			for( int k=0 ; k<9 ; k++)
			{
				temp += CV_MAT_ELEM(*HessianInv,double,j,k) * delJ_vector[k];
			}
		
			delJ_final[j] = temp;
		}

		
		//******************* Perform back tracking line search***************

		
		double alpha =  constants::INITIAL_ALPHA;	 //Learning rate
		double delJ_New[9];				//Vector for the slopes
		double J_New = 0.0;				//New objective function value at each iteration
		double backtrackFactor = 2;		//Back tracking factor (Beta) back tracking line search
	
	
		//Back track till there is an improvement in the objective function
		do
		{	
			//compute new learning rate
			alpha /= backtrackFactor;

			//Compute HDash to hold new value of Slope
			CvMat *HDash = cvCloneMat(H);

			for( int y=0 ; y<3 ; y++)
			{
				for( int x=0 ; x<3 ; x++)
				{
					int entry = y * 3 + x;
					
					delJ_New[entry] = delJ_final[entry] * alpha;
					
					//Update HDash
					CV_MAT_ELEM(*HDash,double,y,x) = CV_MAT_ELEM(*H,double,y,x) - delJ_New[entry];
				}
			}

         							
			//Compute value of objective function with HDash
			J_New = J(IDash,HDash,index);

			//Free memory
			cvReleaseMat(&HDash);
					
			//printf("\n Jnew = %e",J_New);
			
		}
		while(J_New > J_i_minus_1);

                	
		//Compute new slopes
		for( int y=0 ; y<3 ; y++)
		{
			for( int x=0 ; x<3 ; x++)
			{
				int entry = y * 3 + x;
		
				//Update H
				CV_MAT_ELEM(*H,double,y,x) = CV_MAT_ELEM(*H,double,y,x) - delJ_New[entry];
			}
		}

		//Final J at the end of the iteration
		double J_i = J_New;

		//If due to some bug improvement is not made to the overall objective function
		//stop program for debugging
		if( J_i_minus_1 - J_i < 0 )
		{
			printf("??????");
			char gg;
            cin>>gg;
			
		}


		//if vel is less than tolerance
        if( fabs( J_i_minus_1 - J_i ) <= constants::TOLERANCE || (constrainIterations && i % constants::ITERATIONS_FOR_PF == 1)  )
		{	

			//printf("\n ******************************************** STRIKE **************************\n");
			
            //Update J_reference with the resultant value of this smoothing level
            *J_reference = *J_reference + J_i * SMOOTHING_WEIGHTS[index];
            
            index++;

			if( index < TOTAL_SMOOTHING_ITERATIONS )
			{	
				cvReleaseImage(&IDash);

				//Make copies for blurring etc naming consistent with math notation
				IDash = cvCloneImage(imageTwo);
				
				//Anneal the objective function
                for( unsigned j=0 ; j<SMOOTHING_ITERATIONS[index] ; j++)
                    cvSmooth(IDash,IDash,CV_GAUSSIAN,constants::SMOOTHING_MASK_SIZE,constants::SMOOTHING_MASK_SIZE);

       
                populateModelNCCStats(index);
       
			}
			else
			{                
                break;
			}
			
		}//if tolerance

	}//i


	//Free memory
	cvReleaseImage(&IDash);
    cvReleaseImage(&_IDashOccColor);

	//Return the estimation for the projective matrix
	return H;
}


//Finalizes occlusion stats and updates the model of the region being kept
void objectTracker::finalizeOptimizationStats(CvMat *H , IplImage *imageTwo , IplImage *previousFrame , IplImage *currentFrame)
{
     IplImage *IDash = cvCloneImage(imageTwo);   
     IplImage *IDashOccColor = cvCloneImage(currentFrame);


    //Smoothen a colored copy of the current frame for occlusion detection
     for( unsigned i=0 ; i<SMOOTHING_ITERATIONS[OCC_SMOOTHING_INDEX] ; i++)
         cvSmooth(IDashOccColor,IDashOccColor,CV_GAUSSIAN,constants::SMOOTHING_MASK_SIZE,constants::SMOOTHING_MASK_SIZE);
   
    //Improve the estimate of the occluded pixels by checking with the final projective transform
    estimateOccludedPixels(H,IDashOccColor);


	//if occlusion is detected.. store the clean image
    if( _occlusionMode ==1 && _previousOcclusionMode == 0)
    {
	    printf("\n Starting with clean resources");

	    if( _cleanImage != NULL )
		    cvReleaseImage(&_cleanImage);

	    _cleanImage = cvCloneImage(previousFrame);	

        for( unsigned i=0 ; i<SMOOTHING_ITERATIONS[OCC_SMOOTHING_INDEX] ; i++)
            cvSmooth(_cleanImage,_cleanImage,CV_GAUSSIAN,constants::SMOOTHING_MASK_SIZE,constants::SMOOTHING_MASK_SIZE); 	

     }


    if( _occlusionMode == 0 && _previousOcclusionMode == 1 )
    {        
	    if( _cleanImage != NULL )
        {
		    cvReleaseImage(&_cleanImage);
            _cleanImage = NULL;
        }

        if( _cleanTransform != NULL )
        {
		    cvReleaseMat(&_cleanTransform);
            _cleanTransform = NULL;
        }
    }


    //******************** Update pixel model **************************

    //Update pixel locations in the trajectory
    for( unsigned i=0 ; i<_regionPixels.size() ; i++)
    {		
	    projectCoordinate(H , &(_regionPixels[i].x) , &(_regionPixels[i].y));			
    }
    
  
    //Compute statistics about the current, needed later on for finalizing occlusion
    if( _occlusionMode)
        computeOcclusionMetrics(H);

    if( !_occlusionMode )
    {
        //Update pixel values over all levels of blurring
        for( int i=0 ; i<TOTAL_SMOOTHING_ITERATIONS ; i++)
        {	
	        //release previous version
	        cvReleaseImage(&IDash);

	        //Make copies for blurring etc naming consistent with math notation
	        IDash = cvCloneImage(imageTwo);
    				
            for( unsigned j=0 ; j<SMOOTHING_ITERATIONS[i] ; j++)
                cvSmooth(IDash,IDash,CV_GAUSSIAN,constants::SMOOTHING_MASK_SIZE,constants::SMOOTHING_MASK_SIZE); 	//Blur target image

	        int index = i ;

	        //Upate pixel model
	        updatePixelModel(IDash,IDashOccColor,index,  i == TOTAL_SMOOTHING_ITERATIONS -1 ? 1 : 0);	
        }
    }

    //Free memory
    cvReleaseImage(&IDash);
    cvReleaseImage(&IDashOccColor);
}





//Estimates which pixels are currently occluded
int objectTracker::estimateOccludedPixels(CvMat *A,IplImage *IDashOccColor)
{
    cout<<"\n\n Estimating occluded pixels";
	
	_unOccludedPixels = 0;
	_occlusionMode = 0;
    _NCCOcclusionMode = 0;
    int occludedPixels = 0; //Total occluded pixels out of the pixels being tracked
    

     //Estimate whether each pixel is occluded or not
     for( unsigned i=0 ; i<_regionPixels.size() ; i++)
     {		           
   
	        double Hx = _regionPixels[i].x , Hy = _regionPixels[i].y;
	        projectCoordinate(A , &Hx , &Hy);

           if( !(Hx - constants::MARGIN < 0 || Hx + constants::MARGIN - IDashOccColor->width  > 0 ||
                 Hy - constants::MARGIN < 0 || Hy + constants::MARGIN - IDashOccColor->height  > 0) )
           {
          
                CvScalar estimatedPixelValue = subPixelValueRGB(IDashOccColor,Hx,Hy);

                //Check if the currently estimated value is an outlier in the distribution of the pixel
               if ( isOutlier(i,estimatedPixelValue) )
                    {
                        _isOccluded[i] = 1;
                        occludedPixels++;  
                    }
                    else
                        _isOccluded[i] = 0;
           }
           else
           {
                _isOccluded[i] = 1;
                occludedPixels++;  
           }
                	
        }//pixels loop
		
				
    cout<<"\n\n Total occluded pixels "<<occludedPixels<<"\n";

    if( occludedPixels > 0 )
    {
        _occlusionMode = 1;
    
        if( _currentParticle == constants::IDENTITY_PARTICLE )
        {
            if( (occludedPixels + 0.0) / _regionPixels.size() > constants::OCC_PERCENTAGE_FOR_IDENTITY_PARTICLE)
               _NCCOcclusionMode = 1;
            
        }
        else if( (occludedPixels + 0.0) / _regionPixels.size() > constants::OCC_PERCENTAGE_FOR_PARTICLE)
            _NCCOcclusionMode = 1;
        
    }

    //Compute total unoccluded pixels
    _unOccludedPixels = (int)_regionPixels.size() - occludedPixels;

    if( (occludedPixels + 0.0) / _regionPixels.size() > constants::TRACTABLE_OCCLUSION_THRESHOLD )
        return 1;

    return 0;
    		
}


//Computes statistics needed later on for finalizing the occlusion process
void objectTracker::computeOcclusionMetrics(CvMat *H)
{
    cout<<"\n\n Computing occlusion metric";

	//*********** Setup transform matrix ************

	//if there was no occlusion previously
	if( _previousOcclusionMode == 0 )
	{	
        if( this->_cleanTransform != NULL )
            cvReleaseMat(&_cleanTransform);

        this->_cleanTransform = cvCreateMat(H->rows,H->cols,H->type);
        cvmInvert( H , this->_cleanTransform);	
	}
	else
	{
		CvMat *inv = cvCreateMat(H->rows,H->cols,H->type);
        CvMat *temp = cvCreateMat(H->rows,H->cols,H->type);
        cvmInvert(H,inv);
		cvmMul(_cleanTransform,inv,temp);
		cvReleaseMat(&_cleanTransform);
		_cleanTransform = cvCloneMat(temp);
		cvReleaseMat(&temp);
        cvReleaseMat(&inv);
	}


    //Average out all the individual gaussian variances to estimate a variance for each channel
    _tempModelVariances = cvScalar(0.0 , 0.0 , 0.0);
    
    for( unsigned i=0 ; i<_regionPixels.size() ; i++)
        for( int k=0 ; k<3 ; k++)
            _tempModelVariances.val[k] += _colorPixelVariances[i].val[k];

    for( int k=0 ; k<3 ; k++)
        _tempModelVariances.val[k] /= _regionPixels.size();

    //TESTING AVERAGING ESTIMATE ACCURACY
   /* CvScalar testVariances = cvScalar(0.0 , 0.0 , 0.0);

    for( int i=0 ; i<_regionPixels.size() ; i++)
        for( int k=0 ; k<3 ; k++)
            testVariances.val[k] += pow( modelVariances.val[k] - _colorPixelVariances[i].val[k] , 2 );

    for( int k=0 ; k<3 ; k++)
        testVariances.val[k] /= _regionPixels.size();

    printf("\n %f %f %f",testVariances.val[0] , testVariances.val[1],testVariances.val[2]);*/

}

//Frees all the memory used by the class after tracking is done
void objectTracker::freeMemory()
{    
_regionPixels.clear();
_regionPixelMeans.clear();
_regionPixelVariances.clear();

if( _IDashOccColor != NULL )
    cvReleaseImage(&_IDashOccColor);

_colorPixelMeans.clear();
_colorPixelVariances.clear();

if( _preCutSceneFrame != NULL )
    cvReleaseImage(&_preCutSceneFrame);

if( _preCutSceneFrameColor != NULL)
    cvReleaseImage(&_preCutSceneFrameColor);

_isOccluded.clear();
_occludedPoints.clear();

if( _cleanImage != NULL )
    cvReleaseImage(&_cleanImage);

if( _cleanTransform != NULL )
    cvReleaseMat(&_cleanTransform);

if( _previousH != NULL )
    cvReleaseMat(&_previousH);

_pixelModelSigmasNCC.clear();

}


//Generates an occlusion mode
IplImage* objectTracker::finalizeOccludedPixels(CvMat *H,IplImage *IDashOccColor , IplImage *cleanImage , CvScalar modelVariances, vector<svlPoint2d> polygon , double margin 
                                           , double subSampleRatio , double stepSizeX , double stepSizeY)
{

    //Holds the final occlusion model   
    IplImage *occlusionModel = NULL;

    

    //If frame was occlused during the inital processing of the frame
    if( cleanImage != NULL && H != NULL)
    {
    
        cvCvtColor(IDashOccColor,IDashOccColor,CV_RGB2Lab);

            	
        double minX = 0.0 , maxX = 0.0 , minY = 0.0 , maxY = 0.0;
        findMinMax(polygon,0,minX,minY,maxX,maxY);
    	
        int row = 0 , col =0 ; //Indices to compute the size of the matrix

        //Create occlusion model of correct size
        occlusionModel = cvCreateImage(cvSize( (int)((maxX - minX + 1 + 2 * margin) / (stepSizeX) ) , (int)((maxY - minY +1 + 2*margin) / (stepSizeY) )),IPL_DEPTH_8U,1);

           
        for( unsigned i=0 ; i<SMOOTHING_ITERATIONS[OCC_SMOOTHING_INDEX] ; i++)
            cvSmooth(IDashOccColor,IDashOccColor,CV_GAUSSIAN,constants::SMOOTHING_MASK_SIZE,constants::SMOOTHING_MASK_SIZE); 
       
       
        //#DEBUG
        //ofstream out(objectTracker::_tempOut.c_str() , ios::out);
        //cout<<"\n******* "<<objectTracker::_tempOut.c_str();
        //out<<minX - margin<<" "<<minY - margin<<" "<<maxX + margin<<" "<<maxY + margin<<" "<<stepSizeX<<" "<<stepSizeY<<"\n";
        //#DEBUG#
           
	    //Iterate over the enclosing rectangle
	    for( double y=minY - margin ; y<=maxY + margin; y+=stepSizeY , row++)
	    {
            col = 0;

		    for( double x=minX - margin ; x<=maxX + margin; x+=stepSizeX , col++)
		    {
                  
			    svlPoint2d p(x,y);
             				
				double Hx = p.x , Hy = p.y;
				projectCoordinate(H , &Hx , &Hy);

                //Filter points in the occlusion model if they lie outside the image 
                if( !(p.x - constants::MARGIN < 0 || p.x + constants::MARGIN - IDashOccColor->width  > 0 ||
                    p.y - constants::MARGIN < 0 || p.y + constants::MARGIN - IDashOccColor->height  > 0) && 
                    !(Hx - constants::MARGIN < 0 || Hx + constants::MARGIN - IDashOccColor->width  > 0 ||
                    Hy - constants::MARGIN < 0 || Hy + constants::MARGIN - IDashOccColor->height  > 0))
                {
            
				    CvScalar cleanValue = subPixelValueRGB(cleanImage,Hx,Hy);
                
                    CvScalar currentValue = subPixelValueRGB(IDashOccColor,p.x,p.y);
                
                    //Set value in occlusion model after checking outlier status of the pixel distribution for the
                    //given lab value
                    if( isOutlier(cleanValue,modelVariances,currentValue) )
				        CV_IMAGE_ELEM(occlusionModel,unsigned char,row,col) = OCCLUDED;
                    else
                        CV_IMAGE_ELEM(occlusionModel,unsigned char,row,col) = FREE;

                    //#DEBUG#
                    //out<<visionUtilities::computeMahalnobis(cleanValue,modelVariances,currentValue);
                    //if( x != maxX + margin -1 )
                    //   out<<" ";
                    //#DEBUG#
                    

                }//if not outside image
                        
		    }//x

            //#DEBUG#
            //out<<"\n";
            //#DEBUG#

	    }//y
    
        //#DEBUG#
        //out.close();
        //#DEBUG#
       
        //**TEST NEED FOR THIS AGAIN**

        //Resize occlusion model to overcome incosistencies in the sampling process
        //to remove false spaces at the ends which cause morphological operations to fail
        //IplImage *temp = cvCreateImage(cvSize(col,row),occlusionModel->depth,occlusionModel->nChannels);
        //cvSetImageROI(occlusionModel,cvRect(0,0,col,row));
        //cvCopyImage(occlusionModel,temp);
        //cvResetImageROI(occlusionModel);
        //cvReleaseImage(&occlusionModel);
        //occlusionModel = cvCloneImage(temp);
        //cvReleaseImage(&temp);

        //************************************************************************
        
        //Remove false occlusions
        visionUtilities::removeSmallComponents(occlusionModel,constants::CC_AREA_THRESHOLD,0,1);

        //Remove holes in occlusions
        visionUtilities::removeSmallComponents(occlusionModel,constants::CC_AREA_THRESHOLD,1,1);
         
        //**********************************************************************

     
        //Perform morphological operations to fill small holes and erode small noise occlusion patches
        //cvErode(occlusionModel,occlusionModel,NULL,8);
        //cvDilate(occlusionModel,occlusionModel,NULL,7);
        cvMorphologyEx(occlusionModel,occlusionModel,NULL,NULL,CV_MOP_CLOSE,1); 

        //Setup the 3 pixel inline which is also used for sub pixel interpolation
        //to make it look realistic
        for( row=0 ; row < occlusionModel->height ; row++)
        {
          int occlusionOn = 0;

            for( col=0 ; col < occlusionModel->width ; col++)
            {

                int pixelValue = CV_IMAGE_ELEM(occlusionModel,unsigned char,row,col);

                    if( pixelValue  == OCCLUDED )
				    {			
                        if( occlusionOn++ < 3 )
                        {
                            CV_IMAGE_ELEM(occlusionModel,unsigned char,row,col) = ADD;
                        }
                       
				    }
                    else if( pixelValue == FREE )
                    {
                        if( occlusionOn != 0 )
                        {
                            for( int k=1 ; k<=3 ; k++)
                            {
                                if( col-k >= 0 )
                                    CV_IMAGE_ELEM(occlusionModel,unsigned char,row,col-k) = ADD;
                            }                            
                        }
                        else
                        {
                            CV_IMAGE_ELEM(occlusionModel,unsigned char,row,col) = 0;
                            
                        }

                        occlusionOn = 0;
                    }
                    
            }//x

            
        }//y
        
    }
      

    cvReleaseImage(&IDashOccColor); //Free memory

    return occlusionModel; //Return the occlusion model

}




int objectTracker::isOutlier(CvScalar pixelMean , CvScalar pixelVariance , CvScalar pixelValue)
{
    double result = 0.0;
    int isOutlier = 1;

    for( int k=0 ; k<3 ; k++)
        result += pow( pixelValue.val[k] - pixelMean.val[k] , 2 )
        / ( pixelVariance.val[k] + constants::VARIANCE_REGULARIZATION[k] );


    if ( result < constants::OUTLIER_THRESHOLD )
        isOutlier = 0;

    return isOutlier;
}




//Checks if the point is very close to an estimated occluded pixel
int objectTracker::isOccluded(IplImage * occlusionModel ,int row , int col)
{
	int result = 0;
 	
    if( occlusionModel != NULL && CV_IMAGE_ELEM(occlusionModel,unsigned char , row , col) == OCCLUDED )
        result = 1;
    else if (occlusionModel != NULL && CV_IMAGE_ELEM(occlusionModel,unsigned char , row , col) == ADD)
        result = 2;

 
	return result;
}


//Updates the pixel model at the blurring specified up the index
void objectTracker::updatePixelModel(IplImage *IDash,IplImage *IDashOccColor,int index, int update)
{	
	//Compute weights via exponential decay	/ laplacian / gaussian
	double lambda = 0.0009;//0.0009;//0.0009 //0.09;//0.1;
    double lambdaColor = -0.09;//0.0009;//0.0009 //0.09;//0.1;
	
	//static double previousWeight = exp((double)lamba * abs(1)); //Rev
	static double previousWeight = exp(-(double)lambda * 1); //Fwd
    static double previousColorWeight = exp(-(double)lambdaColor * 1); //Fwd
	//static double previousWeight = 1; //Avg 
	//static double previousWeight = 1; //Dynamic
	
	
	//Compute weight for the current pixel value
	//double currentWeight = exp((double)lamba * _currentFrame); //Rev
	double currentWeight = exp(-(double)lambda * _currentFrame); //Fwd
    double currentColorWeight = exp(-(double)lambdaColor * _currentFrame); //Fwd
    
		
	//Update pixel locations in the trajectory and recompute pixel values
	for( unsigned i=0 ; i<_regionPixels.size() ; i++)
	{
		if( _isOccluded[i] )
        {            
			continue;
        }

				
        double currentPixelValue = subPixelValue(IDash,_regionPixels[i].x,_regionPixels[i].y);
        

		_regionPixelMeans[index][i] = _regionPixelMeans[index][i] * previousWeight + currentPixelValue * currentWeight;
		_regionPixelMeans[index][i] /= (previousWeight + currentWeight);

        _regionPixelVariances[index][i] = _regionPixelVariances[index][i] * previousWeight 
                                            + pow( currentPixelValue - _regionPixelMeans[index][i] , 2) * currentWeight;
        _regionPixelVariances[index][i] /= (previousWeight + currentWeight);

	}

    //if its the last iteration for the update... update color model means
    if( update )
    {
        for( unsigned i=0 ; i<_regionPixels.size() ; i++)
        {
            if( _isOccluded[i] )
			    continue;

            CvScalar currentColorValue = subPixelValueRGB(IDashOccColor,_regionPixels[i].x,_regionPixels[i].y);

  
            ////Update all three channels
            for( int k=0 ; k<3 ; k++)
            {            
                _colorPixelMeans[i].val[k] = _colorPixelMeans[i].val[k] * previousColorWeight + currentColorValue.val[k] * currentColorWeight;
                _colorPixelMeans[i].val[k] /= (previousColorWeight + currentColorWeight);

                _colorPixelVariances[i].val[k] = _colorPixelVariances[i].val[k] * previousColorWeight 
                    +  pow( _colorPixelMeans[i].val[k] - currentColorValue.val[k] , 2) * currentColorWeight;

                _colorPixelVariances[i].val[k] /= (previousColorWeight + currentColorWeight);
            
            }

        }//pixels i
       
    }
		
	
	printf("\n %d Pweight = %f Cweight %f",index,previousWeight,currentWeight);
    printf("\n %d PColorWeight = %f CurrentColorweight %f",index,previousColorWeight,currentColorWeight);

	if( update )
    {
		previousWeight += currentWeight;
        previousColorWeight += currentColorWeight;
    }
}



//Computes the value of the differential objective function given all the variables and parameters
double objectTracker::delJ(IplImage *IDash, CvMat *H, int y , int x ,int index)
{
	//********************************* Compute Del J1 ********************************
	
	//Compute the matrix HDash for numerical differentiation of J1
	CvMat *HDash = cvCloneMat(H);
	CV_MAT_ELEM(*HDash,double,y,x) = CV_MAT_ELEM(*H,double,y,x) + constants::DELTA_HJ;
	
	
	//Holds differential of sub objective function J1
	double delJ1 = NCC(IDash,H,index) - NCC(IDash,HDash,index);			
		
	delJ1 /= constants::DELTA_HJ  ;

	cvReleaseMat(&HDash);

	

	//********************************  Compute Del J2 *********************************

	//Holds differential of sub objective function J2
	double delJ2 = 0.0;

    if( _previousH != NULL )
    {
	    delJ2 = 2 * ( CV_MAT_ELEM(*H,double,y,x) - CV_MAT_ELEM(*_previousH,double,y,x) );
    }

	//********************************  Del J3 *********************************
	
	//******************************** Return final value *****************************

	return (delJ1 + constants::OPTIMIZATION_REGULARIZATION * delJ2 ) ;
}


//Computes the value of the objective function given all the variables and parameters
double objectTracker::J(IplImage *IDash, CvMat *H, int index)
{	

	//******************************* Compute J1 ************************************

	//Holds value of the sub objective function J1
	double J1 = 1 - NCC(IDash,H,index);
	
	//******************************* Compute J2 **************************************

	//Holds value of sub objective function J2
	double J2 = 0.0;

    if( _previousH != NULL )
    {
	    //Compute Frobenius norm
	    for( int i=0 ; i<3 ; i++)
	    {
		    for( int j=0 ; j<3 ; j++)
		    {
			    J2 += pow( CV_MAT_ELEM(*H,double,i,j) - CV_MAT_ELEM(*_previousH,double,i,j) ,2 );
		    }
	    }
    }


	//******************************* Return final value ************************************

	return (J1 + constants::OPTIMIZATION_REGULARIZATION * J2);
}

//Computes the normalized cross correaltion in the given neighbourhood around the the 2 points in different images
double objectTracker::NCC(IplImage *IDash , CvMat *H  , int index)
{
	
	//****************** Compute Means *************************
	double IDashBar = 0.0;
    unsigned totalPixels = _regionPixels.size();

	vector<double> subPixelValues(totalPixels);
    vector<double> subPixelProjectionsX(totalPixels);
    vector<double> subPixelProjectionsY(totalPixels);
		
	for( unsigned i=0 ; i<totalPixels ; i++)
	{
		if(_isOccluded[i] )
            continue;
        		
		double Hx = _regionPixels[i].x , Hy = _regionPixels[i].y;
		projectCoordinate(H , &Hx , &Hy);
        subPixelProjectionsX[i] = Hx;
        subPixelProjectionsY[i] = Hy;
		        

        //League_5 with firt particle results in error if this statement is not present.. check
        if(	Hx - constants::MARGIN < 0 || Hx + constants::MARGIN - IDash->width  > 0 
            || Hy - constants::MARGIN < 0 || Hy + constants::MARGIN - IDash->height  > 0 )
	    	return  0.0;

		double value = subPixelValue(IDash,Hx,Hy);

		IDashBar += value;
		subPixelValues[i] = value;
	}


	IDashBar /= _unOccludedPixels;



	//****************** Compute Variances & Numerator*************************
	
    double IDashSigma = 0.0 , denominator = 0.0 , numerator = 0.0 , subtract = 0.0;

	for( unsigned i=0 ; i<_regionPixels.size() ; i++)
	{
		if(_isOccluded[i] )
			continue;

		double IDashValue = subPixelValues[i] - IDashBar;
		
       // double temp = _pixelModelSigmasNCC[i] * IDashValue; 

        //Code for creating the truncated NCC plot
        if( _NCCOcclusionMode == 0 )
        {
            IDashSigma += pow( IDashValue , 2);
            numerator += _pixelModelSigmasNCC[i] * IDashValue; //repalce with temp
        }
        else if( _NCCOcclusionMode == 1 && isOutlier(i,subPixelValueRGB(_IDashOccColor,subPixelProjectionsX[i],subPixelProjectionsY[i]) ) )// temp < FALLOFF_VALUE  )
        {
            subtract += _pixelModelSigmasNCC[i] * _pixelModelSigmasNCC[i];
        }
        else
        {
            IDashSigma += pow( IDashValue , 2);
            numerator += _pixelModelSigmasNCC[i] * IDashValue; //replace with temp
        }

	}

	//Product of variances
    denominator = sqrt( (_pixelModelSigmaNCC - subtract) * IDashSigma);

    subPixelValues.clear();
    subPixelProjectionsX.clear();
    subPixelProjectionsY.clear();
    
	if( denominator == 0 )
	{
		//printf("\n********* deno 0 %e %e",numerator,denominator);
		//char gg;
		//scanf("%c",&gg);
		return 0.0;
	}	

	double ncc = numerator / denominator;

	//if( ncc < 0.0 )
	//{
	//	//printf("\n********* < -1 ???? %e %e",numerator,denominator);
	//	//char gg;
	//	//scanf("%c",&gg);
	//	return 0.0;
	//}

	if( ncc > 1.0 )
	{	
		//printf("\n********* == 1 ???? %e %e %e",numerator,denominator,numerator - denominator);
		//char gg;
		//scanf("%c",&gg);
		return 1.0;
	}
	
	return ncc;
}


//Buffers statistics for NCC of the model to speed up NCC
void objectTracker::populateModelNCCStats(int index)
{
      
//****************** Compute Means *************************
   double IBar = 0.0;
  		
	for( unsigned i=0 ; i<_regionPixels.size() ; i++)
	{
		if( _isOccluded[i] )
            continue;
        
        IBar += _regionPixelMeans[index][i];        
	}
    
    IBar /= _unOccludedPixels;
        

	//****************** Compute Variances & Numerator*************************

    _pixelModelSigmaNCC = 0.0;
    _pixelModelSigmasNCC.clear();


	for( unsigned i=0 ; i<_regionPixels.size() ; i++)
	{
		if(_isOccluded[i] )
        {
            _pixelModelSigmasNCC.push_back(0);
			continue;
        }
	
		double IValue = _regionPixelMeans[index][i] - IBar;

        _pixelModelSigmasNCC.push_back(IValue);
        _pixelModelSigmaNCC += pow( IValue , 2);
		
	}

}


//************ Data Retrieval Functions **********

IplImage *objectTracker::getPreCut()
{
    return _preCutSceneFrame;
}

IplImage *objectTracker::getPreCutColor()
{
    return _preCutSceneFrameColor;
}


CvMat *objectTracker::getCleanTransform()
{
    return _cleanTransform;
}

IplImage* objectTracker::getCleanImage()
{
    return _cleanImage;
}

CvScalar objectTracker::getModelVariances()
{
    return _tempModelVariances;
}

//IplImage *objectTracker::getOcclusionModel()
//{
//    return _occlusionModel;
//}

//*********************************************


double objectTracker::subPixelValueFloat( IplImage *I , svlPoint2d Pi)
{
	//Floating point of the coordinates
	double delX = Pi.x - (int)Pi.x;
	double delY = Pi.y - (int)Pi.y;

	double x1 = CV_IMAGE_ELEM(I,float,(int)(Pi.y),(int)(Pi.x)) 
		+  delX * ( CV_IMAGE_ELEM(I,float,(int)(Pi.y) ,(int)(Pi.x + 1)) - CV_IMAGE_ELEM(I,float,(int)(Pi.y),(int)(Pi.x)) );

	double x2 = CV_IMAGE_ELEM(I,float,(int)(Pi.y+1) ,(int)(Pi.x)) 
		+ delX * ( CV_IMAGE_ELEM(I,float,(int)(Pi.y+1) ,(int)(Pi.x + 1)) - CV_IMAGE_ELEM(I,float,(int)(Pi.y+1) ,(int)(Pi.x)) );

	double final = x1 + delY * (x2 - x1);

	return final;
}




//Maps points lying within a polygon to anohter polygon.. whose corner mappings are known
svlPoint2d objectTracker::findMappingInPolygon(vector<svlPoint2d> polygon ,vector<svlPoint2d> targetPolygon ,svlPoint2d p)
{

	double uSlope = (polygon[1].y - polygon[0].y) / (polygon[1].x - polygon[0].x);
	double lSlope = (polygon[2].y - polygon[3].y) / (polygon[2].x - polygon[3].x);

	//printf("\n\nSlopes %f %f",uSlope,lSlope);

	double Yu = polygon[0].y + uSlope * ( p.x - polygon[0].x );
	double Yl = polygon[3].y + lSlope * ( p.x - polygon[3].x );

	//printf("\n\nYs %f %f",Yu,Yl);

	double D01 = sqrt( pow( polygon[1].x - polygon[0].x , 2) + pow( polygon[1].y - polygon[0].y , 2) );
	double t01 = sqrt( pow( p.x - polygon[0].x , 2) + pow( Yu - polygon[0].y , 2) );

	double D32 = sqrt( pow( polygon[2].x - polygon[3].x , 2) + pow( polygon[2].y - polygon[3].y , 2) );
	double t32 = sqrt( pow( p.x - polygon[3].x , 2) + pow( Yl - polygon[3].y , 2) );

	double Dul = sqrt( pow( p.x - p.x , 2) + pow( Yl - Yu , 2) );
	double tul = sqrt( pow( p.x - p.x , 2) + pow( Yu - p.y , 2) );

	//printf("\n\nDs 01 %f %f \n32 %f %f \nul %f %f",D01,t01,D32,t32,Dul,tul);

	//double D01Dash = sqrt( pow( targetPolygon[1].x - targetPolygon[0].x , 2) + pow( targetPolygon[1].y - targetPolygon[0].y , 2) );
	//double D32Dash = sqrt( pow( targetPolygon[2].x - targetPolygon[3].x , 2) + pow( targetPolygon[2].y - targetPolygon[3].y , 2) );

	//printf("\n\nDs 01 %f \n32 %f \n",D01Dash,D32Dash);
			
	
	svlPoint2d PDashu , PDashl;
	
	PDashu.x = targetPolygon[0].x + (t01 / D01) * (targetPolygon[1].x  - targetPolygon[0].x );
	PDashu.y = targetPolygon[0].y + (t01 / D01) * (targetPolygon[1].y  - targetPolygon[0].y );

	PDashl.x = targetPolygon[3].x + (t32 / D32) * (targetPolygon[2].x  - targetPolygon[3].x );
	PDashl.y = targetPolygon[3].y + (t32 / D32 )* (targetPolygon[2].y  - targetPolygon[3].y );

	//printf("\n\npd u %f %f \npd l %f %f",PDashu.x,PDashu.y,PDashl.x,PDashl.y);

	//double DulDash = sqrt( pow( PDashu.x - PDashl.x , 2) + pow( PDashu.y - PDashl.y , 2) );

	svlPoint2d PDash;

	PDash.x = PDashu.x + (tul/Dul) * (PDashl.x - PDashu.x);
	PDash.y = PDashu.y + (tul/Dul) * (PDashl.y - PDashu.y);

	//printf("\n************");

	return PDash;

}


void objectTracker::computeEdgePixels(IplImage *I , IplImage *IColor,vector<svlPoint2d> polygon , int visualize )
{

	//Clean data members before insertion
    _regionPixels.clear();	
	_regionPixelMeans.clear();
	_regionPixelVariances.clear();
    _colorPixelMeans.clear();
    _colorPixelVariances.clear();
    	

	IplImage *initial = cvCloneImage(I);
    IplImage *p = NULL;	  //Image object for visualizing the edge points
	  

	//Variables for holding the possibly larger rectangle surronding the polygon
	double minX = 0.0 , minY = 0.0 , maxX = 0.0 , maxY = 0.0;
	
	//Compute enclosing rectangle with a margin
	findMinMax(polygon,0,minX,minY,maxX,maxY);

    //#DEBUG# //Create image object to hold a view of how the region models looks like
	//_model = cvCreateImage(cvSize((int)(maxX-minX+1),(int)(maxY-minY+1)),I->depth,I->nChannels);
    //#DEBUG#

	if( visualize )
		p = cvCloneImage(I);


    //Sample every single pixel in the region beign tracked
	for( double y=minY ; y<=maxY ; y++)
	{
		for( double x=minX ; x<=maxX ; x++)
		{
	
	    	svlPoint2d c(x,y);

	    	//Add only if the point lies in the polygon
	    	if( inPolygon(polygon , c) )
	    	{
	    		_regionPixels.push_back(c);

                //#DEBUG //Used for plotting values / occlusion onto the model
	    		//_originalLocations.push_back(c-svlPoint2d(minX,minY));
                //#DEBUG
	
	    		if( visualize )
	    			cvDrawRect(p,cvPoint((int)x,(int)y),cvPoint((int)x,(int)y),cvScalar(0,0,0));
	    		
	    	}
	
    	}
    }

   

	//************************************ Add pixel values at multiple blurring scales ***************

    CvScalar initialVariance = INITIAL_VARIANCE;

    for( int i=0 ; i<TOTAL_SMOOTHING_ITERATIONS ; i++ )
	{		
		//release previous version
		cvReleaseImage(&I);
        
		//Make copies for blurring etc naming consistent with math notation
		I = cvCloneImage(initial);
                
        for( unsigned j=0 ; j<SMOOTHING_ITERATIONS[i] ; j++)
            cvSmooth(I,I,CV_GAUSSIAN,constants::SMOOTHING_MASK_SIZE,constants::SMOOTHING_MASK_SIZE);           
        
		int index = i;
        

		//Insert empty vector for this level of blurring
		_regionPixelMeans.push_back(vector<double>());
        _regionPixelVariances.push_back(vector<double>());
     

		for( unsigned j=0 ; j<_regionPixels.size() ; j++)
		{
            double value = subPixelValue(I,_regionPixels[j].x,_regionPixels[j].y);
            
			_regionPixelMeans[index].push_back( value );
            _regionPixelVariances[index].push_back( initialVariance.val[0] );
			
            //#DEBUG#
			//CV_IMAGE_ELEM(_model,unsigned char, utilities::roundOff(_regionPixels[j].y - minY),utilities::roundOff(_regionPixels[j].x - minX) ) = value;
            //#DEBUG#
		}
				
    }//Grayscale smoothing levels heirarchy


    //Populate color model statistics at a single different level of blurring
    for( unsigned j=0 ; j<SMOOTHING_ITERATIONS[OCC_SMOOTHING_INDEX] ; j++)
    {
        cvSmooth(IColor,IColor,CV_GAUSSIAN,constants::SMOOTHING_MASK_SIZE,constants::SMOOTHING_MASK_SIZE);           
    }
    
    
    for( unsigned j=0 ; j<_regionPixels.size() ; j++)
	{		
        CvScalar colorValue = subPixelValueRGB(IColor,_regionPixels[j].x,_regionPixels[j].y);
            
	    _colorPixelMeans.push_back( colorValue );
        _colorPixelVariances.push_back( initialVariance );           
            			
	}

	//Visualize region pixels
	if( visualize )
	{	
		cvNamedWindow("Debug Windows - Track points", 1);
		cvShowImage("Debug Windows - Track points" , p);
		cvWaitKey(-1);
		cvDestroyAllWindows();

		cvReleaseImage(&p);
	}

	//Free memory
	cvReleaseImage(&I);
    cvReleaseImage(&IColor);
	cvReleaseImage(&initial);

}


//Computes the shortest distance from the given point to one of polygon edges
double objectTracker::shortestDistanceToEdge(vector<svlPoint2d> polygon, svlPoint2d p)
{	
	svlPoint2d p1 = polygon[0] , p2;
	double minimumDistance = 9999999;

	for (unsigned i=1 ; i<=polygon.size() ; i++ )
	{
		p2 = polygon[i % polygon.size()];

		double distance = fabs( (p2.x - p1.x) * (p1.y-p.y) - (p1.x - p.x) * (p2.y - p1.y));
		distance /= sqrt( pow(p2.x - p1.x,2) + pow(p2.y - p1.y,2) );

        if( distance < minimumDistance && ( (p.x >= p1.x && p.x <= p2.x) || (p.x <= p1.x && p.x >= p2.x) ) && ( (p.y >= p1.y && p.y <= p2.y) || (p.y <= p1.y && p.y >= p2.y) ))
			minimumDistance = distance;

		p1 = p2;
	}

	return minimumDistance;
}

//Computes whether point lies in polygon or not
int objectTracker::inPolygon(vector<svlPoint2d> polygon, svlPoint2d p)
{
	//Total number of horizontal intercepts made with polygon
    int totalIntercepts = 0;
	int isInside = 0;

	svlPoint2d p1 = polygon[0] , p2;

	for (unsigned i=1 ; i<=polygon.size() ; i++ )
	{
		
		p2 = polygon[i % polygon.size()];

		if (p.y > MIN(p1.y,p2.y)) 
		{
			  if (p.y <= MAX(p1.y,p2.y)) 
			  {
				if (p.x <= MAX(p1.x,p2.x)) 
				{
					if (p1.y != p2.y) 
					{
						double xinters = (p.y-p1.y)*(p2.x-p1.x)/(p2.y-p1.y)+p1.x;
						if (p1.x == p2.x || p.x <= xinters)
						totalIntercepts++;
					}
				}
		}
		}

		p1 = p2;
	}


	//If intercepts are not even
	if( totalIntercepts % 2 == 1 )
		isInside = 1;

      return isInside;
}


//Finds the possibly larger enclosing rectangle for the given polygon
void objectTracker::findMinMax(vector<svlPoint2d> objects,int margin ,double &minX,double &minY ,double &maxX ,double &maxY)
{
	//Initial values
	minX = objects[0].x;
	maxX = objects[0].x;
	minY = objects[0].y;
	maxY = objects[0].y;

	for( unsigned i=1 ; i<objects.size() ; i++)
	{
		if( objects[i].x < minX )
			minX = objects[i].x;
		if( objects[i].x > maxX )
			maxX = objects[i].x;
		if( objects[i].y < minY )
			minY = objects[i].y;
		if( objects[i].y > maxY )
			maxY = objects[i].y;
	}


	//minX -= margin;
	//maxX += margin;
	//minY -= margin;
	//maxY += margin;
}


