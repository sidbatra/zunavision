/*****************************************************************************
** ZunaVision
** Copyright (c) 2008 
**
** FILENAME:    contentTracker.cpp
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Computes homographies and tracks content regions between frames

*****************************************************************************/

#include "contentTracker.h"

//Default constructor
contentTracker::contentTracker()
{
    //Does nothing as of now
    
}


//Tracks the given content in the given video volume
void contentTracker::trackContent(video &cVideo ,unsigned volumeIndex , int contentIndex)
{
    //Index of the centre frame in the volume for which meta data is availible
    int centreFrame = constants::VOLUME_MARGIN * 2; 

    //Retrieve tracks from current frame to be used to compute tracks for frame f+1
    contentTracks currentTracks;
    vector< vector< vector<svlPoint2d> > > correspondences;
    
    //Track the content in the next frame only if meta data for that frame is not availible or is a stub frame
    if( cVideo._frames[volumeIndex][ centreFrame + 1 ].getContentTracks(contentIndex,currentTracks) == -1 && cVideo._frames[volumeIndex][centreFrame +1]._index != -1  )
    {   
        
        cVideo._frames[volumeIndex][ centreFrame ].getContentTracks(contentIndex,currentTracks);


        if( !(cVideo._content[contentIndex]._trackingType == STILL_CAMERA) )
        {        
            //Compute SIFT features for all the frames in the volume
            for( unsigned i=0 ; i<cVideo._frames[volumeIndex].size() ; i++)
                _siftObject.computeSIFTFeatures(cVideo._frames[volumeIndex][i]);
            
            ////Compute the point correspondences amongst the frames
            correspondences = _siftObject.computeCorrespondences(cVideo._frames[volumeIndex]);
        }
        
        //If nothing is wrong with the SIFT feature correspondences
        if(  _siftObject.computeMetricsForSampling(correspondences,currentTracks) != -1 )
        {  
            cvCvtColor(cVideo._frames[volumeIndex][centreFrame +1]._colorImage,cVideo._frames[volumeIndex][centreFrame +1]._colorImage,CV_RGB2Lab);
            cvCvtColor(cVideo._frames[volumeIndex][centreFrame ]._colorImage,cVideo._frames[volumeIndex][centreFrame ]._colorImage,CV_RGB2Lab);

            double J=0.0;
            CvMat *A = NULL;
            CvMat *H = NULL;
            
            //If tracking type is still camera simply copy the affine estimate
            if( cVideo._content[contentIndex]._trackingType == STILL_CAMERA )
            {
                utilities::populateMatrix(&A , constants::IDENTITY,3,3);
                H = estimateIdentityTransform(cVideo,volumeIndex,A,cVideo._content[contentIndex],currentTracks,J,1);
            }
            else
            {
                A = generateHomographyEstimate(correspondences,volumeIndex,currentTracks , cVideo , contentIndex);
                H = estimateProjectiveTransform(cVideo,volumeIndex,A,cVideo._content[contentIndex],currentTracks,J,0);
            }

            //Tracks for the next frame
            contentTracks nextTracks;
            nextTracks.populateForNextFrame(H,A,currentTracks,contentIndex);

            
            finalizeTracking(cVideo,volumeIndex,currentTracks,nextTracks,cVideo._content[contentIndex]);
            
              
            cvReleaseMat(&H);
            cvReleaseMat(&A);
            
            cVideo._frames[volumeIndex][centreFrame + 1].setContentTracks(contentIndex,nextTracks);
            cvCvtColor(cVideo._frames[volumeIndex][centreFrame +1]._colorImage,cVideo._frames[volumeIndex][centreFrame +1]._colorImage,CV_Lab2RGB);
            cvCvtColor(cVideo._frames[volumeIndex][centreFrame ]._colorImage,cVideo._frames[volumeIndex][centreFrame ]._colorImage,CV_Lab2RGB);
            
            
        }//if sift is ok
        else
        {
            //Incase of error , (minimum number of SIFT correspondences are not found)
            //Replicate tracks from current frame for next frame
            cVideo._frames[volumeIndex][centreFrame + 1].setContentTracks(contentIndex,currentTracks);
        }

    }//if meta information for next frame is not availible

}

//Wraps up the optimization and updates the models and other data structures
void contentTracker::finalizeTracking(video &cVideo , unsigned volumeIndex, contentTracks &currentTracks,contentTracks &nextTracks,content &cContent)
{
    //Indices for relevant frames
    int nextFrameIndex = constants::VOLUME_MARGIN * 2 + 1;
    int currentFrameIndex = constants::VOLUME_MARGIN * 2;
  
    
    IplImage *IDashColor = cvCloneImage( cVideo._frames[volumeIndex][ nextFrameIndex ]._colorImage );
    
    //Smoothen a colored copy of the current frame for occlusion detection
    for( unsigned i=0 ; i<constants::SMOOTHING_ITERATIONS[constants::OCC_SMOOTHING_INDEX] ; i++)
         cvSmooth(IDashColor,IDashColor,CV_GAUSSIAN,constants::SMOOTHING_MASK_SIZE,constants::SMOOTHING_MASK_SIZE);
   
    //Improve the estimate of the occluded pixels by checking with the final projective transform
    estimateOccludedPixels(cContent,nextTracks._H,IDashColor);


	//if occlusion is detected for the first time store the clean image
    if( cContent._occlusionMode ==1 && cContent._previousOcclusionMode == 0)
    {
        if( constants::VERBOSE )
	        cout<<"\n Starting with clean resources";

        IplImage *IColor = cvCloneImage( cVideo._frames[volumeIndex][ currentFrameIndex ]._colorImage );

        for( unsigned i=0 ; i<constants::SMOOTHING_ITERATIONS[constants::OCC_SMOOTHING_INDEX] ; i++)
         cvSmooth(IColor,IColor,CV_GAUSSIAN,constants::SMOOTHING_MASK_SIZE,constants::SMOOTHING_MASK_SIZE);

        nextTracks._cleanColorImage = cvCloneImage(IColor);

        cvReleaseImage(&IColor);
        
     }
    else if(cContent._occlusionMode ==1 && cContent._previousOcclusionMode == 1)
    {
        if( constants::VERBOSE )
            cout<<"\n Maintaining clean resources";

        nextTracks._cleanColorImage = cvCloneImage(currentTracks._cleanColorImage);
    }


    //******************** Update pixel model **************************

    //Update pixel locations in the trajectory
    cContent.projectModelPixels(nextTracks._H);
  
    if( !cContent._occlusionMode )
    {
        //Update pixel values over all levels of blurring
        for( unsigned i=0 ; i<constants::TOTAL_SMOOTHING_ITERATIONS ; i++)
        {	        
	        //Make local copies for updating pixel model
            IplImage *IDash = cvCloneImage( cVideo._frames[volumeIndex][ nextFrameIndex ]._grayScaleImage );
    				
            for( unsigned j=0 ; j<constants::SMOOTHING_ITERATIONS[i] ; j++)
                cvSmooth(IDash,IDash,CV_GAUSSIAN,constants::SMOOTHING_MASK_SIZE,constants::SMOOTHING_MASK_SIZE); 	//Blur target image

            cContent.updateGrayModelValue(IDash,i);	        

            cvReleaseImage(&IDash);
        }

        cContent.updateColorModelValue(IDashColor);

        cContent._timeStep++;
    }
    else
    {
        nextTracks.computeOcclusionMetrics(IDashColor,cContent,currentTracks);
    }

    //Cycle occlusion mode to next time step
    cContent._previousOcclusionMode = cContent._occlusionMode;

    //Free memory
    cvReleaseImage(&IDashColor);

}


//Computes the best initial estimate of the homography using various particles
CvMat* contentTracker::generateHomographyEstimate(vector<vector<vector<svlPoint2d> > > correspondences , unsigned volumeIndex ,contentTracks &currentTracks , video &cVideo , int contentIndex)
{
    CvMat *A_Star = NULL;       //Holds the best affine estimate of the transform		                
    double J_Star = INT_MAX;    //Holds the value of the objective function for the best transform
    int selectedParticle = -1;  //Holds the index of the selected particle
    int *siftTypes = new int[constants::TOTAL_PARTICLES];   //Indices of the SIFT types of the particles
    int *sampleTypes = new int[constants::TOTAL_PARTICLES]; //Indices of the total number of samples of the SIFT features

    for( int i=0 ; i<constants::SIFT_PARTICLE_TYPES ; i++)
    {
        siftTypes[i] = siftTypes[i+constants::SIFT_PARTICLE_TYPES] = i;
        sampleTypes[i] = 0;
        sampleTypes[i+constants::SIFT_PARTICLE_TYPES] = 1;
    }

    //Additional particles which uses all the SIFT matches
    siftTypes[constants::SIFT_PARTICLE_TYPES * 2] = 0;
    sampleTypes[constants::SIFT_PARTICLE_TYPES * 2] = 2;

    //Place stubs for other different particles
    for( int i=constants::SIFT_PARTICLE_TYPES * 2 + 1 , type = 0; i<constants::TOTAL_PARTICLES ; i++, type++)
    {
        siftTypes[i] = type;
        sampleTypes[i] = -1;
    }
    

    //Iterate over each particle
    for( int p=0 ; p< constants::TOTAL_PARTICLES ; p++)
    {
      if( constants::VERBOSE )
        cout<<"\n\n Particle being tested "<<p<<" ";
       _currentParticle = p;
              
       CvMat *A = NULL;

       if( sampleTypes[p] != -1 )
       {
           if( constants::VERBOSE )
                printf(" SIFT %d SAMPLE %d",siftTypes[p],sampleTypes[p]);

            //Computes an estimate of the homography based on the current sampling strategy
            A = _siftObject.sampleSIFTFeatures(correspondences , currentTracks ,siftTypes[p] , sampleTypes[p]);
            
       }
       else if( sampleTypes[p] == -1)   //Handles additional non SIFT based particles
       {
       
           if( siftTypes[p] == 0 )
           {
               if( constants::VERBOSE )
                    cout<<" VELOCITY";

                if( currentTracks._H == NULL)
                    continue;

                A = cvCloneMat(currentTracks._H);
           }
           else if( siftTypes[p] == 1 )
           {
               if( constants::VERBOSE )
                    cout<<" IDENTITY";

                utilities::populateMatrix(&A , constants::IDENTITY,3,3);
           }
       
       
       }//if sampleTypes[p] == -1
   

        double J_reference = evaluateParticle(cVideo ,volumeIndex, A, currentTracks,contentIndex);  
        
    
            
          //If optimization is better than the previous ideal case replace A_Star
          if( J_reference < J_Star )
          {
              if( constants::VERBOSE )
                cout<<" - Particle selected ";

              if( A_Star != NULL)
                cvReleaseMat(&A_Star);
          
              A_Star = cvCloneMat(A);
              J_Star = J_reference;

              selectedParticle = p;
          }
        


         cvReleaseMat(&A);

    } //Particle p

    if( constants::VERBOSE )
        cout<<" \n Final particle selected - "<<selectedParticle<<" with J value "<<J_Star;

    delete []siftTypes;
    delete []sampleTypes;

    return A_Star;
}


//Runs a restricted optimization to evaluate the given particle
double contentTracker::evaluateParticle(video &cVideo , unsigned volumeIndex , CvMat *A, contentTracks &currentTracks, int contentIndex)
{
    double J = 0.0;

    CvMat *H = estimateProjectiveTransform(cVideo, volumeIndex, A,cVideo._content[contentIndex],currentTracks,J,1);
    cvReleaseMat(&H);

    return J;

}



//Run Newton's method to improve the estimation of the transform
CvMat* contentTracker::estimateProjectiveTransform(video &cVideo , unsigned volumeIndex , CvMat *A, content &cContent , contentTracks &currentTracks, double &J_Reference , int constrainIterations)
{
    CvMat *H = cvCloneMat(A);
    int nextFrameIndex = constants::VOLUME_MARGIN * 2 + 1;
    
    //If the content model has not been populated
    if( cContent._pixelLocations.empty() )
            populateModel(cVideo._frames[volumeIndex], cContent, currentTracks , 0); //Populate region model

    //Make local copies for opitmization
    IplImage *IDash = cvCloneImage( cVideo._frames[volumeIndex][ nextFrameIndex ]._grayScaleImage );
    IplImage *IDashColor = cvCloneImage( cVideo._frames[volumeIndex][ nextFrameIndex ]._colorImage );
    

    unsigned currentAnnealingLevel = 0;      //Index of the current annealing level for the optimization

    //Blur the images to match the current annealing level
    for( unsigned i=0 ; i<constants::SMOOTHING_ITERATIONS[ currentAnnealingLevel ] ; i++)
        cvSmooth(IDash,IDash,CV_GAUSSIAN,constants::SMOOTHING_MASK_SIZE,constants::SMOOTHING_MASK_SIZE);

    for( unsigned i=0 ; i<constants::SMOOTHING_ITERATIONS[ constants::OCC_SMOOTHING_INDEX ] ; i++)
        cvSmooth(IDashColor,IDashColor,CV_GAUSSIAN,constants::SMOOTHING_MASK_SIZE,constants::SMOOTHING_MASK_SIZE);


    //Removes model pixels which may be occluded from the tracking process 
    if( estimateOccludedPixels( cContent,A,IDashColor) || cContent._trackingType == EASIER_MOTION )
   {
       if( constants::VERBOSE )
        cout<<"\n Occlusion is greater than tractable OR Easier motions \n";

       //If the entire region is out of bounds, set J_Reference to max value
       if( cContent._unOccludedPixels == 0 )
       {
            J_Reference = 1.0;
       }
       else
       {
           cContent.populateNCCStats(currentAnnealingLevel);  //Cache stats for the future NCC operations
           J_Reference =J(IDash,IDashColor,H,currentAnnealingLevel,cContent,currentTracks);; 
       }

       	//Free memory
	    cvReleaseImage(&IDash);
        cvReleaseImage(&IDashColor);

        return H;
   }

    cContent.populateNCCStats(currentAnnealingLevel);  //Cache stats for the future NCC operations

    double J_i_minus_1 = J(IDash,IDashColor,H,currentAnnealingLevel,cContent,currentTracks); //Compute initial value of the objective function

    

    //Perform the optimization over the transform for the max number of iterations
    for( unsigned i=0 ; i<constants::MAX_ITERATIONS ; i++ )
    {
        if( constants::VERBOSE )
            printf("\n J_Initial %e %d",J_i_minus_1,cContent._unOccludedPixels);
        //Holds all the differentials of the objective function as a vector
		double delJ_vector[9];
		
		for( int y=0 ; y<3 ; y++)
		    for( int x=0 ; x<3 ; x++)
                delJ_vector[y * 3 + x] = delJ(IDash,IDashColor,H,y,x,currentAnnealingLevel,cContent,currentTracks);   //Compute and store the differential wrt diff parameters

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
				
                double Jjk  = J(IDash,IDashColor,HDash,currentAnnealingLevel,cContent,currentTracks);
				double Jj   = J(IDash,IDashColor,HDashj,currentAnnealingLevel,cContent,currentTracks);
				double Jk   = J(IDash,IDashColor,HDashk,currentAnnealingLevel,cContent,currentTracks);
				
				double Hjk = Jjk - Jj - Jk + J_i_minus_1;

				Hjk /= (constants::DELTA_HJ * constants::DELTA_HJ);


                CV_MAT_ELEM(*Hessian,double,j,k) = Hjk;
                CV_MAT_ELEM(*Hessian,double,k,j) = Hjk;

				cvReleaseMat(&HDash);
				cvReleaseMat(&HDashj);
				cvReleaseMat(&HDashk);

			}
		}

        //Invert the hessian
		CvMat *HessianInv = cvCreateMat(9,9,CV_64FC1);
        cvmInvert(Hessian,HessianInv);
        cvReleaseMat(&Hessian);

        //**************************** Compute steps before line search *************

		//Holds all the slopes as a vector
		double delJ_final[9];

		for( int j=0 ; j<9 ; j++)
		{
			delJ_final[j] = 0.0;

			for( int k=0 ; k<9 ; k++)
			    delJ_final[j] += CV_MAT_ELEM(*HessianInv,double,j,k) * delJ_vector[k];			
		}


        //******************* Perform back tracking line search***************

		
		double alpha =  constants::INITIAL_ALPHA;	 //Learning rate
		double delJ_New[9];				//Vector for the slopes
		double J_i = 0.0;				//New objective function value at each iteration
		
		//Back track till there is an improvement in the objective function
		do
		{	
            
			//compute new learning rate
			alpha /= constants::BACK_TRACKING_FACTOR ;

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
            J_i = J(IDash,IDashColor,HDash,currentAnnealingLevel,cContent,currentTracks);

			//Free memory
			cvReleaseMat(&HDash);
			
		}
		while(J_i > J_i_minus_1);


        //Update H after back tracking line search
		for( int y=0 ; y<3 ; y++)
		    for( int x=0 ; x<3 ; x++)
			    CV_MAT_ELEM(*H,double,y,x) = CV_MAT_ELEM(*H,double,y,x) - delJ_New[y * 3 + x];
			
        //Simple failsafe mechanism
		if( J_i_minus_1 - J_i < 0 )
		{
			cout<<"\n\nOptimization has failed "<<J_i_minus_1<<" "<<J_i;
            exit(-1);			
		}



        //Anneal the objective function and continue optimization until annealing ends
        if( fabs( J_i_minus_1 - J_i ) <= constants::TOLERANCE || (constrainIterations && i % constants::ITERATIONS_FOR_PF == 1)  )
		{	

            if( constants::VERBOSE )
			    printf("\n ******************************************** STRIKE ************************** \n");

            //Update J_reference with the resultant value of this smoothing level
            J_Reference = J_Reference + J_i * constants::SMOOTHING_WEIGHTS[currentAnnealingLevel];
            
            currentAnnealingLevel++;

            if( currentAnnealingLevel < constants::TOTAL_SMOOTHING_ITERATIONS )
			{	
				cvReleaseImage(&IDash);

				//Make copies for blurring etc naming consistent with math notation
                IDash = cvCloneImage(cVideo._frames[volumeIndex][ nextFrameIndex ]._grayScaleImage);
				
				//Anneal the objective function
                for( unsigned j=0 ; j<constants::SMOOTHING_ITERATIONS[currentAnnealingLevel] ; j++)
                    cvSmooth(IDash,IDash,CV_GAUSSIAN,constants::SMOOTHING_MASK_SIZE,constants::SMOOTHING_MASK_SIZE);

       
                cContent.populateNCCStats(currentAnnealingLevel);

                J_i_minus_1 = J(IDash,IDashColor,H,currentAnnealingLevel,cContent,currentTracks); //Compute initial value of the objective function
       
			}
			else
			    break;
						
		}//if tolerance
        else
        {
                  //Set value of current iteration, as value of previous iteration
                    J_i_minus_1 = J_i;

        }


  
    }//Optimization loop i


    //Free memory
    cvReleaseImage(&IDash);
    cvReleaseImage(&IDashColor);

    return H;
}


//Does required checks for the STILL_CAMERA tracking type
CvMat* contentTracker::estimateIdentityTransform(video &cVideo , unsigned volumeIndex, CvMat *A, content &cContent , contentTracks &currentTracks, double &J_Reference , int constrainIterations)
{
    CvMat *H = cvCloneMat(A);
    int nextFrameIndex = constants::VOLUME_MARGIN * 2 + 1;
    
    //If the content model has not been populated
    if( cContent._pixelLocations.empty() )
            populateModel(cVideo._frames[volumeIndex],cContent, currentTracks , 0); //Populate region model

    //Make local copies for opitmization
    IplImage *IDashColor = cvCloneImage( cVideo._frames[volumeIndex][ nextFrameIndex ]._colorImage );

    for( unsigned i=0 ; i<constants::SMOOTHING_ITERATIONS[ constants::OCC_SMOOTHING_INDEX ] ; i++)
        cvSmooth(IDashColor,IDashColor,CV_GAUSSIAN,constants::SMOOTHING_MASK_SIZE,constants::SMOOTHING_MASK_SIZE);

    //Removes model pixels which may be occluded from the tracking process 
    estimateOccludedPixels( cContent,A,IDashColor);

    cvReleaseImage(&IDashColor);
   
    return H;
}


//Computes the value of the objective function
double contentTracker::J(IplImage *IDash , IplImage *IDashColor , CvMat *H , unsigned index , content &cContent , contentTracks &cTracks)
{
    //************** Compute J1 ******************
    double J1 = 1 - NCC(IDash,IDashColor,H,index,cContent);

    //************* Compute J2 ******************
    double J2 = 0.0;

    if( cTracks._H != NULL )
    {
	    //Compute Frobenius norm
	    for( int i=0 ; i<3 ; i++)
	        for( int j=0 ; j<3 ; j++)
		        J2 += pow( CV_MAT_ELEM(*H,double,i,j) - CV_MAT_ELEM(*cTracks._H,double,i,j) ,2 );
	    
    }

    return ( J1 + constants::OPTIMIZATION_REGULARIZATION * J2 );

}

//Computes the value of the differential objective function given all the variables and parameters
double contentTracker::delJ(IplImage *IDash, IplImage *IDashColor, CvMat *H, int y , int x ,unsigned index , content &cContent , contentTracks &cTracks)
{

	//********************************* Compute Del J1 ********************************
	
	//Compute the matrix HDash for numerical differentiation of J1
	CvMat *HDash = cvCloneMat(H);
	CV_MAT_ELEM(*HDash,double,y,x) = CV_MAT_ELEM(*H,double,y,x) + constants::DELTA_HJ;
	
	
	//Holds differential of sub objective function J1
	double delJ1 = NCC(IDash,IDashColor,H,index,cContent) - NCC(IDash,IDashColor,HDash,index,cContent);			
		
	delJ1 /= constants::DELTA_HJ;

	cvReleaseMat(&HDash);
	

	//********************************  Compute Del J2 *********************************

	//Holds differential of sub objective function J2
	double delJ2 = 0.0;

    if( cTracks._H != NULL )
        delJ2 = 2 * ( CV_MAT_ELEM(*H,double,y,x) - CV_MAT_ELEM(*cTracks._H,double,y,x) );
    

	return (delJ1 + constants::OPTIMIZATION_REGULARIZATION * delJ2 ) ;
}

//Computes the normalized cross correlation of the given gray image with the model
double contentTracker::NCC(IplImage *IDash, IplImage* IDashColor, CvMat *H, unsigned index,content &cContent)
{
    double IDashBar = 0.0;      //Mean of the region in the new frame
    double subtract = 0.0;      //Amount subtracted from the model variance as a result of newly found occluded pixels
    unsigned totalPixels = (unsigned)cContent._pixelLocations.size();

    //Cache for the new frame, computed during mean computation recycled during variance & deviation computation
    vector<double> subPixelValues(totalPixels);
    vector<double> subPixelProjectionsX(totalPixels);
    vector<double> subPixelProjectionsY(totalPixels);

    //************ Compute means of the region in the new frame ****************
    for( unsigned i=0 ; i<totalPixels ; i++)
    {
        if( cContent._isOccluded[i] )
            continue;
        
        //Estimate location of the model pixel in the new frame based upon the given transform
        double Hx = cContent._pixelLocations[i].x , Hy = cContent._pixelLocations[i].y;
        this->projectCoordinate(H , Hx , Hy);
        
        //If pixel is out of bounds treat it like an occlusion
        if(	Hx - constants::MARGIN < 0 || Hx + constants::MARGIN - IDash->width  > 0 
        || Hy - constants::MARGIN < 0 || Hy + constants::MARGIN - IDash->height  > 0 )
        {
            return -1.0;
            //cContent._isOccluded[i] = true;
            //cContent._unOccludedPixels--;
            //subtract += pow( cContent._NCCDeviations[i] , 2);
	        //continue;
        }

        subPixelProjectionsX[i] = Hx;
        subPixelProjectionsY[i] = Hy;

        double pixelValue = this->subPixel(IDash,Hx,Hy);
        
        IDashBar += pixelValue;
        subPixelValues[i] = pixelValue;

    }

    IDashBar /= cContent._unOccludedPixels;

    //************** Compute variance and deviations ***********
    double IDashVariance = 0.0 , denominator = 0.0 , numerator = 0.0;

    for( unsigned i=0 ; i<totalPixels ; i++)
    {
        if( cContent._isOccluded[i])
            continue;
        
        
        if( !cContent._NCCOcclusionMode || (cContent._NCCOcclusionMode && !cContent.isOutlier(i,this->subPixelRGB(IDashColor,subPixelProjectionsX[i],subPixelProjectionsY[i])) ) )
        {            
            double deviation = subPixelValues[i] - IDashBar;

            IDashVariance += pow( deviation ,2 );
            numerator += cContent._NCCDeviations[i] * deviation;            
        }
        else
        {
            subtract += pow( cContent._NCCDeviations[i] , 2);
        }


    }

    denominator = sqrt( (cContent._NCCVariance - subtract) * IDashVariance );

    double nccValue = 0.0;

    //Testing for boundary condition
    if( denominator != 0 )
    {
        nccValue = numerator / denominator;

        if( nccValue > 1 )
            nccValue = 1;
    }

    //Free memory
    subPixelValues.clear();
    subPixelProjectionsX.clear();
    subPixelProjectionsY.clear();

    return nccValue;

}


//Estimates which pixels may be occluded in the model based upon initial transform estimates
bool contentTracker::estimateOccludedPixels(content &cContent, CvMat *A , IplImage *IDashColor)
{
    if( constants::VERBOSE )
        cout<<"\n\n Estimating occluded pixels";
	
    cContent.cleanOcclusionModel();//Reset occlusion related variables
	int occludedPixels = 0; //Total occluded pixels out of the pixels being tracked
    unsigned totalPixels = (unsigned)cContent._pixelLocations.size(); //Total pixels in the model
    
     //Estimate whether each pixel is occluded or not
     for( unsigned i=0 ; i<totalPixels ; i++)
     {		           
            //Compute location of model pixel in current frame
	        double Hx = cContent._pixelLocations[i].x , Hy = cContent._pixelLocations[i].y;
            this->projectCoordinate(A,Hx,Hy);
    
           if( !(Hx - constants::MARGIN < 0 || Hx + constants::MARGIN - IDashColor->width  > 0 ||
                 Hy - constants::MARGIN < 0 || Hy + constants::MARGIN - IDashColor->height  > 0) )
           {
          
               CvScalar estimatedPixelValue = this->subPixelRGB(IDashColor,Hx,Hy);

                //Check if the currently estimated value is an outlier in the distribution of the pixel
               if ( cContent.isOutlier(i,estimatedPixelValue) )
                    {
                        cContent._isOccluded[i] = true;
                        occludedPixels++;  
                    }
                    else
                        cContent._isOccluded[i] = false;
           }
           else
           {
                cContent._isOccluded[i] = true;
                occludedPixels++;  
           }
                	
        }//Model pixels loop
		
				
    if( constants::VERBOSE )
        cout<<"\n\n Total occluded pixels "<<occludedPixels<<"\n";

    if( occludedPixels > 0 )
    {
        cContent._occlusionMode = 1;
    
        if( _currentParticle == constants::IDENTITY_PARTICLE )
        {
            if( (occludedPixels + 0.0) / totalPixels > constants::OCC_PERCENTAGE_FOR_IDENTITY_PARTICLE)
               cContent._NCCOcclusionMode = 1;
            
        }
        else if( (occludedPixels + 0.0) / totalPixels > constants::OCC_PERCENTAGE_FOR_PARTICLE)
            cContent._NCCOcclusionMode = 1;
        
    }

    //Compute total unoccluded pixels
    cContent._unOccludedPixels = totalPixels - occludedPixels;

    if( (occludedPixels + 0.0) / totalPixels > constants::TRACTABLE_OCCLUSION_THRESHOLD )
        return true;

    return false;
}



//Populates the pixel model for the given content
void contentTracker::populateModel(vector<frame> &cFrames, content &cContent , contentTracks &currentTracks , int visualize)
{
    int centreFrame = constants::VOLUME_MARGIN * 2;

    //Clone images for processing to populate the olor and grayscale models
    IplImage *colorImage = cvCloneImage( cFrames[centreFrame]._colorImage );
    IplImage *grayImage  = cvCloneImage( cFrames[centreFrame]._grayScaleImage );
    IplImage *tempBlur = NULL;  //Holds the blurred copies of the images

    IplImage *visualizeImage = NULL; //Image object for visualizing the location of the model
  
    if( visualize )
       visualizeImage = cvCloneImage(colorImage);

      
    cContent.cleanRegionModel();    //Empty all data structures associated with the region model
    cContent.populateModelLocations(currentTracks, visualizeImage);


    //************************ Populate the grayscale model ****************

    for( unsigned i=0 ; i<constants::TOTAL_SMOOTHING_ITERATIONS ; i++)
    {
        tempBlur = cvCloneImage(grayImage);   //Create temp copy for blurring

        for( unsigned j=0 ; j<constants::SMOOTHING_ITERATIONS[i] ; j++)
                cvSmooth(tempBlur,tempBlur,CV_GAUSSIAN,constants::SMOOTHING_MASK_SIZE,constants::SMOOTHING_MASK_SIZE);  

        cContent.populateGrayModelValues(tempBlur,i);

        cvReleaseImage(&tempBlur);
    }

    //************************ Populate the color model ****************

    tempBlur = cvCloneImage(colorImage);

    //Smooth the image as per the required number of iterations
    for( unsigned j=0 ; j<constants::SMOOTHING_ITERATIONS[ constants::OCC_SMOOTHING_INDEX ] ; j++)
        cvSmooth(tempBlur,tempBlur,CV_GAUSSIAN,constants::SMOOTHING_MASK_SIZE,constants::SMOOTHING_MASK_SIZE);  

    cContent.populateColorModelValues(tempBlur);

     cvReleaseImage(&tempBlur);

     if( visualize )
     {
         utilities::display(visualizeImage);
         cvReleaseImage(&visualizeImage);
     }

    //Free memory
   
    cvReleaseImage(&colorImage);
    cvReleaseImage(&grayImage);     
}
