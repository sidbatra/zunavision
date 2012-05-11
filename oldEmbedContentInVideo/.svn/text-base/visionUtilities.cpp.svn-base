/*****************************************************************************
** MOVIE PROJECT
** Copyright (c) 2008 
**
** FILENAME:    visionUtilities.cpp
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Impementation functions in visionUtilities.h

*****************************************************************************/

#include "visionUtilities.h"



//Draws an arrow at the given locations on the given image
void visionUtilities::drawArrow(IplImage *image, vector<svlPoint2d> points)
{    

    double angle = atan2( (double) points[0].y - points[1].y, (double) points[0].x - points[1].x );
	double hypotenuse = sqrt( pow(points[0].y - points[1].y,2) + pow(points[0].x - points[1].x ,2));

	points[1].x = (int) (points[0].x - hypotenuse * cos(angle));
	points[1].y = (int) (points[0].y - hypotenuse * sin(angle));

    cvLine( image,cvPoint((int)points[0].x,(int)points[0].y),cvPoint((int)points[1].x,(int)points[1].y), constants::ARROW_COLOR, constants::ARROW_THICKNESS, CV_AA, 0 );
	
    points[1].x = (int) (points[0].x + constants::ARROW_LENGTH / constants::RATIO_OF_ARROW_AHEADS * cos(angle + constants::PI / constants::ANGLE_OF_ARROW_AHEADS));
	points[1].y = (int) (points[0].y + constants::ARROW_LENGTH / constants::RATIO_OF_ARROW_AHEADS * sin(angle + constants::PI / constants::ANGLE_OF_ARROW_AHEADS));
    cvLine( image, cvPoint((int)points[0].x,(int)points[0].y),cvPoint((int)points[1].x,(int)points[1].y), constants::ARROW_COLOR, constants::ARROW_THICKNESS, CV_AA, 0 );
    
	points[1].x = (int) (points[0].x + constants::ARROW_LENGTH / constants::RATIO_OF_ARROW_AHEADS  * cos(angle - constants::PI / constants::ANGLE_OF_ARROW_AHEADS));
	points[1].y = (int) (points[0].y + constants::ARROW_LENGTH / constants::RATIO_OF_ARROW_AHEADS  * sin(angle - constants::PI / constants::ANGLE_OF_ARROW_AHEADS));
    cvLine( image, cvPoint((int)points[0].x,(int)points[0].y), cvPoint((int)points[1].x,(int)points[1].y), constants::ARROW_COLOR, constants::ARROW_THICKNESS, CV_AA, 0 );
}

//Computes ideal point for drawing arrow next to a rendered region in the video frame
vector<svlPoint2d> visionUtilities::findPointsForArrow( vector<svlPoint2d> polygon , IplImage *image , int direction)
{
     
    vector<svlPoint2d> finalPoints(2);

    //Left top corner of polygon to left top corner of image

    if( direction == 0 )
    {
        finalPoints[0] = svlPoint2d( polygon[0].x - constants::ARROW_GAP_FROM_POLYGON , polygon[0].y - constants::ARROW_GAP_FROM_POLYGON); 
        finalPoints[1] = svlPoint2d( finalPoints[0].x - constants::ARROW_LENGTH , finalPoints[0].y - constants::ARROW_LENGTH); 
    }
    else if(direction == 1)
    {
        finalPoints[0] = svlPoint2d( polygon[1].x + constants::ARROW_GAP_FROM_POLYGON , polygon[1].y - constants::ARROW_GAP_FROM_POLYGON); 
        finalPoints[1] = svlPoint2d( finalPoints[0].x + constants::ARROW_LENGTH , finalPoints[0].y - constants::ARROW_LENGTH); 
    }
    else if(direction == 2)
    {
        finalPoints[0] = svlPoint2d( polygon[2].x + constants::ARROW_GAP_FROM_POLYGON , polygon[2].y + constants::ARROW_GAP_FROM_POLYGON); 
        finalPoints[1] = svlPoint2d( finalPoints[0].x + constants::ARROW_LENGTH , finalPoints[0].y + constants::ARROW_LENGTH); 
    }
    else if(direction == 3)
    {
        finalPoints[0] = svlPoint2d( polygon[3].x - constants::ARROW_GAP_FROM_POLYGON , polygon[3].y + constants::ARROW_GAP_FROM_POLYGON); 
        finalPoints[1] = svlPoint2d( finalPoints[0].x - constants::ARROW_LENGTH , finalPoints[0].y + constants::ARROW_LENGTH); 
    }
        
    return finalPoints;    
}


//Computes ideal point for drawing arrow next to a rendered region in the video frame
int visionUtilities::findDirectionForArrow( vector<svlPoint2d> polygon , IplImage *image )
{     
    int direction = 0;

    //Left top corner of polygon to left top corner of image
    double distance = utilities::squareDistance( polygon[0] , svlPoint2d(0,0) ) ;
    
    
    //Right top corner of polygon to right top corner of image
    double temp = utilities::squareDistance( polygon[1] , svlPoint2d(image->width,0) ) ;
    
    //If this corner is further away than the previous one
    if( temp > distance )
    {      
        distance = temp;      
        direction = 1;
    }

    //Right bottom corner of polygon to right bottom corner of image
    temp = utilities::squareDistance( polygon[2] , svlPoint2d(image->width,image->height) ) ;
    
    //If this corner is further away than the previous one
    if( temp > distance )
    {
        distance = temp;
        direction = 2;

    }

    //Left bottom corner of polygon to left bottom corner of image
    temp = utilities::squareDistance( polygon[3] , svlPoint2d(0,image->height) ) ;
    
    //If this corner is further away than the previous one
    if( temp > distance )
    {
        distance = temp;
        direction = 3; 
    }

    return direction;    
}


//Fast method of retriving the values of a color image
CvScalar visionUtilities::getRGB( IplImage *image , int *x , int *y )
{
    uchar* temp_ptr = &((uchar*)(image->imageData + image->widthStep* (*y) ))[(*x)*3];
    return cvScalar(temp_ptr[0],temp_ptr[1],temp_ptr[2]);  
}


//Computes the connected components in an image and removes the ones below a certain size 
void visionUtilities::removeSmallComponents( IplImage *occlusionModel , double areaThershold, bool invert , bool keepEdgeComponents)
{
            
        //Create structures need to store connected components in the image
        IplImage* colorMap = cvCreateImage( cvGetSize(occlusionModel), IPL_DEPTH_8U, 3 );
        CvMemStorage* storage = cvCreateMemStorage(0);
        CvSeq* contour = 0;
        int maskArea = occlusionModel->width * occlusionModel->height;


        
        //If the image is to be inverted
        if( invert )
        {
            //Invert every value in the binary image
            for( int y=0 ; y<occlusionModel->height ; y++)
                for( int x=0 ; x<occlusionModel->width ; x++)
                    CV_IMAGE_ELEM(occlusionModel,unsigned char,y,x) = 255 - CV_IMAGE_ELEM(occlusionModel,unsigned char,y,x);
        }
        
        //Compute the contours
        IplImage *temp = cvCloneImage(occlusionModel);
        cvFindContours( temp, storage, &contour, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
        cvReleaseImage(&temp);
        
        cvZero( colorMap );
        
        for( ; contour != 0; contour = contour->h_next )
        {
            //Generate random color for contour
            CvScalar color = CV_RGB( rand() % 255, rand() % 255, rand() % 255); 
            
            //Draw contour with the randomly generated color
            cvDrawContours( colorMap, contour, color, color, -1, CV_FILLED, 8 );
            
            CvRect r = cvContourBoundingRect(contour);

            if( fabs(cvContourArea(contour)) / (maskArea+0.0) < areaThershold &&
                ( !keepEdgeComponents || (  r.x > 1 && r.y > 1 && r.x+r.width < occlusionModel->width-1 
                                            && r.y+r.height < occlusionModel->height-1 )) )
                
            {
                //Wipe out the component from the mask
                for( int y=r.y ; y<r.y+r.height ; y++)  
                    for( int x=r.x ; x<r.x+r.width; x++)
                    {
                        CvScalar c = visionUtilities::getRGB(colorMap,&(x),&(y));

                        if( c.val[0] == color.val[0] && c.val[1] == color.val[1] && c.val[2] == color.val[2] )
                            CV_IMAGE_ELEM(occlusionModel,unsigned char,y,x) = 0;                        
                    }
            }
            
        }//for contour

        //utilities::display(colorMap , "Components");

        //If the image is to be inverted
        if( invert )
        {
            //Invert every value in the binary image
            for( int y=0 ; y<occlusionModel->height ; y++)
                for( int x=0 ; x<occlusionModel->width ; x++)
                    CV_IMAGE_ELEM(occlusionModel,unsigned char,y,x) = 255 - CV_IMAGE_ELEM(occlusionModel,unsigned char,y,x);
        }

        //Free memory
        cvReleaseImage(&colorMap);

        if( storage != NULL )
            cvReleaseMemStorage(&storage);
        
}


//Computes Mahalnobis distance
double visionUtilities::computeMahalnobis(CvScalar pixelMean , CvScalar pixelVariance , CvScalar pixelValue)
{
    double result = 0.0;
    
    for( int k=0 ; k<3 ; k++)
        result += pow( pixelValue.val[k] - pixelMean.val[k] , 2 )
        / ( pixelVariance.val[k] + constants::VARIANCE_REGULARIZATION[k] );
    
    return result;
}



//Checks whether the given polygon lies within the image or not
int visionUtilities::checkOutofBounds(IplImage *image , vector<svlPoint2d> polygon , int margin)
{
    int isOutside = 1;

    for( unsigned i=0 ; i<polygon.size() ; i++)
    {        
        if( !(polygon[i].x - margin < 0 || polygon[i].x + margin - image->width  > 0 ||
                    polygon[i].y - margin < 0 || polygon[i].y + margin - image->height  > 0) )
        {
            isOutside = 0;
            break;
        }
    }

    return isOutside;
}

//Checks whether the given polygon lies within the image or not
int visionUtilities::checkOutofBounds(IplImage *image , vector<svlPoint2d> polygon)
{
    return visionUtilities::checkOutofBounds(image,polygon,constants::MARGIN);
}


//Checks whether the project of the given polygon is out of bounds
int visionUtilities::checkOutofBounds(IplImage *image , vector<svlPoint2d> polygon , CvMat *H)
{
    int isOutside = 1;

    for( int i=0 ; i<(int)polygon.size() ; i++)
    {
        svlPoint2d p = polygon[i];
        visionUtilities::projectCoordinate(H,&(p.x),&(p.y));

        if( !(p.x - constants::MARGIN < 0 || p.x + constants::MARGIN - image->width  > 0 ||
                    p.y - constants::MARGIN < 0 || p.y + constants::MARGIN - image->height  > 0) )
        {
            isOutside = 0;
            break;
        }
    }

    return isOutside;
}

//Performs the opeation H * X .. where H is the homography matrix and X = [ x y z ] the coordinate vector
//where z is assumed to be 1 and the nprojects the new x y coordinates via xNew = xNew / zNew &  yNew = yNew / zNew
void visionUtilities::projectCoordinate(CvMat *homography , double *x , double *y)
{
	double xOld = *x;
	double yOld = *y;
	double zOld = 1.0;
	double z = 0.0;
	

	*x = xOld * CV_MAT_ELEM(*homography,double,0,0) + yOld * CV_MAT_ELEM(*homography,double,0,1) + zOld * CV_MAT_ELEM(*homography,double,0,2);
	*y = xOld * CV_MAT_ELEM(*homography,double,1,0) + yOld * CV_MAT_ELEM(*homography,double,1,1) + zOld * CV_MAT_ELEM(*homography,double,1,2);
	 z = xOld * CV_MAT_ELEM(*homography,double,2,0) + yOld * CV_MAT_ELEM(*homography,double,2,1) + zOld * CV_MAT_ELEM(*homography,double,2,2);

	(*x) = (*x) / (z);
	(*y) = (*y) / (z);
	
}


//Estimates based on histograms whether the current 
int visionUtilities::estimateCutScene(IplImage *imageOne, IplImage *imageTwo)
{
    //THINK ABOUT USING OPENCV HISTOGRAMS TO SPEED UP / IMRPOVE THIS STEP
     //Clone the images for blurring
    IplImage *one = cvCloneImage(imageOne);    
    IplImage *two = cvCloneImage(imageTwo);
    IplImage *planeOne[] = {one};
    IplImage *planeTwo[] = {two};
    int size[] = {256};

    cvSmooth(one,one,CV_GAUSSIAN,constants::SMOOTHING_MASK_SIZE,constants::SMOOTHING_MASK_SIZE);
    cvSmooth(two,two,CV_GAUSSIAN,constants::SMOOTHING_MASK_SIZE,constants::SMOOTHING_MASK_SIZE);

    //Init default return value
    int isCutScene = 0;

    CvHistogram *histOne = cvCreateHist(1,size,CV_HIST_ARRAY);
    CvHistogram *histTwo = cvCreateHist(1,size,CV_HIST_ARRAY);
    cvCalcHist(planeOne,histOne);
    cvCalcHist(planeTwo,histTwo);

    cvNormalizeHist(histOne,1);
    cvNormalizeHist(histTwo,1);
    double value = cvCompareHist(histOne,histTwo,CV_COMP_CHISQR );

    cout<<"\n diff - "<<value;

    
    if( value  > constants::CUT_SCENE_THRESHOLD )
        isCutScene = 1;

    //Free memory
    cvReleaseImage(&one);
    cvReleaseImage(&two);

    return isCutScene;
}



//Estimates whether the given scene makes a cut scene based upon
//image differencing
//int visionUtilities::estimateCutScene(IplImage *imageOne, IplImage *imageTwo)
//{
//     //Clone the images for blurring
//    IplImage *one = cvCloneImage(imageOne);    
//    IplImage *two = cvCloneImage(imageTwo);
//    
//
//    cvSmooth(one,one,CV_GAUSSIAN,constants::SMOOTHING_MASK_SIZE,constants::SMOOTHING_MASK_SIZE);
//    cvSmooth(two,two,CV_GAUSSIAN,constants::SMOOTHING_MASK_SIZE,constants::SMOOTHING_MASK_SIZE);
//
//    //Init default return value
//    int isCutScene = 0;
//
//    double sum = 0.0;
//
//    //Compute the square difference sum between the pixel value
//    for( int y=0 ; y<one->height ; y++)
//    {
//        for( int x=0 ; x<one->width ; x++)
//        {
//            int temp = CV_IMAGE_ELEM(one,unsigned char,y,x) - CV_IMAGE_ELEM(two,unsigned char,y,x);
//            sum += temp * temp;
//
//        }
//    }
//
//    sum /= one->height * one->width;
//
//    cout<<"\n SUM - "<<sum;
//
//    //Test for cut scene
//    if( sum > constants::CUT_SCENE_THRESHOLD )
//        isCutScene = 1;
//
//    //Free memory
//    cvReleaseImage(&one);
//    cvReleaseImage(&two);
//
//    return isCutScene;
//}

