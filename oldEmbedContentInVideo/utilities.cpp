/*****************************************************************************
** MOVIE PROJECT
** Copyright (c) 2008 
**
** FILENAME:    utilities.cpp
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Implementations of the static utilities in utilities.h
*****************************************************************************/

#include "utilities.h"


using namespace std;

//Computes the square distance between the two points
double utilities::squareDistance( svlPoint2d a , svlPoint2d b)
{
    return pow(a.x - b.x ,2 ) * pow(a.y - b.y ,2);
}

//Resize image in place based on new height width
void utilities::resizeInPlace(IplImage **image, int height, int width)
{
    IplImage *tmpImage = cvCreateImage(cvSize(width, height),(*image)->depth, (*image)->nChannels);
    cvResize(*image, tmpImage);
    cvReleaseImage(image);
    *image = tmpImage;
}

//Resize image in place based on scale factor
void utilities::resizeInPlace(IplImage **image, double scale)
{
    int height = (int)(scale * (*image)->height);
    int width = (int)(scale * (*image)->width);
    IplImage *tmpImage = cvCreateImage(cvSize(width, height),(*image)->depth, (*image)->nChannels);
    cvResize(*image, tmpImage);
   
    *image = cvCloneImage(tmpImage);
    cvReleaseImage(&tmpImage);
}

//loads all the filenames with the given extension from the given folder into the given vector
void utilities::loadAllFilenames(string folderName , string extension , vector<string> &frameNames , bool isReverse )
{    
	//Open link to image directory
	DIR *dir = opendir(folderName.c_str());
    if (dir == NULL) 
	{
		cerr << "ERROR: could not open image folder " << folderName.c_str() << endl;
		exit(-1);
    }	
	struct dirent *e = readdir(dir);//Init object to read image names
    
	//Iterate over each image in the folder and store its filename
    while (e != NULL) 
	{
		//Load next images with needed extension
		if (strstr(e->d_name, extension.c_str() ) != NULL) 
		    frameNames.push_back(e->d_name);
          
		//Read next file
		e = readdir(dir);
	
	}//While images are being read

    //Reverse the frame names if needed
    if( isReverse )
        reverse(frameNames.begin() , frameNames.end());    
}


//Computes the edge map for the given image
IplImage* utilities::edgeMap(IplImage *I)
{    

	CvMat *kernel = cvCreateMat(3,3,CV_32F);
	
    double ker[9] = {1 ,0 ,-1, 2 ,0 ,-2, 1 ,0 ,-1}; 

	for( int i=0 ; i<3  ; i++)
		for( int j=0 ; j<3 ; j++)
			cvmSet(kernel,i,j,ker[i*3+j]);
    
    //Create IplImage object to hold the final result
    IplImage *tempI   = cvCreateImage(cvSize(I->width+2,I->height+2),IPL_DEPTH_32F,I->nChannels);
    IplImage *edges   = cvCreateImage(cvSize(I->width+2,I->height+2),IPL_DEPTH_32F,I->nChannels);
    IplImage *final   = cvCreateImage(cvSize(I->width,I->height),IPL_DEPTH_32F,I->nChannels);


    for( int y=0 ; y<tempI->height ; y++)
	{  
		for( int x=0; x<tempI->width ; x++)
		{
            if( y== 0 || y == tempI->height || x == 0 || x == tempI->width -1)
                CV_IMAGE_ELEM(tempI,float,y,x) = 0.0;
            else
                CV_IMAGE_ELEM(tempI,float,y,x) = (float)CV_IMAGE_ELEM(I,unsigned char,y,x);
        }
    }
	
	cvFilter2D(tempI,edges,kernel,cvPoint(0,0));


    for( int y=0 ; y<edges->height-2 ; y++)
	{
        float alpha = 0;
		for( int x=0; x<edges->width-2 ; x++)
		{
           // if( y == 199)
           //     cout<<" "<<CV_IMAGE_ELEM(edges,float,y,x);

            CV_IMAGE_ELEM(final,float,y,x) = alpha;
            alpha += CV_IMAGE_ELEM(edges,float,y,x);            
        }
    }

 //   for( int y=0 ; y<edges->height ; y++)
	//{
	//	for( int x=0; x<edges->width ; x++)
	//	{
 //           if( CV_IMAGE_ELEM(edges,unsigned char ,y,x) < threshold || 
 //               x - EDGE_FILTER < 0 || x + EDGE_FILTER - edges->width > 0 ||
 //               y - EDGE_FILTER < 0 || y + EDGE_FILTER - edges->height > 0)
 //               CV_IMAGE_ELEM(edges,unsigned char ,y,x) = 0; 
 //           else
 //               CV_IMAGE_ELEM(edges,unsigned char ,y,x) = 255; 
 //       }
 //   }


    //Free memory
    cvReleaseMat(&kernel);
    cvReleaseImage(&edges);
    cvReleaseImage(&tempI);
   // delete []ker;

	return final;
}



void utilities::display(IplImage *image , string text )
{
	cvNamedWindow(text.c_str(), 1);
	cvShowImage(text.c_str() , image);
	cvWaitKey(-1);
	cvDestroyAllWindows();
}

string utilities::toString(int v)
{
	
    std::stringstream s;
    s << v;
    return s.str();
}

//Rounds off a floating point to an integer
int utilities::roundOff(double d)
{
	//Truncated value
	int result = (int)d;

	//Fractional value is floating point minus truncated value
	double fraction = d - (int)d;

	//If fraction is >= 0.5 round it off
	if( fraction >= 0.5 )
		result = (int)(d + 1);

	return result;
}
	


