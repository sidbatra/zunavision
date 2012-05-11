/*****************************************************************************
** ZunaVision
** Copyright (c) 2008 
**
** FILENAME:    visionUtilities.h
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** A set of static functions for various vision utilities to be used throughout the codebase

*****************************************************************************/
#pragma once

#include "base.h"

using namespace std;

class visionUtilities
{
	public:

        //Checks whether the given polygon lies within the image or not
        static int checkOutofBounds(IplImage *image , vector<svlPoint2d> polygon);
        static int checkOutofBounds(IplImage *image , vector<svlPoint2d> polygon , int margin);
        static int checkOutofBounds(IplImage *image , vector<svlPoint2d> polygon , CvMat *H);

        static svlPoint2d findCentreOfRegion( vector<svlPoint2d> region );   //Finds the centre of the given quadrilaterlal
        static void findEnclosingRectangle(vector<svlPoint2d> polygon,double &minX,double &minY ,double &maxX ,double &maxY); //Finds the possibly larger enclosing rectangle for the given polygon

        //Generates the poisson edge map for the given image
        static IplImage* poissonEdgeMap(IplImage *image);

	//Reads a transparent PNG in the form of a text file and populates an image and a mask from it
	static void readTransparentPNG(string filename , IplImage **content , IplImage **mask);
        
        //Computes Mahalnobis distance
        static double computeMahalnobis(CvScalar pixelMean , CvScalar pixelVariance , CvScalar pixelValue);
        
        //Performs the opeation H * X .. where H is the homography matrix and X = [ x y z ] the coordinate vector
        //where z is assumed to be 1 and the nprojects the new x y coordinates via xNew = xNew / zNew &  yNew = yNew / zNew
        static void projectCoordinate(CvMat *homography , double &x , double &y);
        
        //Scales the given polygon
        static void scalePolygon( vector<svlPoint2d> &polygon , double scale);


        //Computes a pseduo inverse of the correspondences to estimate the affine transformation matrix
        static CvMat* estimateHomography(vector<CvMat*> matches);

        //Compute the transformation matrix required to map pixels from the sourcePolygon to the destination polygon
        static CvMat* findMappingBetweenPolygons(vector<svlPoint2d> sourcePolygon , vector<svlPoint2d> targetPolygon , int targetOffsetX=0 , int targetOffsetY=0);

        //Checks if the given point lies within the given polygon or not
        static bool inPolygon(vector<svlPoint2d> &polygon, double &x , double &y);
        //Computes the shortest distance from the given point to one of polygon edges
        static double shortestDistanceToEdge(vector<svlPoint2d> &polygon, double &pX , double &pY);

        //Checks whether the given pixel value is an outlier based on the gaussian model of that pixel
        static bool isOutlier(CvScalar &pixelMean , CvScalar &pixelValue , CvScalar &pixelVariance);

        //Forms a polygon from the corners of the image
        static vector<svlPoint2d> formPolygonFromImage(IplImage *image);
        static vector<svlPoint2d> formPolygonFromRect(CvRect rect);//Forms a polygon from the given rectangle
        
        //Computes subpixel values in the image
        static double subPixel( IplImage *I , double &PiX , double &PiY);

        //Computes subpixel values in the image of type float
        static double subPixelFloat( IplImage *I , double &PiX , double &PiY);

        //Retrieves subpixel values in an RGB image
        static CvScalar subPixelRGB( IplImage *I , double &PiX , double &PiY);

        //Fast method of retriving the values of a color image
        static CvScalar getRGB( IplImage *image , int &x , int &y );

        //Fast method of setting the values of a color image
        //static void setRGB( IplImage *image , int &x , int &y , CvScalar newValue);
        
        //Computes the connected components in an image and removes the ones below a certain size 
        static void removeSmallComponents( IplImage *occlusionModel , double areaThershold, bool invert , bool keepEdgeComponents);

        static void computeImageMeansVariances(IplImage *image, vector<double> &means , vector<double> &variances);

        static vector<svlPoint2d> findPointsForArrow( vector<svlPoint2d> polygon , IplImage *image , int direction);
        static int findDirectionForArrow( vector<svlPoint2d> polygon , IplImage *image );
        static void drawArrow(IplImage *image, vector<svlPoint2d> points);

        //Tests if the two given frames are the same
        static int isRepeatFrame(IplImage *one , IplImage *two);
        static bool isBlankFrame( IplImage *frame );

	static IplImage *toGrayscale(IplImage *source);
	static IplImage *computeSpatialImage(IplImage *source);
	static int compareSpatialImages(IplImage *one , IplImage *two);


        static vector<float> computeHistogram(IplImage *source , bool freeSource = false);
        static double compareHistograms(CvHistogram *histOne , CvHistogram *histTwo);



};
