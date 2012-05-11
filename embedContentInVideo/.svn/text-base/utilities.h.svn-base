/*****************************************************************************
** ZunaVision
** Copyright (c) 2008 
**
** FILENAME:    utilities.h
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** A set of static methods which are used throughout the code as utilities
*****************************************************************************/

#pragma once

#include "base.h"


using namespace std;

class utilities
{
	public:
	static std::string toString(int v);     //Converts the given integer to string
        static std::string padZeroes(string text , unsigned length );    //Pads zeroes as prefix to the given string

        static void populateMatrix(CvMat **A  , double *data , int width , int height);      //Populates the given array into the matrix
		
        static int roundOff(double d); //Rounds of floating point to integer
	static void display(IplImage* image,std::string text = "Default" );
        static void split(string& str, vector<string>& tokens, string& delimiters);
 
        
        static vector<string> loadFilenames(string folderName , string extension , bool appendFoldername = false, bool isReverse = false );

        static void resizeInPlace(IplImage **image, double scale);
        static void resizeInPlace(IplImage **image, int height, int width);
        static double squareDistance( svlPoint2d a , svlPoint2d b);

        static void createDirectory(string path);
        static void moveFile(string source , string destination);
        static void copyFile(string source , string destination);
        static bool doesFileExist(string path);
                
        
};

