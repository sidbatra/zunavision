/*****************************************************************************
** MOVIE PROJECT
** Copyright (c) 2008 
**
** FILENAME:    contentData.h
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Encapsulates the properties associated with a region to be tracked
*****************************************************************************/


#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <limits>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <iomanip>
#include <map>

#include <sys/types.h>
#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
#include "dirent.h"
#else
#include <dirent.h>
#endif

#include "wx/wx.h"
#include "wx/utils.h"
#include "wx/wxprec.h"
#include "wx/cmdline.h"
#include "wx/aboutdlg.h"
#include "wx/glcanvas.h"
#include "wx/msgdlg.h"

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "svlPoint2d.h"
#include "utilities.h"

using namespace std;

class contentData
{

public:
        vector<wxPoint> points;
        int contentType;
        int renderType;
        int occType;
        int trackType;
        string contentName;
        string contentExtension;  
        int startingFrame;
        int endingFrame;
        int reverseEndingFrame;

public:
    contentData();
    static contentData getContentDataForFrame(int frameIndex , vector<contentData> data,int &isFound);
    static void saveContentData(vector<contentData> &data,string filename,string extension);
    static void loadContentData(vector<contentData> &data,string filename,string &extension);
    static int checkClosestContent(vector<contentData> data , int currentFrame);
    static int checkClosestContentForward(vector<contentData> data , int currentFrame);
    static int isContentAtIndex(vector<contentData> data , int currentFrame);


};
