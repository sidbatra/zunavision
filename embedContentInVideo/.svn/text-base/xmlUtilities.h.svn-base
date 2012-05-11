/*****************************************************************************
** ZunaVision
** Copyright (c) 2008 
**
** FILENAME:    xmlUtilities.h
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Various utility functions needed to interface to reading and writing data structures to XML tags

*****************************************************************************/
#pragma once

#include "base.h"

using namespace std;


class xmlUtilities
{
	public:
        
        static void writeVolume(ofstream &out , vector<CvPoint> &volume , unsigned index); //Writes the volume as an XML tag
        static vector<CvPoint> readVolume(XMLNode &volumeNode); //Reads the volume from an XML node
        
        static void writePoly(ofstream &out , vector<svlPoint2d> &polygon); //Writes the polygon as an XML tag
	static void writeMatrix(ofstream &out , CvMat *matrix); //Writes the given matrix as an xml tag
        static vector<svlPoint2d> readPoly(XMLNode polyNode); //Reads the polygon from an XML node
       
};
