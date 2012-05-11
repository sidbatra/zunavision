/************************************************************************
Demo software: Invariant keypoint matching.
Author: David Lowe

defs.h:
This file contains the headers for a sample program to read images and
  keypoints, then perform simple keypoint matching.
*************************************************************************/

#pragma once

/* From the standard C libaray: */
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>

/*------------------------------ Macros  ---------------------------------*/

#define ABS(x)    (((x) > 0) ? (x) : (-(x)))
#define LOWE_MAX(x,y)  (((x) > (y)) ? (x) : (y))
#define LOWE_MIN(x,y)  (((x) < (y)) ? (x) : (y))


/*---------------------------- Structures --------------------------------*/

/* Data structure for a float image.
*/
typedef struct ImageSt {
  int rows, cols;          /* Dimensions of image. */
  float **pixels;          /* 2D array of image pixels. */
  struct ImageSt *next;    /* Pointer to next image in sequence. */
} *Image;


/* Data structure for a keypoint.  Lists of keypoints are linked
   by the "next" field.
*/
typedef struct KeypointSt
{
  float row, col,var;             /* Subpixel location of keypoint. */
  float scale, ori;           /* Scale and orientation (range [-PI,PI]) */
  unsigned char *descrip;     /* Vector of descriptor values */
  float *descVar;
  struct KeypointSt *next;    /* Pointer to next keypoint in list. */
  float distance;
  int sharedIndex;
} *Keypoint;



/*-------------------------- Function prototypes -------------------------*/
/* These are prototypes for the external functions that are shared
   between files.
*/

/* From util.c */
void FatalError(char *fmt, ...);
Image CreateImage(int rows, int cols);
Image ReadPGMFile(char *filename);
Image ReadPGM(FILE *fp);
void WritePGM(FILE *fp, Image image);
void DrawLine(Image image, int r1, int c1, int r2, int c2);
Keypoint ReadKeyFile(char *filename);
Keypoint ReadKeys(FILE *fp);
Keypoint FindMatches(Keypoint keys1, Keypoint keys2, double threshold = 6.0);
Keypoint CheckForMatch(Keypoint key, Keypoint klist, double threshold = 6.0);
int DistSquared(Keypoint k1, Keypoint k2);
Image CombineImagesVertically(Image im1, Image im2);
Keypoint GetKeypoints( Image image );
