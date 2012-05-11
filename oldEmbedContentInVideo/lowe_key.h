/************************************************************************
  Copyright (c) 2003. David G. Lowe, University of British Columbia.
  This software is being made freely available for research purposes
  only (see file LICENSE.txt for conditions).  This notice must be
  retained on all copies.
*************************************************************************/

#ifndef LOWE_KEY_H
#define LOWE_KEY_H

/* From the standard C libaray: */
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>

/*------------------------- Macros and constants  -------------------------*/

/* Following defines TRUE and FALSE if not previously defined. */
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

/* Value of PI, rounded up, so orientations are always in range [0,PI]. */
#define PI 3.1415927f

#define ABS(x)    (((x) > 0) ? (x) : (-(x)))
#define LOWE_MAX(x,y)  (((x) > (y)) ? (x) : (y))
#define LOWE_MIN(x,y)  (((x) < (y)) ? (x) : (y))

/* Given the name of a structure, NEW allocates space for it in the
   given pool (see util.c) and returns a pointer to the structure.
*/
#define NEW(s,pool) ((struct s *) MallocPool(sizeof(struct s),pool))

/* Assign a unique number to each pool of storage needed for this application. 
*/
#define PERM_POOL  0     /* Permanent storage that is never released. */
#define IMAGE_POOL 1     /* Data used only for the current image. */
#define KEY_POOL   2     /* Data for set of keypoints. */

/* These constants specify the size of the index vectors that provide
   a descriptor for each keypoint.  The region around the keypoint is
   sampled at OriSize orientations with IndexSize by IndexSize bins.
   VecLength is the length of the resulting index vector.
*/
#define OriSize 8    /* was 8 */
#define IndexSize 4  /* was 4 */
#define VecLength (IndexSize * IndexSize * OriSize)


/*---------------------------- Structures -------------------------------*/

/* Data structure for a float image.
*/
typedef struct ImageSt {
    int rows, cols;          /* Dimensions of image. */
    float **pixels;          /* 2D array of image pixels. */
} *Image;


/* This structure describes a keypoint that has been found in an image.
*/
typedef struct KeypointSt
{
    float row, col , var;      /* Row, column location relative to input image.  */
    float scale;         /* Scale (in sigma units for smaller DOG filter). */
    float ori;           /* Orientation in radians (-PI to PI). */
    unsigned char *ivec; /* Vector of gradient samples for indexing.*/
	float *descVar;
    struct KeypointSt *next;   /* Links all keypoints for an image. */
} *Keypoint;


/*------------------------------- Externals -----------------------------*/

extern const int MagFactor;
extern const float GaussTruncate;


/*-------------------------- Function prototypes -------------------------*/
/* The are prototypes for the external functions that are shared
   between files.
*/

/* Only interface needed to key.c. */
Keypoint GetKeypoints(Image image);


/* Following are from util.c */
void *MallocPool(int size, int pool);
void FreeStoragePool(int pool);
float **AllocMatrix(int rows, int cols, int pool);
Image CreateImage(int rows, int cols, int pool);
Image LoweCopyImage(Image image, int pool);
Image DoubleSize(Image image);
Image HalfImageSize(Image image);
void SubtractImage(Image im1, Image im2);
void GradOriImages(Image im, Image grad, Image ori);
void GaussianBlur(Image image, float sigma);
void SolveLeastSquares(float *solution, int rows, int cols, float **jacobian,
		       float *errvec, float **sqarray);
void SolveLinearSystem(float *solution, float **sq, int size);
float DotProd(float *v1, float *v2, int len);


#endif
