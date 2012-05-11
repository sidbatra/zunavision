/************************************************************************
Demo software: Invariant keypoint matching.
Author: David Lowe

match.c:
This file contains a sample program to read images and keypoints, then
   draw lines connecting matched keypoints.
*************************************************************************/

#include "lowe_defs.h"



/* -------------------- Local function prototypes ------------------------ */

Keypoint FindMatches(Keypoint keys1, Image im2, Keypoint keys2, double threshold);
Keypoint CheckForMatch(Keypoint key, Keypoint klist, double threshold);
int DistSquared(Keypoint k1, Keypoint k2);
Image CombineImagesVertically(Image im1, Image im2);
Keypoint GetKeypoints( Image image );
void *MallocPool(int size, int pool);


///* Given a pair of images and their keypoints, pick the first keypoint
//   from one image and find its closest match in the second set of
//   keypoints.  Then write the result to a file.
//*/
Keypoint FindMatches(Keypoint keys1, Keypoint keys2 , double threshold)
{
    Keypoint k, match , matches;

	matches = NULL;
	

    int count = 0;

	printf("\nMessage from external SIFT code \n\n");
	

    /* Create a new image that joins the two images vertically. */
    //result = CombineImagesVertically(im1, im2);



    /* Match the keys in list keys1 to their best matches in keys2.
    */
    for (k= keys1; k != NULL; k = k->next)
	{
	
      match = CheckForMatch(k, keys2,threshold);  
	
      /* Draw a line on the image from keys1 to match.  Note that we
	 must add row count of first image to row position in second so
	 that line ends at correct location in second image.
      */
      if (match != NULL) 
	  {
		count++;
		printf("Point on first image (R,C) - %d %d   %d %d - Point on second image (R,C)\n" , (int)k->row ,(int)k->col ,(int) match->row,(int)match->col);

		//Dirty hack to get data out from this function
		//Including openCV data structures or lasik structures was giving lots of issues
		//Keypoint temp = ((struct KeypointSt *) MallocPool(sizeof(struct KeypointSt),2));
		//NEW(KeypointSt, 2);//(Keypoint) malloc(sizeof(struct KeypointSt));
		Keypoint temp = (Keypoint) malloc(sizeof(struct KeypointSt));
		
		

		temp->row = k->row;
		temp->col = k->col;
		temp->scale = match->row;
		temp->ori = match->col;

		
		

		temp->next = matches;

		matches = temp;

      }
    }
	
    printf("Found %d matches.\n", count);

	printf("\nEnd of Message from external SIFT code \n\n");

	return matches;
}


/* This searches through the keypoints in klist for the two closest
   matches to key.  If the closest is less than 0.6 times distance to
   second closest, then return the closest match.  Otherwise, return
   NULL.
*/
Keypoint CheckForMatch(Keypoint key, Keypoint klist , double threshold)
{
    int dsq, distsq1 = 100000000, distsq2 = 100000000;
    Keypoint k, minkey = NULL;

    /* Find the two closest matches, and put their squared distances in
       distsq1 and distsq2.
    */
    for (k = klist; k != NULL; k = k->next)
	{

      dsq = DistSquared(key, k);

      if (dsq < distsq1) {
	distsq2 = distsq1;
	distsq1 = dsq;
	minkey = k;
      } else if (dsq < distsq2) {
	distsq2 = dsq;
      }
    }



    /* Check whether closest distance is less than 0.6 of second. */
    if (10 * 10 * distsq1 < (int)(threshold * threshold * distsq2))
      return minkey;
    else return NULL;
}


/* Return squared distance between two keypoint descriptors.
*/
int DistSquared(Keypoint k1, Keypoint k2)
{
    int i, dif, distsq = 0;
    unsigned char *pk1, *pk2;

    pk1 = k1->descrip;
    pk2 = k2->descrip;

    for (i = 0; i < 128; i++) 
	{
	dif = (int) *pk1++ - (int) *pk2++;
      distsq += dif * dif;
    }
	
    return distsq;
}

