/************************************************************************
  Copyright (c) 2003. David G. Lowe, University of British Columbia.
  This software is being made available for research purposes only
  (see file LICENSE for conditions).  This notice must be retained on
  all copies or modifications of this software.
*************************************************************************/

/* key.c:
   This file contains code to extract invariant keypoints from an image.
   The main routine is GetKeypoints(image) which returns a list of all
   invariant features from an image.
 */

#include "lowe_key.h"

#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
#pragma warning(disable:4244)
#endif

extern int CUSTOM_KEY_POOL;

/* --------------------------- Constants ------------------------------- */
/* These constants can be changed to experiment with keypoint detection. */

/* Set this constant to FALSE to skip step of doubling image size prior
   to finding keypoints.  This will reduce computation time by factor of 4
   but also find 4 times fewer keypoints.
*/
int DoubleImSize = TRUE;//FALSE; // FALSE

/* Scales gives the number of discrete smoothing levels within each octave.
   For example, Scales = 4 implies dividing octave into 4 intervals.
   Value of 3 works well, but higher values find more keypoints.
*/
int Scales = 3;//3
#define SCALES 3//3

/* InitSigma gives the amount of smoothing applied to the image at the
   first level of each octave.  In effect, this determines the sampling
   needed in the image domain relative to amount of smoothing.  Good
   values determined experimentally are in the range 1.4 to 1.8.
*/
float InitSigma = 1.6f;//1.6

/* Gaussian convolution kernels are truncated at this many sigmas from
   the center.  While it is more efficient to keep this value small,
   experiments show that for consistent scale-space analysis it needs
   a value at least 3.0, at which point the Gaussian has fallen to
   only 1% of its central value.  A value of 2.0 greatly reduces
   keypoint consistency, and a value of 4.0 is better than 3.0.
*/
const float GaussTruncate = 4.0;

/* Peaks in the DOG function must be at least BorderDist samples away
   from the image border, at whatever sampling is used for that scale.
   Keypoints close to the border (BorderDist < about 15) will have part
   of the descriptor landing outside the image, which is approximated by
   having the closest image pixel replicated.  However, to perform as much
   matching as possible close to the edge, use BorderDist of 4.
*/
const int BorderDist = 5;

/* Magnitude of difference-of-Gaussian value at a keypoint must be
   above PeakThresh.  This avoids considering points with very low
   contrast that are dominated by noise.  PeakThreshInit is divided by
   Scales during initialization to give PeakThresh, because more
   closely spaced scale samples produce smaller DOG values.  A value
   of 0.08 considers only the most stable keypoints, but applications
   may wish to use lower values such as 0.02 to find keypoints from
   low-contast regions.
*/
const float PeakThreshInit = 0.04f;
float PeakThresh;

/* EdgeEigenRatio is used to eliminate keypoints that lie on an edge
   in the image without their position being accurately determined
   along the edge.  This can be determined by checking the ratio of
   eigenvalues of a Hessian matrix of the DOG function computed at the
   keypoint.  The eigenvalues are proportional to the two principle
   curvatures.  An EdgeEigenRatio of 10 means that all keypoints with
   a ratio of principle curvatures greater than 10 are discarded.
*/
const float EdgeEigenRatio = 10.0;

/* If UseHistogramOri flag is TRUE, then the histogram method is used
   to determine keypoint orientation.  Otherwise, just use average
   gradient direction within surrounding region (which has been found
   to be less stable).  If using histogram, then OriBins gives the
   number of bins in the histogram (36 gives 10 degree spacing of
   bins).
*/
#define UseHistogramOri TRUE
#define OriBins 36

/* Size of Gaussian used to select orientations as multiple of scale
     of smaller Gaussian in DOG function used to find keypoint.
     Best values: 1.0 for UseHistogramOri = FALSE; 1.5 for TRUE.
*/
const float OriSigma = (UseHistogramOri ? 1.5 : 1.0);

/* All local peaks in the orientation histogram are used to generate
   keypoints, as long as the local peak is within OriHistThresh of
   the maximum peak.  A value of 1.0 only selects a single orientation
   at each location.
*/
const float OriHistThresh = 0.8f;

/* This constant specifies how large a region is covered by each index
   vector bin.  It gives the spacing of index samples in terms of
   pixels at this scale (which is then multiplied by the scale of a
   keypoint).  It should be set experimentally to as small a value as
   possible to keep features local (good values are in range 3 to 5).
*/
const int MagFactor = 3;

/* Width of Gaussian weighting window for index vector values.  It is
   given relative to half-width of index, so value of 1.0 means that
   weight has fallen to about half near corners of index patch.  A
   value of 1.0 works slightly better than large values (which are
   equivalent to not using weighting).  Value of 0.5 is considerably
   worse.
*/
const float IndexSigma = 1.0;  /* Was 1.0 */

/* If this is TRUE, then treat gradients with opposite signs as being
   the same.  In theory, this could create more illumination invariance,
   but generally harms performance in practice.
*/
const int IgnoreGradSign = FALSE;

/* Index values are thresholded at this value so that regions with
   high gradients do not need to match precisely in magnitude.
   Best value should be determined experimentally.  Value of 1.0
   has no effect.  Value of 0.2 is significantly better.
*/
const float MaxIndexVal = 0.2f;

/* Set SkipInterp to TRUE to skip the quadratic fit for accurate peak
   interpolation within the pyramid.  This can be used to study the value
   versus cost of interpolation.
*/
const int SkipInterp = FALSE;


/* -------------------- Local function prototypes ------------------------ */

Keypoint OctaveKeypoints(Image image, Image *pnextImage, float octSize,
			 Keypoint keys);
Keypoint FindMaxMin(Image *dogs, Image *blur, float octSize, Keypoint keys);
int LocalMaxMin(float val, Image dog, int row, int col);
int NotOnEdge(Image dog, int r, int c);
Keypoint InterpKeyPoint(Image *dogs, int s, int r, int c, Image grad,
	Image ori, Image map, float octSize, Keypoint keys, int movesRemain);
float FitQuadratic(float offset[3], Image *dogs, int s, int r, int c);
Keypoint AssignOriHist(Image grad, Image ori, float octSize, float octScale,
		       float octRow, float octCol, Keypoint keys);
void SmoothHistogram(float *hist, int bins);
float InterpPeak(float a, float b, float c);
Keypoint AssignOriAvg(Image grad, Image ori, float octSize, float octScale,
			float octRow, float octCol, Keypoint keys);
Keypoint MakeKeypoint(Image grad, Image ori, float octSize, float octScale,
		      float octRow, float octCol, float angle, Keypoint keys);
void MakeKeypointSample(Keypoint key, Image grad, Image ori, float scale,
			float row, float col);
void NormalizeVec(float *vec, int len);
void KeySampleVec(float *vec, Keypoint key, Image grad, Image ori,
		  float scale, float row, float col);
void KeySample(float index[IndexSize][IndexSize][OriSize], Keypoint key,
	       Image grad, Image ori, float scale, float row, float col);
void AddSample(float index[IndexSize][IndexSize][OriSize], Keypoint k,
	       Image grad, Image orim, int r, int c, float rpos, float cpos,
	       float rx, float cx);
void PlaceInIndex(float index[IndexSize][IndexSize][OriSize],
		  float mag, float ori, float rx, float cx);


/* ----------------------------- Routines --------------------------------- */


/* Given an image, find the keypoints and return a pointer to a list of
   keypoint records.
*/

Keypoint GetKeypoints( Image image ) {

   
    int minsize;
    float curSigma, sigma, octSize = 1.0;
    Keypoint keys = NULL;   /* List of keypoints found so far. */
    Image nextImage;

    PeakThresh = PeakThreshInit / Scales;

    /* If DoubleImSize flag is set, then double the image size prior to
       finding keypoints.  The size of pixels for first octave relative
       to input image, octSize, are divided by 2.
    */
    if (DoubleImSize) 
	{
      image = DoubleSize(image);
      octSize *= 0.5;
    }
	else
      image = LoweCopyImage(image, IMAGE_POOL);
   
    /* Apply initial smoothing to input image to raise its smoothing
       to InitSigma.  We assume image from camera has smoothing of
       sigma = 0.5, which becomes sigma = 1.0 if image has been doubled.
    */
    curSigma = (DoubleImSize ? 1.0 : 0.5);
    if (InitSigma > curSigma) {
      sigma = sqrt(InitSigma * InitSigma - curSigma * curSigma);
      GaussianBlur(image, sigma);
    }

    /* Examine one octave of scale space at a time.  Keep reducing
       image by factors of 2 until one dimension is smaller than
       minimum size at which a feature could be detected.
    */
    minsize = 2 * BorderDist + 2;
    
	while (image->rows > minsize &&  image->cols > minsize) 
	{
	keys = OctaveKeypoints(image, & nextImage, octSize, keys);
	image = HalfImageSize(nextImage);
	octSize *= 2.0;
    }
    
	/* Release all memory used to process this image. */
    FreeStoragePool(IMAGE_POOL);
    return keys;
}


/* Find keypoints within one octave of scale space starting with the
   given image.  The octSize parameter gives the size of each pixel
   relative to the input image.  Returns new list of keypoints after
   adding to the existing list "keys".
*/
Keypoint OctaveKeypoints(Image image, Image *pnextImage, float octSize,
			 Keypoint keys)
{
    int i;
    float sigmaRatio, prevSigma, increase;
    Image blur[SCALES+3], dogs[SCALES+2];
 
    /* Ratio of each scale to the previous one.  The parameter Scales
       determines how many scales we divide the octave into, so
          sigmaRatio ** Scales = 2.0.  
    */
    sigmaRatio = pow(2.0, 1.0 / (float) Scales);

    /* Build array "blur", holding Scales+3 blurred versions of the image. */
    blur[0] = image;          /* First level is input to this routine. */
    prevSigma = InitSigma;    /* Input image has InitSigma smoothing. */

    /* Form each level by adding incremental blur from previous level.
       Increase in blur is from prevSigma to prevSigma * sigmaRatio, so
       increase**2 + prevSigma**2 = (prevSigma * sigmaRatio)**2
    */
    for (i = 1; i < Scales + 3; i++) {
      blur[i] = LoweCopyImage(blur[i-1], IMAGE_POOL);
      increase = prevSigma * sqrt(sigmaRatio * sigmaRatio - 1.0);
      GaussianBlur(blur[i], increase);
      prevSigma *= sigmaRatio;
    }

    /* Compute an array, dogs, of difference-of-Gaussian images by
       subtracting each image from its next blurred version.
    */
    for (i = 0; i < Scales + 2; i++) {
      dogs[i] = LoweCopyImage(blur[i], IMAGE_POOL);
      SubtractImage(dogs[i], blur[i+1]);
    }

    /* Image blur[Scales] has twice the blur of starting image for
       this octave, so it is returned to downsample for next octave.
    */
    *pnextImage = blur[Scales];

    return FindMaxMin(dogs, blur, octSize, keys);
}


/* Find the local maxima and minima of the DOG images in scale space.
   Return the keypoints for these locations, added to existing "keys".
*/
Keypoint FindMaxMin(Image *dogs, Image *blur, float octSize, Keypoint keys)
{
   int s, r, c, rows, cols;
   float val, **pix;
   Image map, grad, ori;
   
   rows = dogs[0]->rows;
   cols = dogs[0]->cols;

   /* Create an image map in which locations that have a keypoint are
      marked with value 1.0, to prevent two keypoints being located at
      same position.  This may seem an inefficient data structure, but
      does not add significant overhead.
   */
   map = CreateImage(rows, cols, IMAGE_POOL);
   for (r = 0; r < rows; r++)
     for (c = 0; c < cols; c++)
       map->pixels[r][c] = 0.0;

   /* Search through each scale, leaving 1 scale below and 1 above.
      There are Scales+2 dog images.
   */
   for (s = 1; s < Scales+1; s++) {
      
     /* For each intermediate image, compute gradient and orientation
	images to be used for keypoint description.
     */
     grad = CreateImage(rows, cols, IMAGE_POOL);
     ori = CreateImage(rows, cols, IMAGE_POOL);
     GradOriImages(blur[s], grad, ori);

     pix = dogs[s]->pixels;   /* Pointer to pixels for this scale. */

     /* Only find peaks at least BorderDist samples from image border, as
	peaks centered close to the border will lack stability.
     */
     assert(BorderDist >= 2);
     for (r = BorderDist; r < rows - BorderDist; r++)
       for (c = BorderDist; c < cols - BorderDist; c++) {
	 val = pix[r][c];       /* Pixel value at (r,c) position. */

	 /* DOG magnitude must be above 0.8 * PeakThresh threshold
	    (precise threshold check will be done once peak
	    interpolation is performed).  Then check whether this
	    point is a peak in 3x3 region at each level, and is not
	    on an elongated edge.
	 */
	 if (fabs(val) > 0.8 * PeakThresh  &&
	     LocalMaxMin(val, dogs[s], r, c) &&
	     LocalMaxMin(val, dogs[s-1], r, c) &&
	     LocalMaxMin(val, dogs[s+1], r, c) &&
	     NotOnEdge(dogs[s], r, c))
	   keys = InterpKeyPoint(dogs, s, r, c, grad, ori, map, octSize,
				 keys, 5);
       }
   }
   return keys;
}


/* Return TRUE iff val is a local maximum (positive value) or
   minimum (negative value) compared to the 3x3 neighbourhood that
   is centered at (row,col).  
*/
int LocalMaxMin(float val, Image dog, int row, int col)
{
    int r, c;
    float **pix = dog->pixels;

    /* For efficiency, use separate cases for maxima or minima, and
       return as soon as possible. */
    if (val > 0.0) {
       for (r = row - 1; r <= row + 1; r++)
          for (c = col - 1; c <= col + 1; c++)
             if (pix[r][c] > val)
                return FALSE;
    } else {
       for (r = row - 1; r <= row + 1; r++)
          for (c = col - 1; c <= col + 1; c++)
             if (pix[r][c] < val)
                return FALSE;
    }
    return TRUE;
}


/* Returns FALSE if this point on the DOG function lies on an edge.
   This test is done early because it is very efficient and eliminates
   many points.  It requires that the ratio of the two principle
   curvatures of the DOG function at this point be below a threshold.
*/
int NotOnEdge(Image dog, int r, int c)
{
    float H00, H11, H01, det, trace, inc;
    float **d = dog->pixels;

    /* Compute 2x2 Hessian values from pixel differences. */
    H00 = d[r-1][c] - 2.0 * d[r][c] + d[r+1][c];
    H11 = d[r][c-1] - 2.0 * d[r][c] + d[r][c+1];
    H01 = ((d[r+1][c+1] - d[r+1][c-1]) - (d[r-1][c+1] - d[r-1][c-1])) / 4.0;

    /* Compute determinant and trace of the Hessian. */
    det = H00 * H11 - H01 * H01;
    trace = H00 + H11;

    /* To detect an edge response, we require the ratio of smallest
       to largest principle curvatures of the DOG function
       (eigenvalues of the Hessian) to be below a threshold.  For
       efficiency, we use Harris' idea of requiring the determinant to
       be above a threshold times the squared trace.
    */
    inc = EdgeEigenRatio + 1.0;
    return (det * inc * inc  > EdgeEigenRatio * trace * trace);
}


/* Create a keypoint at a peak near scale space location (s,r,c), where
   s is scale (index of DOGs image), and (r,c) is (row, col) location.
   Return the list of keys with any new keys added.
*/
Keypoint InterpKeyPoint(Image *dogs, int s, int r, int c, Image grad,
   Image ori, Image map, float octSize, Keypoint keys, int movesRemain)
{
    int newr = r, newc = c;
    float offset[3], octScale, peakval;

    /* The SkipInterp flag means that no interpolation will be performed
       and the peak will simply be assigned to the given integer sampling
       locations.
    */
    if (SkipInterp)
	{
      assert(UseHistogramOri);    /* Only needs testing for this case. */
      if (fabs(dogs[s]->pixels[r][c]) < PeakThresh)
		return keys;
      else
	 return AssignOriHist(grad, ori, octSize,
			    InitSigma * pow( (float)2.0, s / (float) Scales),
			    (float) r, (float) c, keys);
    }

    /* Fit quadratic to determine offset and peak value. */
    peakval = FitQuadratic(offset, dogs, s, r, c);

    /* Move to an adjacent (row,col) location if quadratic interpolation
       is larger than 0.6 units in some direction (we use 0.6 instead of
       0.5 to avoid jumping back and forth near boundary).  We do not
       perform move to adjacent scales, as it is seldom useful and we
       do not have easy access to adjacent scale structures.  The
       movesRemain counter allows only a fixed number of moves to
       prevent possibility of infinite loops.
    */
    if (offset[1] > 0.6 && r < dogs[0]->rows - 3)
      newr++;
    if (offset[1] < -0.6 && r > 3)
      newr--;
    if (offset[2] > 0.6 && c < dogs[0]->cols - 3)
      newc++;
    if (offset[2] < -0.6 && c > 3)
      newc--;
    if (movesRemain > 0  &&  (newr != r || newc != c))
      return InterpKeyPoint(dogs, s, newr, newc, grad, ori, map, octSize,
			    keys, movesRemain - 1);

    /* Do not create a keypoint if interpolation still remains far
       outside expected limits, or if magnitude of peak value is below
       threshold (i.e., contrast is too low).
    */
    if (fabs(offset[0]) > 1.5  || fabs(offset[1]) > 1.5  ||
	fabs(offset[2]) > 1.5 || fabs(peakval) < PeakThresh)
      return keys;

    /* Check that no keypoint has been created at this location (to avoid
       duplicates).  Otherwise, mark this map location.
    */
    if (map->pixels[r][c] > 0.0)
      return keys;
    map->pixels[r][c] = 1.0;

    /* The scale relative to this octave is given by octScale.  The scale
       units are in terms of sigma for the smallest of the Gaussians in the
       DOG used to identify that scale.
    */
    octScale = InitSigma * pow( (float)2.0, (s + offset[0]) / (float) Scales);

    if (UseHistogramOri)
      return AssignOriHist(grad, ori, octSize, octScale, r + offset[1],
			   c + offset[2], keys);
    else
      return AssignOriAvg(grad, ori, octSize, octScale, r + offset[1],
			  c + offset[2], keys);
}


/* Apply the method developed by Matthew Brown (see BMVC 02 paper) to
   fit a 3D quadratic function through the DOG function values around
   the location (s,r,c), i.e., (scale,row,col), at which a peak has
   been detected.  Return the interpolated peak position by setting
   the vector "offset", which gives offset from position (s,r,c).  The
   returned function value is the interpolated DOG magnitude at this peak.
*/
float FitQuadratic(float offset[3], Image *dogs, int s, int r, int c)
{
    float g[3], **dog0, **dog1, **dog2;
    static float **H = NULL;

    /* First time through, allocate space for Hessian matrix, H. */
    if (H == NULL) H = AllocMatrix(3, 3, PERM_POOL );

    /* Select the dog images at peak scale, dog1, as well as the scale
       below, dog0, and scale above, dog2.
    */
    dog0 = dogs[s-1]->pixels;
    dog1 = dogs[s]->pixels;
    dog2 = dogs[s+1]->pixels;

    /* Fill in the values of the gradient from pixel differences. */
    g[0] = (dog2[r][c] - dog0[r][c]) / 2.0;
    g[1] = (dog1[r+1][c] - dog1[r-1][c]) / 2.0;
    g[2] = (dog1[r][c+1] - dog1[r][c-1]) / 2.0;

    /* Fill in the values of the Hessian from pixel differences. */
    H[0][0] = dog0[r][c] - 2.0 * dog1[r][c] + dog2[r][c];
    H[1][1] = dog1[r-1][c] - 2.0 * dog1[r][c] + dog1[r+1][c];
    H[2][2] = dog1[r][c-1] - 2.0 * dog1[r][c] + dog1[r][c+1];
    H[0][1] = H[1][0] = ((dog2[r+1][c] - dog2[r-1][c]) -
			 (dog0[r+1][c] - dog0[r-1][c])) / 4.0;
    H[0][2] = H[2][0] = ((dog2[r][c+1] - dog2[r][c-1]) -
			 (dog0[r][c+1] - dog0[r][c-1])) / 4.0;
    H[1][2] = H[2][1] = ((dog1[r+1][c+1] - dog1[r+1][c-1]) -
			 (dog1[r-1][c+1] - dog1[r-1][c-1])) / 4.0;

    /* Solve the 3x3 linear sytem, Hx = -g.  Result gives peak offset.
       Note that SolveLinearSystem destroys contents of H.
    */
    offset[0] = - g[0];
    offset[1] = - g[1];
    offset[2] = - g[2];
    SolveLinearSystem(offset, H, 3); 

    /* Also return value of DOG at peak location using initial value plus
       0.5 times linear interpolation with gradient to peak position
       (this is correct for a quadratic approximation).  
    */
    return (dog1[r][c] + 0.5 * DotProd(offset, g, 3));
}


/* Assign an orientation to this keypoint.  This is done by creating a
     Gaussian weighted histogram of the gradient directions in the
     region.  The histogram is smoothed and the largest peak selected.
     The results are in the range of -PI to PI.
*/
Keypoint AssignOriHist(Image grad, Image ori, float octSize, float octScale,
		       float octRow, float octCol, Keypoint keys)
{
   int i, r, c, row, col, rows, cols, radius, bin, prev, next;
   float hist[OriBins], distsq, gval, weight, angle, sigma, interp,
         maxval = 0.0;
   
   row = (int) (octRow+0.5);
   col = (int) (octCol+0.5);
   rows = grad->rows;
   cols = grad->cols;
   
   for (i = 0; i < OriBins; i++)
      hist[i] = 0.0;
   
   /* Look at pixels within 3 sigma around the point and sum their
      Gaussian weighted gradient magnitudes into the histogram.
   */
   sigma = OriSigma * octScale;
   radius = (int) (sigma * 3.0); //3.0
   for (r = row - radius; r <= row + radius; r++)
      for (c = col - radius; c <= col + radius; c++)

         /* Do not use last row or column, which are not valid. */
         if (r >= 0 && c >= 0 && r < rows - 2 && c < cols - 2) {
            gval = grad->pixels[r][c];
            distsq = (r - octRow) * (r - octRow) + (c - octCol) * (c - octCol);

            if (gval > 0.0  &&  distsq < radius * radius + 0.5) {
               weight = exp(- distsq / (2.0 * sigma * sigma));
               /* Ori is in range of -PI to PI. */
               angle = ori->pixels[r][c];
               bin = (int) (OriBins * (angle + PI + 0.001) / (2.0 * PI));
               assert(bin >= 0 && bin <= OriBins);
               bin = LOWE_MIN(bin, OriBins - 1);
               hist[bin] += weight * gval;
            }
         }
   /* Apply circular smoothing 6 times for accurate Gaussian approximation. */
   for (i = 0; i < 6; i++)
     SmoothHistogram(hist, OriBins);

   /* Find maximum value in histogram. */
   for (i = 0; i < OriBins; i++)
      if (hist[i] > maxval)
         maxval = hist[i];

   /* Look for each local peak in histogram.  If value is within
      OriHistThresh of maximum value, then generate a keypoint.
   */
   for (i = 0; i < OriBins; i++) {
     prev = (i == 0 ? OriBins - 1 : i - 1);
     next = (i == OriBins - 1 ? 0 : i + 1);
     if (hist[i] > hist[prev]  &&  hist[i] > hist[next]  &&
	 hist[i] >= OriHistThresh * maxval) {
       
       /* Use parabolic fit to interpolate peak location from 3 samples.
	  Set angle in range -PI to PI.
       */
       interp = InterpPeak(hist[prev], hist[i], hist[next]);
       angle = 2.0 * PI * (i + 0.5 + interp) / OriBins - PI;
       assert(angle >= -PI  &&  angle <= PI);
       
       /* Create a keypoint with this orientation. */
       keys = MakeKeypoint(grad, ori, octSize, octScale, octRow, octCol,
			   angle, keys);
     }
   }
   return keys;
}


/* Smooth a histogram by using a [1/3 1/3 1/3] kernel.  Assume the histogram
   is connected in a circular buffer.
*/
void SmoothHistogram(float *hist, int bins)
{
   int i;
   float prev, temp;
   
   prev = hist[bins - 1];
   for (i = 0; i < bins; i++) {
      temp = hist[i];
      hist[i] = (prev + hist[i] + hist[(i + 1 == bins) ? 0 : i + 1]) / 3.0;
      prev = temp;
   }
}


/* Return a number in the range [-0.5, 0.5] that represents the
   location of the peak of a parabola passing through the 3 evenly
   spaced samples.  The center value is assumed to be greater than or
   equal to the other values if positive, or less than if negative.
*/
float InterpPeak(float a, float b, float c)
{
    if (b < 0.0) {
	a = -a; b = -b; c = -c;
    }
    assert(b >= a  &&  b >= c);
    return 0.5 * (a - c) / (a - 2.0 * b + c);
}


/* Alternate approach to return an orientation for a keypoint, as
   described in the PhD thesis of Krystian Mikolajczyk.  This is done
   by creating a Gaussian weighted average of the gradient directions
   in the region.  The result is in the range of -PI to PI.  This was
   found not to work as well, but is included so that comparisons can
   continue to be done.
*/
Keypoint AssignOriAvg(Image grad, Image ori, float octSize, float octScale,
		      float octRow, float octCol, Keypoint keys)
{
   int r, c, irow, icol, rows, cols, radius;
   float gval, sigma, distsq, weight, angle, xvec = 0.0, yvec = 0.0;
   
   rows = grad->rows;
   cols = grad->cols;
   irow = (int) (octRow+0.5);
   icol = (int) (octCol+0.5);
   
   /* Look at pixels within 3 sigma around the point and put their
      Gaussian weighted vector values in (xvec, yvec).
   */
   sigma = OriSigma * octScale;
   radius = (int) (3.0 * sigma);
   for (r = irow - radius; r <= irow + radius; r++)
      for (c = icol - radius; c <= icol + radius; c++)
         if (r >= 0 && c >= 0 && r < rows && c < cols) {
            gval = grad->pixels[r][c];
            distsq = (r - octRow) * (r - octRow) + (c - octCol) * (c - octCol);
            if (distsq <= radius * radius) {
               weight = exp(- distsq / (2.0 * sigma * sigma));
               /* Angle is in range of -PI to PI. */
               angle = ori->pixels[r][c];
	       xvec += gval * cos(angle);
	       yvec += gval * sin(angle);
            }
         }
   /* atan2 returns angle in range [-PI,PI]. */
   angle = atan2(yvec, xvec);  
   return MakeKeypoint(grad, ori, octSize, octScale, octRow, octCol,
		       angle, keys);
}


/* Create a new keypoint and return list of keypoints with new one added.
*/
Keypoint MakeKeypoint(Image grad, Image ori, float octSize, float octScale,
		      float octRow, float octCol, float angle, Keypoint keys)
{
    Keypoint k;
    //k = NEW(KeypointSt, KEY_POOL);
    k = NEW(KeypointSt, CUSTOM_KEY_POOL);

    k->next = keys;
    keys = k;
    k->ori = angle;
    k->row = octSize * octRow;
    k->col = octSize * octCol;
    k->scale = octSize * octScale;
    MakeKeypointSample(k, grad, ori, octScale, octRow, octCol);
    return keys;
}


/*--------------------- Making image descriptor -------------------------*/

/* Use the parameters of this keypoint to sample the gradient images
     at a set of locations within a circular region around the keypoint.
     The (scale,row,col) values are relative to current octave sampling.
     The resulting vector is stored in the key.
*/
void MakeKeypointSample(Keypoint key, Image grad, Image ori, float scale,
			float row, float col)
{
   int i, intval, changed = FALSE;
   float vec[VecLength];
   
   /* Produce sample vector. */
   KeySampleVec(vec, key, grad, ori, scale, row, col);
   
   /* Normalize vector.  This should provide illumination invariance
      for planar lambertian surfaces (except for saturation effects).
      Normalization also improves nearest-neighbor metric by
      increasing relative distance for vectors with few features.
   */
   NormalizeVec(vec, VecLength);

   /* Now that normalization has been done, threshold elements of
      index vector to decrease emphasis on large gradient magnitudes.
      Admittedly, the large magnitude values will have affected the
      normalization, and therefore the threshold, so this is of
      limited value.
   */
   for (i = 0; i < VecLength; i++)
     if (vec[i] > MaxIndexVal) {
       vec[i] = MaxIndexVal;
       changed = TRUE;
     }
   if (changed)
     NormalizeVec(vec, VecLength);

   /* Convert float vector to integer. Assume largest value in normalized
      vector is likely to be less than 0.5.
   */
   //key->ivec = (unsigned char*) MallocPool(VecLength * sizeof(unsigned char), KEY_POOL);
   key->ivec = (unsigned char*) MallocPool(VecLength * sizeof(unsigned char), CUSTOM_KEY_POOL);


   for (i = 0; i < VecLength; i++) {
     intval = (int) (512.0 * vec[i]);
     assert(intval >= 0);
     key->ivec[i] = (unsigned char) LOWE_MIN(255, intval);
   }
}


/* Normalize length of vec to 1.0.
*/
void NormalizeVec(float *vec, int len)
{
   int i;
   float val, fac, sqlen = 0.0;

   for (i = 0; i < len; i++) {
     val = vec[i];
     sqlen += val * val;
   }
   fac = 1.0 / sqrt(sqlen);
   for (i = 0; i < len; i++)
     vec[i] *= fac;
}


/* Create a 3D index array into which gradient values are accumulated.
   After filling array, copy values back into vec.
*/
void KeySampleVec(float *vec, Keypoint key, Image grad, Image ori,
		  float scale, float row, float col)
{
   int i, j, k, v;
   float index[IndexSize][IndexSize][OriSize];

   /* Initialize index array. */
   for (i = 0; i < IndexSize; i++)
      for (j = 0; j < IndexSize; j++)
         for (k = 0; k < OriSize; k++)
            index[i][j][k] = 0.0;
    
   KeySample(index, key, grad, ori, scale, row, col);
      
   /* Unwrap the 3D index values into 1D vec. */
   v = 0;
   for (i = 0; i < IndexSize; i++)
     for (j = 0; j < IndexSize; j++)
       for (k = 0; k < OriSize; k++)
	 vec[v++] = index[i][j][k];
}


/* Add features to vec obtained from sampling the grad and ori images
   for a particular scale.  Location of key is (scale,row,col) with respect
   to images at this scale.  We examine each pixel within a circular
   region containing the keypoint, and distribute the gradient for that
   pixel into the appropriate bins of the index array.
*/
void KeySample(float index[IndexSize][IndexSize][OriSize], Keypoint key,
	       Image grad, Image ori, float scale, float row, float col)
{
   int i, j, iradius, irow, icol;
   float spacing, radius, sine, cosine, rpos, cpos, rx, cx;

   irow = (int) (row + 0.5);
   icol = (int) (col + 0.5);
   sine = sin(key->ori);
   cosine = cos(key->ori);
          
   /* The spacing of index samples in terms of pixels at this scale. */
   spacing = scale * MagFactor;

   /* Radius of index sample region must extend to diagonal corner of
      index patch plus half sample for interpolation.
   */
   radius = 1.414 * spacing * (IndexSize + 1) / 2.0;
   iradius = (int) (radius + 0.5);
          
   /* Examine all points from the gradient image that could lie within the
      index square.
   */
   for (i = -iradius; i <= iradius; i++)
     for (j = -iradius; j <= iradius; j++) {
         
       /* Rotate sample offset to make it relative to key orientation.
	  Uses (row,col) instead of (x,y) coords.  Also, make subpixel
          correction as later image offset must be an integer.  Divide
          by spacing to put in index units.
       */
       rpos = ((cosine * i + sine * j) - (row - irow)) / spacing;
       cpos = ((- sine * i + cosine * j) - (col - icol)) / spacing;
         
       /* Compute location of sample in terms of real-valued index array
	  coordinates.  Subtract 0.5 so that rx of 1.0 means to put full
	  weight on index[1] (e.g., when rpos is 0 and IndexSize is 3.
       */
       rx = rpos + IndexSize / 2.0 - 0.5;
       cx = cpos + IndexSize / 2.0 - 0.5;

       /* Test whether this sample falls within boundary of index patch. */
       if (rx > -1.0 && rx < (float) IndexSize  &&   cx > -1.0 && cx < (float) IndexSize)
	 AddSample(index, key, grad, ori, irow + i, icol + j, rpos, cpos,
		   rx, cx);
     }
}


/* Given a sample from the image gradient, place it in the index array.
*/
void AddSample(float index[IndexSize][IndexSize][OriSize], Keypoint k,
	       Image grad, Image orim, int r, int c, float rpos, float cpos,
	       float rx, float cx)
{
    float mag, ori, sigma, weight;
    
    /* Clip at image boundaries. */
    if (r < 0  ||  r >= grad->rows  ||  c < 0  ||  c >= grad->cols)
       return;
    
    /* Compute Gaussian weight for sample, as function of radial distance
       from center.  Sigma is relative to half-width of index.
    */
    sigma = IndexSigma * 0.5 * IndexSize;
    weight = exp(- (rpos * rpos + cpos * cpos) / (2.0 * sigma * sigma));

    mag = weight * grad->pixels[r][c];

    /* Subtract keypoint orientation to give ori relative to keypoint. */
    ori = orim->pixels[r][c] - k->ori;
    
    /* Put orientation in range [0, 2*PI].  If sign of gradient is to
       be ignored, then put in range [0, PI].
    */
    if (IgnoreGradSign) {
       while (ori > PI)
          ori -= PI;
       while (ori < 0.0)
          ori += PI;
    } else {
       while (ori > 2*PI)
          ori -= 2*PI;
       while (ori < 0.0)
          ori += 2*PI;
    }
    PlaceInIndex(index, mag, ori, rx, cx);
}


/* Increment the appropriate locations in the index to incorporate
   this image sample.  The location of the sample in the index is (rx,cx).
*/
void PlaceInIndex(float index[IndexSize][IndexSize][OriSize],
		  float mag, float ori, float rx, float cx)
{
   int r, c, _or, ri, ci, oi, rindex, cindex, oindex;
   float oval, rfrac, cfrac, ofrac, rweight, cweight, oweight;
   float *ivec;
   
   oval = OriSize * ori / (IgnoreGradSign ? PI : 2*PI);
   
   ri = (int)((rx >= 0.0) ? rx : rx - 1.0);  /* Round down to next integer. */
   ci = (int)((cx >= 0.0) ? cx : cx - 1.0);
   oi = (int)((oval >= 0.0) ? oval : oval - 1.0);
   rfrac = rx - ri;         /* Fractional part of location. */
   cfrac = cx - ci;
   ofrac = oval - oi;
   assert(ri >= -1  &&  ri < IndexSize  &&  oi >= 0  &&  oi <= OriSize  &&
      rfrac >= 0.0  &&  rfrac <= 1.0);
   
   /* Put appropriate fraction in each of 8 buckets around this point
      in the (row,col,ori) dimensions.  This loop is written for
      efficiency, as it is the inner loop of key sampling.
   */
   for (r = 0; r < 2; r++) {
      rindex = ri + r;
      if (rindex >=0 && rindex < IndexSize) {
         rweight = mag * ((r == 0) ? 1.0 - rfrac : rfrac);
         
         for (c = 0; c < 2; c++) {
            cindex = ci + c;
            if (cindex >=0 && cindex < IndexSize) {
               cweight = rweight * ((c == 0) ? 1.0 - cfrac : cfrac);
               ivec = index[rindex][cindex];
               
               for (_or = 0; _or < 2; _or++) {
                  oindex = oi + _or;
                  if (oindex >= OriSize)  /* Orientation wraps around at PI. */
                     oindex = 0;
                  oweight = cweight * ((_or == 0) ? 1.0 - ofrac : ofrac);
                  
                  ivec[oindex] += oweight;
               }
            }
         }
      }
   }
}
