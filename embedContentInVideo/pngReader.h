#define PROGNAME  "rpng-x"
#define LONGNAME  "Simple PNG Viewer for X"
#define VERSION   "2.01 of 16 March 2008"
#define RESNAME   "rpng"	/* our X resource application name */
#define RESCLASS  "Rpng"	/* our X resource class name */

#include "base.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xos.h>
#include <X11/keysym.h>


#include "PNG/readpng.h"   /* typedefs, common macros, readpng prototypes */


class pngReader
{

  char titlebar[1024],*window_name;
  char *appname;
  char *icon_name;
  char *res_name;
  char *res_class;
  const char *filename;
  FILE *infile;

  char *bgstr;
  uch bg_red, bg_green, bg_blue;

  double display_exponent;

  ulg image_width, image_height, image_rowbytes;
  int image_channels;
  uch *image_data;

/* X-specific variables */
  char *displayname;
  XImage *ximage;
  Display *display;
  int depth;
  Visual *visual;
  XVisualInfo *visual_list;
  int RShift, GShift, BShift;
  ulg RMask, GMask, BMask;
  Window window;
  GC gc;
  Colormap colormap;

  int have_nondefault_visual;
  int have_colormap ;
  int have_window ;
  int have_gc ;

  public:
  IplImage *sourceImage ;
  IplImage *maskImage ;

public:
pngReader()
{

 appname = new char[1024];
 icon_name = new char[1024];
 res_name = new char[1024];
 res_class = new char[1024];
 window_name = new char[1024];

  bg_red=bg_green=bg_blue=0;
  
  have_nondefault_visual = FALSE;
  have_colormap = FALSE;
  have_window = FALSE;
  have_gc = FALSE;
}

int mainFunction(string path)
{



#ifdef sgi
    char tmpline[80];
#endif
    char *p;
    int rc, alen, flen;
    int error = 0;
    double LUT_exponent;               /* just the lookup table */
    double CRT_exponent = 2.2;         /* just the monitor */
    double default_display_exponent;   /* whole display system */

    displayname = (char *)NULL;
    filename = (char *)NULL;


    /* First set the default value for our display-system exponent, i.e.,
     * the product of the CRT exponent and the exponent corresponding to
     * the frame-buffer's lookup table (LUT), if any.  This is not an
     * exhaustive list of LUT values (e.g., OpenStep has a lot of weird
     * ones), but it should cover 99% of the current possibilities. */

#if defined(NeXT)
    LUT_exponent = 1.0 / 2.2;
    /*
    if (some_next_function_that_returns_gamma(&next_gamma))
        LUT_exponent = 1.0 / next_gamma;
     */
#elif defined(sgi)
    LUT_exponent = 1.0 / 1.7;
    /* there doesn't seem to be any documented function to get the
     * "gamma" value, so we do it the hard way */
    infile = fopen("/etc/config/system.glGammaVal", "r");
    if (infile) {
        double sgi_gamma;

        fgets(tmpline, 80, infile);
        fclose(infile);
        sgi_gamma = atof(tmpline);
        if (sgi_gamma > 0.0)
            LUT_exponent = 1.0 / sgi_gamma;
    }
#elif defined(Macintosh)
    LUT_exponent = 1.8 / 2.61;
    /*
    if (some_mac_function_that_returns_gamma(&mac_gamma))
        LUT_exponent = mac_gamma / 2.61;
     */
#else
    LUT_exponent = 1.0;   /* assume no LUT:  most PCs */
#endif

    /* the defaults above give 1.0, 1.3, 1.5 and 2.2, respectively: */
    default_display_exponent = LUT_exponent * CRT_exponent;


    /* If the user has set the SCREEN_GAMMA environment variable as suggested
     * (somewhat imprecisely) in the libpng documentation, use that; otherwise
     * use the default value we just calculated.  Either way, the user may
     * override this via a command-line option. */

    if ((p = getenv("SCREEN_GAMMA")) != NULL)
        display_exponent = atof(p);
    else
        display_exponent = default_display_exponent;


filename = path.c_str();

    if (!(infile = fopen(filename, "rb"))) {
        fprintf(stderr, PROGNAME ":  can't open PNG file [%s]\n", filename);
        ++error;
    } else {
        if ((rc = readpng_init(infile, &image_width, &image_height)) != 0) {
            switch (rc) {
                case 1:
                    fprintf(stderr, PROGNAME
                      ":  [%s] is not a PNG file: incorrect signature\n",
                      filename);
                    break;
                case 2:
                    fprintf(stderr, PROGNAME
                      ":  [%s] has bad IHDR (libpng longjmp)\n", filename);
                    break;
                case 4:
                    fprintf(stderr, PROGNAME ":  insufficient memory\n");
                    break;
                default:
                    fprintf(stderr, PROGNAME
                      ":  unknown readpng_init() error\n");
                    break;
            }
            ++error;
        } 
 	else 
	{
            display = XOpenDisplay(displayname);
            if (!display) 
	    {
                readpng_cleanup(TRUE);
                fprintf(stderr, PROGNAME ":  can't open X display [%s]\n",
                  displayname? displayname : "default");
                ++error;
            }
        }
        if (error)
            fclose(infile);
    }


    if (error) {
        fprintf(stderr, PROGNAME ":  aborting.\n");
        exit(2);
    }


    /* set the title-bar string, but make sure buffer doesn't overflow */

    alen = strlen(appname);
    flen = strlen(filename);
    if (alen + flen + 3 > 1023)
        sprintf(titlebar, "%s:  ...%s", appname, filename+(alen+flen+6-1023));
    else
        sprintf(titlebar, "%s:  %s", appname, filename);



    image_data = readpng_get_image(display_exponent, &image_channels,
      &image_rowbytes);


    readpng_cleanup(FALSE);
    fclose(infile);

    if (!image_data) 
    {
        fprintf(stderr, PROGNAME ":  unable to decode PNG image\n");
        exit(3);
    }

    
    if (rpng_x_display_image()) 
    {
        free(image_data);
        exit(4);
    }

    rpng_x_cleanup();

    return 0;
}




int rpng_x_display_image(void)
{
    uch *src;
    uch r, g, b, a;
    ulg i, row, lastrow = 0;
    int screen = DefaultScreen(display);
    depth = DisplayPlanes(display, screen);
    sourceImage = cvCreateImage(cvSize((int)image_width,(int)image_height),IPL_DEPTH_8U,3);
    maskImage = cvCreateImage(cvSize((int)image_width,(int)image_height),IPL_DEPTH_8U,1);



    if (depth == 24 || depth == 32) 
    {
        ulg red, green, blue;

        for (lastrow = row = 0;  row < image_height;  ++row) 
	{
            src = image_data + row*image_rowbytes;
            if (image_channels == 3) 
  	    {
                for (i = image_width;  i > 0;  --i) 
		{
                    red   = *src++;
                    green = *src++;
                    blue  = *src++;
		    cvSet2D(sourceImage,row,i-1,cvScalar((uch)blue,(uch)green,(uch)red));
		    CV_IMAGE_ELEM(maskImage,unsigned char,row,i-1) = 0;	
                }
            } 
	     else /* if (image_channels == 4) */ 
	      {
                for (i = image_width;  i > 0;  --i) 
	        {
                    r = *src++; g = *src++; b = *src++;
                    a = *src++;
		    cvSet2D(sourceImage,row,i-1,cvScalar((uch)b,(uch)g,(uch)r));
		    CV_IMAGE_ELEM(maskImage,unsigned char,row,i-1) = 255 - (uch)a;	
                }
            }
        }

    }//24 bit 
    else if (depth == 16) 
    {
        ush red, green, blue;

        for (lastrow = row = 0;  row < image_height;  ++row) 
	{
            src = image_data + row*image_rowbytes;
            if (image_channels == 3) 
	    {
                for (i = image_width;  i > 0;  --i) 
		{
                    red   = ((ush)(*src) << 8);
                    ++src;
                    green = ((ush)(*src) << 8);
                    ++src;
                    blue  = ((ush)(*src) << 8);
                    ++src;
		    cvSet2D(sourceImage,row,i-1,cvScalar((uch)blue,(uch)green,(uch)red));
		    CV_IMAGE_ELEM(maskImage,unsigned char,row,i-1) = 0;	
                }
            } else /* if (image_channels == 4) */ 
	    {
                for (i = image_width;  i > 0;  --i) 
		{
                    r = *src++;
                    g = *src++;
                    b = *src++;
                    a = *src++;
		    cvSet2D(sourceImage,row,i-1,cvScalar((uch)b,(uch)g,(uch)r));
		    CV_IMAGE_ELEM(maskImage,unsigned char,row,i-1) = 255 - (uch)a;	
                }
            }
        }
    } 
    else /* depth == 8 */ 
    {
        /* GRR:  add 8-bit support */
    }

    return 0;
}




  void rpng_x_cleanup(void)
{
    if (image_data) {
        free(image_data);
        image_data = NULL;
    }

    if (have_gc)
        XFreeGC(display, gc);

    if (have_window)
        XDestroyWindow(display, window);

    if (have_colormap)
        XFreeColormap(display, colormap);

    if (have_nondefault_visual)
        XFree(visual_list);
}





  int rpng_x_msb(ulg u32val)
{
    int i;

    for (i = 31;  i >= 0;  --i) {
        if (u32val & 0x80000000L)
            break;
        u32val <<= 1;
    }
    return i;
}


};//end of pngReader
