
#ifndef TRUE
#  define TRUE 1
#  define FALSE 0
#endif

#ifndef MAX
#  define MAX(a,b)  ((a) > (b)? (a) : (b))
#  define MIN(a,b)  ((a) < (b)? (a) : (b))
#endif

#ifdef DEBUG
#  define Trace(x)  {fprintf x ; fflush(stderr); fflush(stdout);}
#else
#  define Trace(x)  ;
#endif

typedef unsigned char   uch;
typedef unsigned short  ush;
typedef unsigned long   ulg;


/* prototypes for public functions in readpng.c */

void readpng_version_info(void);

int readpng_init(FILE *infile, ulg *pWidth, ulg *pHeight);

int readpng_get_bgcolor(uch *bg_red, uch *bg_green, uch *bg_blue);

uch *readpng_get_image(double display_exponent, int *pChannels,
                       ulg *pRowbytes);

void readpng_cleanup(int free_image_data);
