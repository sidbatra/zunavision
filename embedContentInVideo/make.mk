# Zunavision MAKE INCLUDE FILE
# Sid Batra <sidbatra@cs.stanford.edu>

OS = $(shell uname)
BIN_PATH = $(shell pwd)/bin
EXT_PATH =/afs/cs.stanford.edu/group/movie/zunavisionData/libs/opencv
#WXDIR =/afs/cs.stanford.edu/group/movie/zunavisionData/libs/wxGTK
#EXT_PATH =/afs/cs.stanford.edu/u/sidbatra/opencv
#WXDIR =/afs/cs.stanford.edu/u/sidbatra/wxGTK
#EXT_PATH =/afs/cs.stanford.edu/group/stair/libs32/opencv
#WXDIR =/afs/cs.stanford.edu/group/stair/libs32/wxGTK-2.8.0

# force 32-bit compile (set to 1 in make.local)
FORCE32BIT = 0
#ifeq (,$(findstring x86_64,$(MACHTYPE)))
#  FORCE32BIT = 1
#endif

# platform flags
PLAT_CFLAGS = -D__LINUX__
PLAT_LFLAGS = 

OPENGL_LFLAGS = -lGL -lGLU
JPEG_LFLAGS = -ljpeg

# link to OpenCV 
OPENCV_CFLAGS = -I$(EXT_PATH)/include/opencv
OPENCV_LFLAGS = -L$(EXT_PATH)/lib -lcv -lhighgui -lcxcore -lml

#debug:
#	@echo $(EXT_PATH)
#	@echo $(WXDIR)
#	@echo $(USE_WX)


#link to wxGTK
#WX_CFLAGS = `$(WXDIR)/buildgtk/wx-config --cflags`
#WX_LFLAGS = `$(WXDIR)/buildgtk/wx-config --libs core,base,gl`

EXTRA_CFLAGS += $(OPENCV_CFLAGS)
EXTRA_LFLAGS += $(OPENCV_LFLAGS) $(JPEG_LFLAGS)

EXTRA_CFLAGS += $(WX_CFLAGS)
EXTRA_LFLAGS += $(WX_LFLAGS)

EXTRA_LFLAGS += $(JPEG_LFLAGS)

# compiler and linker flags
ifeq ($(FORCE32BIT), 0)
  CFLAGS = -g -O3 -fPIC $(PLAT_CFLAGS) $(EXTRA_CFLAGS) -Wall
  LFLAGS = -g -lm -lpthread $(PLAT_LFLAGS) $(EXTRA_LFLAGS)
else
  CFLAGS = -g -O3 -m32 $(PLAT_CFLAGS) $(EXTRA_CFLAGS) -Wall
  LFLAGS = -g -lm -m32 -lpthread $(PLAT_LFLAGS) $(EXTRA_LFLAGS)
endif

CCC = g++
OBJ = $(SRC:.cpp=.o)

