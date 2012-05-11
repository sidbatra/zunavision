# Zunavision  MAKEFILE
# Sid Batra <sidbatra@cs.stanford.edu>, Ashutosh Saxena <asaxena@cs.stanford.edu>

-include make.mk
-include make.local

#######################################################################
# add application and shared source files here
#######################################################################
APP_SRC = trackContent.cpp renderContent.cpp convertMetadata.cpp video2images.cpp videoVolumeAnalyzer.cpp userInterface.cpp

OTHER_SRC = PNG/readpng.cpp SIFT/lowe_key.cpp SIFT/lowe_match.cpp SIFT/lowe_util.cpp sift.cpp SIFT/util.cpp \
	    trackCentral.cpp video.cpp contentTracker.cpp utilities.cpp  \
		visionUtilities.cpp constants.cpp  xmlUtilities.cpp \
		xmlParser.cpp renderingEngine.cpp  blendingManager.cpp pngReader.cpp

#######################################################################

APP_PROG_NAMES = $(APP_SRC:.cpp=)
APP_OBJ = $(APP_SRC:.cpp=.o)
OTHER_OBJ = $(OTHER_SRC:.cpp=.o)

.PHONY: clean
.PRECIOUS: $(APP_OBJ)

all: depend ${addprefix ${BIN_PATH}/,$(APP_PROG_NAMES)}

$(BIN_PATH)/%: %.o $(OTHER_OBJ)
	${CCC} $*.o -o $(@:.o=) $(OTHER_OBJ) $(LFLAGS)

.cpp.o:
	${CCC} ${CFLAGS} -c $< -o $@

depend:
	g++ ${CFLAGS} -MM ${APP_SRC} ${OTHER_SRC} >depend

clean:
	-rm $(APP_OBJ)
	-rm $(OTHER_OBJ)
	-rm ${addprefix ${BIN_PATH}/,$(APP_PROG_NAMES)}
	-rm depend

-include depend

