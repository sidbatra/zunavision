# MOVIE PROJECT  MAKEFILE
# Sid Batra <sidbatra@cs.stanford.edu>
# modified by Ashutosh Saxena <asaxena@cs.stanford.edu>

-include make.mk

#######################################################################
# add application and shared source files here
#######################################################################
APP_SRC = embedContentInVideo.cpp images2video.cpp video2images.cpp userInterface.cpp 

OTHER_SRC = lowe_key.cpp lowe_match.cpp lowe_util.cpp svlSIFT.cpp util.cpp \
	    embedCentral.cpp frameInstance.cpp objectTracker.cpp utilities.cpp  \
		visionUtilities.cpp constants.cpp frameSampler.cpp contentData.cpp

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

