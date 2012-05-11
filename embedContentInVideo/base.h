/*****************************************************************************
** ZunaVision
** Copyright (c) 2008 
**
** FILENAME:    base.h
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Includes the core set of headers needed by all the files in the solution
*****************************************************************************/

#pragma once

#include <cstdlib>
#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <time.h>
#include <string>
#include <algorithm>
#include <vector>
#include <sstream>
#include <map>

#include <sys/types.h>
#include <sys/stat.h>

#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
#include <windows.h>
#include "dirent.h"
#else
#include <dirent.h>
#endif

#include "xmlParser.h"

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "constants.h"
#include "svlPoint2d.h"
#include "utilities.h"
#include "xmlUtilities.h"
#include "visionUtilities.h"


