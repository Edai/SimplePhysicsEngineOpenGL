# 15-462 project makefile
# Authors: Eric Butler
# DO NOT EDIT THIS FILE

#########
# USAGE #
#########
# the makefile assumes the following variables are defined:
# TOP_OBJ_DIR: the top object level directory
# MODE: the mode, either "debug" or "release"
# SRCS: the source files
# TARGET: the target (executable) name

ifeq ($(TARGET),)
ERRORMSG = "No target specified"
else ifeq ($(TOP_OBJ_DIR),)
ERRORMSG = "No object directory specified"
else ifeq ($(SRCS),)
ERRORMSG = "No sources specified"
endif

# the current directory
CURR_DIR = $(shell pwd)

# global compiler flags
CXX = g++
CXXFLAGS +=  -Wall -ansi -pedantic -I"$(CURR_DIR)/include" -I"$(CURR_DIR)/$(SRC_DIR)" -std=c++11
LDFLAGS = -L"$(CURR_DIR)/lib" -lGL -lGLU -lSDLmain -lSDL -lpng -lGLEW

# object directories, mode flags

ifeq ($(MODE), release)
	SUB_OBJ_DIR = release
	CXXFLAGS += -O2
else ifeq ($(MODE), debug)
	SUB_OBJ_DIR = debug
	CXXFLAGS += -g -O0
else
ERRORMSG = "unknown build mode: $(MODE)"
endif

OBJ_DIR = $(TOP_OBJ_DIR)/$(SUB_OBJ_DIR)

# list of all object files
OBJS = $(SRCS:.cpp=.o)
# list of all dep files
DEPS = $(OBJS:.o=.d)
# full list of paths to all objs
OBJS_FULL = $(addprefix $(OBJ_DIR)/,$(OBJS))
# full list of paths to all deps
DEPS_FULL = $(addprefix $(OBJ_DIR)/,$(DEPS))

# sanity check for '.cpp' suffix
TMP_SRCS_NOT_CPP = $(filter-out %.cpp,$(SRCS))
ifneq (,$(TMP_SRCS_NOT_CPP))
ERRORMSG = "Feeling nervous about '$(TMP_SRCS_NOT_CPP)'; I only know how to build .cpp files"
endif

# targets

.PHONY: target

ifneq ($(ERRORMSG),)
target:
	$(error $(ERRORMSG))
else
target: $(OBJ_DIR)/$(TARGET).exe
	cp $< $(TARGET)

$(OBJ_DIR)/%.d: $(SRC_DIR)/%.cpp
	mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -MM -MP -MT $(@:.d=.o) -o $@ $<

# don't need to mkdir for object files since d files already exist
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

ifneq ($(MAKECMDGOALS),clean)
-include $(DEPS_FULL)
endif

$(OBJ_DIR)/$(TARGET).exe: $(OBJS_FULL)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)
endif

