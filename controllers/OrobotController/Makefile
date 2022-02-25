###
### Standard Makefile for Webots controllers
###
### Platform: Windows, Mac OS X, Linux
### Language: C, C++ and Java
###
### Authors: Olivier Michel - www.cyberbotics.com
### Revised: Tomislav Horvat - September 27th, 2018.
###
### Uncomment the variables to customize the Makefile

### -----C Sources-----
###
### if your controller uses several C sources files:
# C_SOURCES = my_controller.c my_clever_algo.c my_gui.c

### -----C++ Sources-----
###
### if your controller uses several C++ sources files:
# CPP_SOURCES = my_controller.cpp my_clever_algo.cpp my_gui.cpp
###     or
# CC_SOURCES = my_controller.cc my_clever_algo.cc my_gui.cc

#MAKEFLAGS += --jobs=8

CTRL_HOME = $(shell pwd)
CXX_SOURCES = 	source/robotSim.cpp main.cpp \
				source/controller.cpp source/joystick.cpp  \
				source/controller_misc.cpp source/controller_inverseKinematics.cpp \
				source/controller_getParameters.cpp\
				source/controller_trajectory.cpp \
				source/controller_walking.cpp \
				source/controller_joystick.cpp \
				source/utils.cpp \
				source/misc_math.cpp



### -----C/C++ Options----- source/controller_optimizationStuff.cpp
###
### if special CFLAGS are necessary, for example to set optimization level or
### to find include files:
# CFLAGS=-O3 -I/my_library_path/include

INCLUDE_DIRS = -I$(CTRL_HOME)/header -I$(CTRL_HOME)/Libraries   \
-I$(CTRL_HOME)/Libraries/qpOASES-3.2.0/include/  -I$(CTRL_HOME)/Libraries/dlib-19.0/


DISABLE_WARNING_FLAGS = -Wno-unused-variable -Wno-unused-but-set-variable -Wno-strict-overflow -Wno-write-strings \
						-Wno-unused-value -Wno-deprecated-declarations -Wno-deprecated \

CFLAGS=-O3  $(INCLUDE_DIRS)  -std=c++11 -pthread   $(DISABLE_WARNING_FLAGS)
###
### if your controller needs additional libraries:
# LIBRARIES=-L/path/to/my/library -lmy_library -lmy_other_library
LIBRARIES= 	-L$(CTRL_HOME)/Libraries/qpOASES-3.2.0/bin/ -lqpOASES

### Do not modify: this includes Webots global Makefile.include
WEBOTS_HOME?=/usr/local/webots
space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
