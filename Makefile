CC        := g++ -std=c++11 -O3 -Wno-deprecated -Wno-unused-result
LD        := g++
PREFIX    := /home/shaneg/installs
#use -O0 -g in CC and -g in LD for debugging segfaults.
#use -O3 in CC for fast code.

MODULES   := . FileParsing DataTypes Visualizations Optimization RFlowOptimization RFlowEvaluation ImageAlignment/DREAMFlow ImageAlignment/GeometricFlow ImageAlignment/LiuFlow ImageAlignment/FlowFrameworks BikeSurvey BoatSurvey VisualOdometry Tests
SRC_DIR   := $(addprefix src/,$(MODULES))
BUILD_DIR := $(addprefix build/,$(MODULES))

SRC       := $(foreach sdir,$(SRC_DIR),$(wildcard $(sdir)/*.cpp))
OBJ       := $(patsubst src/%.cpp,build/%.o,$(SRC))
ROOT_DIR  :=$(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))

INCLUDES  := -I/usr/include -I/usr/local/include -I$(PREFIX)/include -I$(PREFIX)/include/opencv2 -I$(PREFIX)/include/eigen3 -I$(PREFIX)/   include/gtsam -I$(ROOT_DIR)/src
LDFLAGS = -L$(PREFIX)/lib -lopencv_core -lopencv_calib3d -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs -lopencv_objdetect -         lopencv_videoio -lgtsam -lboost_system -ltbb -ltbbmalloc -lpthread -lHalf -lboost_serialization -lboost_filesystem -lboost_timer -         lboost_chrono -lopencv_video -lboost_thread -lboost_date_time -lboost_regex
#-lopencv_features2d


vpath %.cpp $(SRC_DIR)

define make-goal
$1/%.o: %.cpp
    $(CC) $(INCLUDES) -c $$< -o $$@
endef

.PHONY: all checkdirs clean

#pre-build:
#   @echo `scl enable devtoolset-1.1 bash`

all: checkdirs build/consec

build/consec: $(OBJ)
    $(LD) $^ $(LDFLAGS) -o $@


checkdirs: $(BUILD_DIR)

$(BUILD_DIR):
    @mkdir -p $@

clean:
    @rm -rf build

$(foreach bdir,$(BUILD_DIR),$(eval $(call make-goal,$(bdir))))
