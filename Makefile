CC        := g++ -std=c++11 -O3 -Wno-deprecated -Wno-unused-result
LD        := g++
PREFIX    := /Volumes/Untitled/installs
#use -O0 -g in CC and -g in LD for debugging segfaults.
#use -O3 in CC for fast code.

MODULES   := . FileParsing DataTypes Visualizations Optimization/SingleSession Optimization/MultiSession RFlowOptimization BoatSurvey VisualOdometry VIOCompetition
SRC_DIR   := $(addprefix src/,$(MODULES))
BUILD_DIR := $(addprefix build/,$(MODULES))

SRC       := $(foreach sdir,$(SRC_DIR),$(wildcard $(sdir)/*.cpp))
OBJ       := $(patsubst src/%.cpp,build/%.o,$(SRC))
ROOT_DIR  :=$(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))
GTSAMVERSION := -DGTSAM4
GTSAMPREFIX := $(PREFIX)/gtsam4

INCLUDES  := -I/usr/include -I/usr/local/include -I$(PREFIX)/include -I$(PREFIX)/include/opencv2 -I/opt/local/include/eigen3 -I/opt/local/include -I$(GTSAMPREFIX)/include -I$(ROOT_DIR)/src -I$(GTSAMPREFIX)/include/gtsam/3rdparty/Eigen
LDFLAGS = -L$(PREFIX)/lib -L$(GTSAMPREFIX)/lib -L/opt/local/lib -lopencv_core -lopencv_calib3d -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs -lopencv_objdetect -lopencv_videoio -lgtsam -lboost_system-mt -ltbb -ltbbmalloc -lpthread -lboost_serialization-mt -lboost_filesystem-mt -lboost_timer-mt -lboost_chrono-mt -lopencv_video -lboost_thread-mt -lboost_date_time-mt -lboost_regex-mt


vpath %.cpp $(SRC_DIR)

define make-goal
$1/%.o: %.cpp
	$(CC) $(GTSAMVERSION) $(INCLUDES) -c $$< -o $$@
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
