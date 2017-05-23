CC        := g++ -std=c++11 -O3
LD        := g++

MODULES   := . FileParsing DataTypes Optimization Visualizations
SRC_DIR   := $(addprefix src/,$(MODULES))
BUILD_DIR := $(addprefix build/,$(MODULES))

SRC       := $(foreach sdir,$(SRC_DIR),$(wildcard $(sdir)/*.cpp))
OBJ       := $(patsubst src/%.cpp,build/%.o,$(SRC))
prefix    := /share/apps
ROOT_DIR  :=$(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))
INCLUDES  := -I/usr/include -I/usr/local/include -I$(prefix)/include -I$(prefix)/include/opencv2 -I$(prefix)/include/eigen3 -I$(prefix)/include/gtsam -I$(ROOT_DIR)/src
LDFLAGS	= -L$(prefix)/lib -lopencv_features2d -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_nonfree -lgtsam -lgtsam_unstable -lboost_system -ltbb -ltbbmalloc -lpqxx -lpq
#-lopencv_imgcodecs

vpath %.cpp $(SRC_DIR)

define make-goal
$1/%.o: %.cpp
	$(CC) $(INCLUDES) -c $$< -o $$@
endef

.PHONY: all checkdirs clean

#pre-build:
#	@echo `scl enable devtoolset-1.1 bash`

all: checkdirs build/cb_script

build/cb_script: $(OBJ)
	$(LD) $^ $(LDFLAGS) -o $@


checkdirs: $(BUILD_DIR)

$(BUILD_DIR):
	@mkdir -p $@

clean:
	@rm -rf build

$(foreach bdir,$(BUILD_DIR),$(eval $(call make-goal,$(bdir))))

## Makefile based on an example at
##  http://stackoverflow.com/questions/231229/how-to-generate-a-makefile-with-source-in-sub-directories-using-just-one-makefil
