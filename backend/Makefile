CC_COMPILER ?= $(shell which g++)
PROTO_COMPILER ?= $(shell which proto)

CC_SOURCES += $(shell find ./app -name "*.cc")

OUTPUT ?= main.o

OBJECTS += $(CC_SOURCES:.cc=.o)
OBJECTS += $(C_SOURCES:.c=.o)

CC_INCLUDE_DIRECTORIES += -I./vendor/rplidar_sdk/include

CC_COMPILER_DEFINITIONS += -DCL_TARGET_OPENCL_VERSION=300
CC_COMPILER_DEFINITIONS += -DBOOST_LOG_DYN_LINK

CC_LINKER_ARGS += -L./vendor/rplidar_sdk

CC_LINKER_LIBS += -lrplidar
CC_LINKER_LIBS += -lboost_log
CC_LINKER_LIBS += -lboost_system
CC_LINKER_LIBS += -lboost_thread
CC_LINKER_LIBS += -lboost_program_options
CC_LINKER_LIBS += -lOpenCL

%.o: %.cc
	$(CC_COMPILER) $(CC_INCLUDE_DIRECTORIES) $(CC_COMPILER_DEFINITIONS) $(CC_COMPILER_ARGS) -O3 -g -c $< -o $@

all: $(OBJECTS)
	$(CC_COMPILER) $(OBJECTS) -g -o $(OUTPUT) $(CC_LINKER_ARGS) $(CC_LINKER_LIBS)
clean:
	rm -rf $(OBJECTS) $(OUTPUT)