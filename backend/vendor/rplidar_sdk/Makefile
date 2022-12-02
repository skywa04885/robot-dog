CC_SOURCES += $(shell find ./src -maxdepth 1 -name "*.cpp")
CC_SOURCES += $(shell find ./src/hal -maxdepth 1 -name "*.cpp")
CC_SOURCES += $(shell find ./src/arch/linux -maxdepth 1 -name "*.cpp")
C_SOURCES += $(shell find . -name "*.c")

OBJECTS += $(CC_SOURCES:.cpp=.o)
OBJECTS += $(C_SOURCES:.c=.o)

ARCHIVER ?= $(shell which ar)
CC_COMPILER ?= $(shell which g++)
C_COMPILER ?= $(shell which gcc)

INCLUDE_DIRECTORIES += -I./include/
INCLUDE_DIRECTORIES += -I./src

OUTPUT_ARCHIVE += ./librplidar.a

%.o: %.c
	$(C_COMPILER) $(INCLUDE_DIRECTORIES) $(C_COMPILER_ARGS) -c $< -o $@
%.o: %.cpp
	$(CC_COMPILER) $(INCLUDE_DIRECTORIES) $(CC_COMPILER_ARGS) -c $< -o $@

all: $(OBJECTS)
	$(ARCHIVER) rvs $(OUTPUT_ARCHIVE) $(OBJECTS)
clean:
	rm -rf $(OBJECTS) $(OUTPUT_ARCHIVE)