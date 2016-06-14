
# Copy this Makefile as a starting point if you are developing your own project
# outside of the ARIA examples, for example in your home directory, with
# multiple source files that must be compiled and linked to ARIA.
#
# You must set the value of TARGET to the name of your program. 
#
# This uses the $(wildcard *.cpp) function, so all .cpp files
# found in the current directory will be compiled and linked
# to produce $(TARGET). If instead you want to list the source
# files individually, replace $(wildcard *.cpp) with the list of
# files.
# libAria will be linked automatically to the target. If you need additional libraries
# such as ArNetworking, ARNL, or other libraries, prepend them to
# the LINK variable.

TARGET = myProgram

# If you use a different file extension for source files such as .cc, change it here.
CXXFILEEXT=.cpp

SOURCES = $(wildcard *$(CXXFILEEXT))
# OR list them explicitly e.g.: SOURCES = myProgram.cpp myUtil.cpp myClass.cpp 

LINK = -lAria -ldl -lrt -lm -lpthread
# e.g. to add ArNetworking: LINK = -lArNetworking -lAria -ldl -lrt -lpthread -lm
# e.g. for ARNL instead: LINK = -lArnl -lArnlBase -lArNetworking -lAriaForArnl -ldl -lrt -lpthread -lm

LFLAGS = -L/usr/local/Aria/lib
# e.g. for ARNL: LINK = -L/usr/local/Arnl/lib

INCLUDE = -I/usr/local/Aria/include
# e.g. for ARNL: INCLUDE = -I/usr/local/Arnl/include # -I/usr/local/Arnl/include/Aria -I/usr/local/Arnl/include/ArNetworking

CXXFLAGS = -fPIC -g -Wall -D_REENTRANT


OBJ := $(patsubst %$(CXXFILEEXT),%.o,$(SOURCES))

ifndef CXX
CXX:=c++
endif

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) $(INCLUDE) $(LFLAGS) -o $@ $^ $(LINK)

%.o: %$(CXXFILEEXT) 
	$(CXX) -c $(CXXFLAGS) $(INCLUDE) -o $@ $<
	
clean: 
	-rm $(TARGET) $(OBJ)

Makefile.dep: $(SOURCES)
	$(CXX) $(CXXFLAGS) $(INCLUDE) -MM $(SOURCES) >Makefile.dep

-include Makefile.dep

# to force remaking of Makefile.dep:
dep: clean Makefile.dep

.PHONY: clean all dep
