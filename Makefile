
TARGETS=mogsWithStraightSeq mogsServerWithStoppingAction

SOURCES=mogsServerWithStoppingAction.cpp mogsWithStraightSeq.cpp ActionGotoStraight.cpp GPSMapTools.cpp ActionLimiterForwards.cpp

ifndef ARNL
ARNL:=/usr/local/Arnl
endif

LFLAGS:=-L$(ARNL)/lib

LINK:=-lMogs -lBaseArnl -lArNetworkingForArnl -lAriaForArnl -ldl -lrt -lm -lpthread

INCLUDE:=-I$(ARNL)/include -I$(ARNL)/include/Aria -I$(ARNL)/include/ArNetworking

CXXFLAGS:=-fPIC -g -Wall -D_REENTRANT

OBJ:=$(patsubst %.c,%.o,$(patsubst %.cc,%.o,$(patsubst %.cpp,%.o,$(SOURCES))))

ifndef CXX
CXX:=c++
endif

all: $(TARGETS)

mogsWithStraightSeq: mogsWithStraightSeq.cpp ActionGotoStraight.o GPSMapTools.o ActionLimiterForwards.o
	$(CXX) $(CXXFLAGS) $(INCLUDE) $(LFLAGS) -o $@ $^ $(LINK)

mogsServerWithStoppingAction: mogsServerWithStoppingAction.cpp GPSMapTools.o
	$(CXX) $(CXXFLAGS) $(INCLUDE) $(LFLAGS) -o $@ $^ $(LINK)

%.o: %.cpp
	$(CXX) -c $(CXXFLAGS) $(INCLUDE) -o $@ $<

%.o: %.cc
	$(CXX) -c $(CXXFLAGS) $(INCLUDE) -o $@ $<

%.o: %.c
	$(CXX) -c $(CXXFLAGS) $(INCLUDE) -o $@ $<
	
clean: 
	-rm $(TARGETS) $(OBJ)

Makefile.dep: $(SOURCES)
	$(CXX) $(CXXFLAGS) $(INCLUDE) -MM $(SOURCES) >Makefile.dep

-include Makefile.dep

# to force remaking of Makefile.dep:
dep: clean Makefile.dep

.PHONY: clean all dep
