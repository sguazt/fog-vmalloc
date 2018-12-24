project_home=$(CURDIR)
#lib_path ?= $(prj_path)/lib
thirdparty_path = $(project_home)/thirdparty

ifneq (,$(wildcard ./config.mk))
include config.mk
endif
#include config_default.mk


CXXFLAGS += -Wall -Wextra -std=c++14 -pedantic
LDLIBS += -lm

# The two lines below are a trick to cheat Make in calling c++ instead of cc when linking object files
CC = $(CXX)
CFLAGS = $(CXXFLAGS)

## [build]
CXXFLAGS += $(build_cflags)
LDFLAGS += $(build_ldflags)
LDLIBS += $(build_ldlibs)

## [Boost]
CXXFLAGS += $(boost_cflags)
CXXFLAGS += -DBOOST_THREAD_VERSION=4
LDFLAGS += $(boost_ldflags)
LDLIBS += -lboost_system -lboost_chrono -lboost_thread -lpthread -lrt
LDLIBS += -lboost_timer
LDLIBS += $(boost_ldlibs)

# [CPLEX]
CXXFLAGS += $(cplex_cflags)
LDFLAGS += $(cplex_ldflags)
LDLIBS += $(cplex_ldlibs)

# [dcsxx-commons]
CXXFLAGS += $(dcs_commons_cflags)
LDFLAGS += $(dcs_commons_ldflags)
LDLIBS += $(dcs_commons_ldlibs)

## [fog]
CXXFLAGS += $(fog_cflags)
CXXFLAGS += -I$(project_home)/c++/include
CXXFLAGS += -I$(project_home)/python
CXXFLAGS += -DDCS_LOGGING_STREAM=std::cout
CXXFLAGS += -DDCS_DEBUG_STREAM=std::cout
LDFLAGS += $(fog_ldflags)
LDLIBS += $(fog_ldlibs)

## [Python]
#CXXFLAGS += $(shell pkg-config python2 --cflags)
#LDFLAGS += $(shell pkg-config python2 --libs)
CXXFLAGS += $(shell python2-config --cflags)
LDFLAGS += $(shell python2-config --ldflags)
PYTHONPATH += :$(project_home)/python/rndwaypoint:$(project_home)/python/trivedi

## [Google or-tools]
CXXFLAGS += -I$(project_home)/thirdparty/or-tools


export project_home thirdparty_path
export CC CFLAGS CXXFLAGS LDFLAGS LDLIBS
export PYTHONPATH


.PHONY: all clean debug release test tools version


#all: version src/fog_coalform
all: release

debug: CXXFLAGS+=-g -Og -UNDEBUG
debug: version c++/src/fog_vmalloc

release: CXXFLAGS+=-O3 -DNDEBUG
release: version c++/src/fog_vmalloc

#test: CXXFLAGS+=-g -Og -UNDEBUG
#test:
#	cd test && $(MAKE)

#tools: CXXFLAGS+=-g -Og -UNDEBUG
#tools:
#	cd tools && $(MAKE)

#version:
#	./dist/make_version.sh $(project_home)

#c++/src/fog_vmalloc: c++/src/fog_vmalloc.o python/rndwaypoint/RndWalkPnt.o python/trivedi/TrivediPerc.o
c++/src/fog_vmalloc: c++/src/fog_vmalloc.o python/rndwaypoint/RndWalkPnt.o thirdparty/or-tools/ortools/algorithms/hungarian.o

clean:
	$(RM) c++/src/fog_vmalloc \
		  c++/src/*.o \
		  python/*.o \
		  python/*.pyc \
		  python/rndwaypoint/*.o \
		  python/rndwaypoint/*.pyc \
		  python/trivedi/*.o \
		  python/trivedi/*.pyc \
		  test/*.o \
		  vgcore.*
