
CC ?= $(CROSS)gcc
CXX ?= $(CROSS)g++
LD ?= $(CROSS)ld
AR ?= $(CROSS)ar
RANLIB ?= $(CROSS)ranlib

INCLUDES=-I../../include

CFLAGS=-pipe -fexceptions $(DEBUG_FLAGS) $(OPTIMIZE_FLAGS) -Wall -std=gnu99 $(TEST_FLAGS)
CXXFLAGS=-pipe -fexceptions $(DEBUG_FLAGS) $(OPTIMIZE_FLAGS) -Wall -std=gnu++17 $(TEST_FLAGS)

TARGET=net.a

SRCCS=$(wildcard *.c)
COBJS=$(SRCCS:%.c=%.o)
SRCCXXS=$(wildcard *.cpp)
CXXOBJS=$(SRCCXXS:%.cpp=%.o)
LIBOBJ=$(COBJS) $(CXXOBJS)

HEADERS=$(wildcard ../include/lighten/*.hpp)

LIBS=
LDFLAGS=

.PHONY: subdirs clean

all: $(TARGET)

test.o: $(HEADERS)

%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ -c $<

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<

$(TARGET): $(LIBOBJ)
	$(AR) rc $@ $^
	$(RANLIB) $@

clean:
	rm -f $(TARGET) $(LIBOBJ)

