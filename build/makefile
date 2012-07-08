# Makefile for UpMech Linux Platform Framework

cc = g++
AR = ar
ARFLAGS = cr

TARGET = upmech.a

INCLUDE_DIRS = -I../framework/hwinterface -I../framework/math -I../framework/movement -I../framework/platform -I../framework/vision

SRCS = ../framework/hwinterface/AX12.cpp    \
       ../framework/math/Point.cpp          \
       ../framework/math/Vector.cpp         \
       ../framework/math/Matrix.cpp         \
       ../framework/movement/Joint.cpp      \
       ../framework/movement/Kinematics.cpp \
       ../framework/movement/Walk.cpp       \
       ../framework/movement/Head.cpp       \
       ../framework/movement/Manager.cpp    \
       ../framework/platform/Ftdi.cpp

OBJS = AX12.o       \
       Point.o      \
       Vector.o     \
       Matrix.o     \
       Joint.o      \
       Kinematics.o \
       Walk.o       \
       Head.o       \
       Manager.o    \
       Ftdi.o
       
$(TARGET):
	$(CC) $(INCLUDE_DIRS) -c $(SRCS)
	$(AR) $(ARFLAGS) ../library/$(TARGET) $(OBJS)
	rm $(OBJS) 
