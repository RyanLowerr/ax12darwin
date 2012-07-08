# UpMech AX-12 Monitor makefile

TARGET = axmonitor
CC = g++
INCLUDE_DIRS = -I../../framework/hwinterface -I../../framework/math -I../../framework/movement -I../../framework/vision -I../../framework/platform
BUILD_DIR = ../../build
LIBRARY_DIR = ../../library

all: upmech.a $(TARGET)

upmech.a:
	make -C $(BUILD_DIR)

$(TARGET): $(TARGET).cpp $(LIBRARY_DIR)/upmech.a
	$(CC) -c $(TARGET).cpp $(INCLUDE_DIRS)
	$(CC) -o $(TARGET) $(TARGET).o $(LIBRARY_DIR)/upmech.a $(INCLUDE_DIRS)
	rm $(TARGET).o
	
clean:
	rm *.o $(TARGET) 
