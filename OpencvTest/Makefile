TARGET = OCVTest 
OBJECTS = main.o marker.o decode.o calib.o
CXXFLAGS = `pkg-config --cflags opencv` -O3 -DDESKTOP
LDLIBS = `pkg-config --libs opencv`

all: $(TARGET)

$(TARGET): $(OBJECTS)
	g++ -o $(TARGET) $(OBJECTS) $(CXXFLAGS) $(LDLIBS)

%o: %cpp

clean:
	$(RM) $(TARGET) $(OBJECTS)
