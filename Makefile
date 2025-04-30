# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++17 -w -O2

# Source files (excluding .ino files)
SOURCES = $(wildcard *.cpp)
HEADERS = $(wildcard *.hpp)

# Object files
OBJECTS = $(SOURCES:.cpp=.o)

# Output executable
TARGET = guidance_system

# Default target
all: $(TARGET)

# Link object files to create the executable
$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Compile source files into object files
%.o: %.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean up build files
clean:
	rm -f $(OBJECTS) $(TARGET)

.PHONY: all clean
