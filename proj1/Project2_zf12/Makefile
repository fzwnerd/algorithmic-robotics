# Change -O2 to -g when you are debugging your code
CXXFLAGS      = -std=c++11 -O2

# Include directories
INCLUDE_FLAGS = -Isrc

# Linker options and libraries
LD_FLAGS      = -lm

# The C++ compiler to invoke (must have support for C++11)
CXX           = c++

PROGRAM       = Project2
SRC_DIR       = src

# The Makefile will compile all *.cpp files in SRC_DIR
SOURCES       = $(wildcard $(SRC_DIR)/*.cpp)
OBJECTS       = $(patsubst %.cpp, %.o, $(SOURCES))

.PHONY : all
all : $(PROGRAM)

# Remove everything (object files in SRC_DIR, the compiled PROGRAM)
clean :
	$(RM) $(SRC_DIR)/*.o
	$(RM) $(PROGRAM)

# Compile all *.cpp files to object files
%.o : %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDE_FLAGS) -c $< -o $@

# Compile the object files into the main program
$(PROGRAM) : $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LD_FLAGS)
