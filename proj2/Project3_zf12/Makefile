CXX_FLAGS=-std=c++11 -O2 -Wall -Wextra

INCLUDE_FLAGS=`pkg-config --cflags ompl eigen3`
# Linker options
LD_FLAGS=`pkg-config --libs ompl`

# The c++ compiler to invoke
CXX=c++
all: Project3Exercise2 Project3Exercise3

clean:
	rm -f *.o
	rm -f Project3Exercise2 Project3Exercise3

%.o: src/%.cpp
	$(CXX) -c $(CXX_FLAGS) $(INCLUDE_FLAGS) $< -o $@

Project3Exercise2: Project3Exercise2.o CollisionChecking.o RTP.o
	$(CXX) $(CXX_FLAGS) $(INCLUDE_FLAGS) -o $@ $^ $(LD_FLAGS)

Project3Exercise3: Project3Exercise3.o RTP.o
	$(CXX) $(CXX_FLAGS) $(INCLUDE_FLAGS) -o $@ $^ $(LD_FLAGS)
