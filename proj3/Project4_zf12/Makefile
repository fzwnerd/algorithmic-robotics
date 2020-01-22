CXX_FLAGS=-std=c++11 -O2 -Wall -Wextra

INCLUDE_FLAGS=`pkg-config --cflags ompl eigen3`
# Linker options
LD_FLAGS=`pkg-config --libs ompl`

# The c++ compiler to invoke
CXX=c++
all: Project4Car Project4Pendulum

clean:
	rm -f *.o
	rm -f Project4Car Project4Pendulum

%.o: src/%.cpp
	$(CXX) -c $(CXX_FLAGS) $(INCLUDE_FLAGS) $< -o $@

Project4Pendulum: Project4Pendulum.o RG-RRT.o
	$(CXX) $(CXX_FLAGS) $(INCLUDE_FLAGS) -o $@ $^ $(LD_FLAGS)

Project4Car: Project4Car.o CollisionChecking.o RG-RRT.o
	$(CXX) $(CXX_FLAGS) $(INCLUDE_FLAGS) -o $@ $^ $(LD_FLAGS)
