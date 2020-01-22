CXX_FLAGS=-std=c++11 -O2 -Wall -Wextra

INCLUDE_FLAGS=`pkg-config --cflags ompl eigen3`
# Linker options
LD_FLAGS=`pkg-config --libs ompl`

# The c++ compiler to invoke
CXX=c++
all: Project5Exercise

clean:
	rm -f *.o
	rm -f Project5Exercise

%.o: src/%.cpp
	$(CXX) -c $(CXX_FLAGS) $(INCLUDE_FLAGS) $< -o $@

Project5Exercise: Project5Exercise.o CollisionChecking.o DRRT.o
	$(CXX) $(CXX_FLAGS) $(INCLUDE_FLAGS) -o $@ $^ $(LD_FLAGS)

