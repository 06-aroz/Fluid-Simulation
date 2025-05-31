CXX = g++
CXXFLAGS = -std=c++20
LDFLAGS = -framework OpenGL -framework GLUT
TARGET = main
SRC = main.cpp

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET) $(LDFLAGS)

clean:
	rm -f $(TARGET)

run: $(TARGET)
	./$(TARGET)


