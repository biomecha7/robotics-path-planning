# Compiler and flags
CXX = clang++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -Iinclude `pkg-config --cflags opencv4`
LDFLAGS = `pkg-config --libs opencv4`

# Folders
SRC_DIR = src
BUILD_DIR = build

# Output binary
TARGET = $(BUILD_DIR)/main

# Source and object files
SRCS = $(shell find $(SRC_DIR) -name '*.cpp')
OBJS = $(SRCS:$(SRC_DIR)/%.cpp=$(BUILD_DIR)/%.o)

# Default target
all: prepare $(TARGET)

# Link object files into the final binary
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

# Compile .cpp files into .o object files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Ensure build & image directory exists
prepare:
	mkdir -p $(BUILD_DIR)
	mkdir -p img

# Clean up
clean:
	rm -rf $(BUILD_DIR) img/*.png

# Optional: Run the program
run: all
	./$(TARGET)

