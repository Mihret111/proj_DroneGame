CC = gcc
CFLAGS = -Wall -Wextra -g -Iheaders
LDFLAGS = -lncurses -lm
TARGET = arp1

# Source files
SRCS = main.c server.c dynamics.c keyboard.c obstacles.c targets.c params.c util.c

# Object files
OBJS = $(SRCS:.c=.o)

# Default target
.PHONY: all
all: $(TARGET)

# Link the executable
$(TARGET): $(OBJS)
	$(CC) $(OBJS) -o $(TARGET) $(LDFLAGS)

# Compile source files into object files
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Clean up build artifacts
.PHONY: clean
clean:
	rm -f $(OBJS) $(TARGET)

# Run the application
.PHONY: run
run: $(TARGET)
	./$(TARGET)

# Help target
.PHONY: help
help:
	@echo "Makefile for $(TARGET)"
	@echo "Usage:"
	@echo "  make        Build the executable"
	@echo "  make clean  Remove object files and executable"
	@echo "  make run    Build and run the program"
	@echo "  make help   Show this help message"
