CC     = gcc
CFLAGS = -std=c11 -g -Wall -Wextra -I../main

SRC    = ../main/wspr_encode.c test_wspr_encode.c
TARGET = test_wspr_encode

all: $(TARGET)
	./$(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $@ $(SRC)

clean:
	rm -f $(TARGET) $(TARGET).exe

.PHONY: all clean
