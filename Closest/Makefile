CC = gcc
CFLAGS = -Wall -g
LDFLAGS = -lm
SRC = generate_points.c serial_closest.c parallel_closest.c utilities_closest.c closest.c
OBJ = $(SRC:.c=.o)
EXEC = closest generate_points

all: $(EXEC)

closest: closest.o serial_closest.o parallel_closest.o utilities_closest.o
	$(CC) -o $@ $^ $(LDFLAGS)

generate_points: generate_points.o
	$(CC) -o $@ $<

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(EXEC)
