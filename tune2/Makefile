CC=gcc

SRC=diseqc.c tune-s2.c
HED=diseqc.h tune-s2.h
OBJ=diseqc.o tune-s2.o

BIND=/usr/local/bin/
INCLUDE=-I../s2/linux/include

TARGET=tune-s2

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(CFLG) $(OBJ) -o $(TARGET) $(CLIB) -lm

$(OBJ): $(HED)

install: all
	cp $(TARGET) $(BIND)

uninstall:
	rm $(BIND)$(TARGET)

clean:
	rm -f $(OBJ) $(TARGET) *~

%.o: %.c
	$(CC) $(INCLUDE) -c $< -o $@
