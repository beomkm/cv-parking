CC=clang++
CFLAGS=`pkg-config --cflags opencv`
LIBS=`pkg-config --libs opencv`
TARGET=parking.out
SRCS=parking.cpp

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) -o $@ $(SRCS) $(CFLAGS) $(LIBS)

clean:
	rm *.out

