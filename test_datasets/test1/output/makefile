CC=g++
LDFLAGS=-Wl,-R,./

eigen_cflags:= ${shell pkg-config --cflags eigen3}
CFLAGS=-Wall -pedantic -std=c++11 ${eigen_cflags} -O2 -DEIGEN_NO_DEBUG

LIB=libur5.so

.PHONY: clean

all: ur5 $(LIB)

$(LIB) : ur5.cpp
	$(CC) $(CFLAGS) -fPIC -shared -o $@ $^ -lilkeigenbackend

ur5: $(LIB)
	$(CC) $(CFLAGS) $(LDFLAGS) -o ur5 ur5_test.cpp $(LIB)

clean:
	rm -f ur5 $(LIB)
