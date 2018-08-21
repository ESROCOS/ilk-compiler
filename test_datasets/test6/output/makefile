CC=g++
LDFLAGS=-Wl,-R,./

eigen_cflags:= ${shell pkg-config --cflags eigen3}
    CFLAGS=-Wall -pedantic -std=c++11 ${eigen_cflags} -O2 -DEIGEN_NO_DEBUG
CFLAGS_DBG=-Wall -pedantic -std=c++11 ${eigen_cflags} -O0 -g -ftest-coverage -fprofile-arcs


LIB=libur5.so

.PHONY: clean

all: $(LIB)

$(LIB) : ur5.cpp
	$(CC) $(CFLAGS) -fPIC -shared -o $@ $^ -lilkeigenbackend

tests :  ur5_fk1_test ur5_fk1_timing ur5_fk2_test ur5_fk2_timing ur5_fk__ik1_test ur5_fk__ik1_timing ur5_ik1_test ur5_ik1_timing ur5_ik2_test ur5_ik2_timing
tests_dbg :  ur5_fk1_test_dbg ur5_fk1_timing_dbg ur5_fk2_test_dbg ur5_fk2_timing_dbg ur5_fk__ik1_test_dbg ur5_fk__ik1_timing_dbg ur5_ik1_test_dbg ur5_ik1_timing_dbg ur5_ik2_test_dbg ur5_ik2_timing_dbg

ur5_fk1_test : ur5_fk1_test.cpp $(LIB)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIB) -lilkeigenbackend
ur5_fk1_timing : ur5_fk1_timing.cpp $(LIB)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIB) -lilkeigenbackend
ur5_fk2_test : ur5_fk2_test.cpp $(LIB)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIB) -lilkeigenbackend
ur5_fk2_timing : ur5_fk2_timing.cpp $(LIB)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIB) -lilkeigenbackend
ur5_fk__ik1_test : ur5_fk__ik1_test.cpp $(LIB)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIB) -lilkeigenbackend
ur5_fk__ik1_timing : ur5_fk__ik1_timing.cpp $(LIB)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIB) -lilkeigenbackend
ur5_ik1_test : ur5_ik1_test.cpp $(LIB)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIB) -lilkeigenbackend
ur5_ik1_timing : ur5_ik1_timing.cpp $(LIB)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIB) -lilkeigenbackend
ur5_ik2_test : ur5_ik2_test.cpp $(LIB)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIB) -lilkeigenbackend
ur5_ik2_timing : ur5_ik2_timing.cpp $(LIB)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIB) -lilkeigenbackend

ur5_fk1_test_dbg : ur5_fk1_test.cpp ur5.cpp
	$(CC) $(CFLAGS_DBG) -o $@ $^ -lilkeigenbackend
ur5_fk1_timing_dbg : ur5_fk1_timing.cpp ur5.cpp
	$(CC) $(CFLAGS_DBG) -o $@ $^ -lilkeigenbackend
ur5_fk2_test_dbg : ur5_fk2_test.cpp ur5.cpp
	$(CC) $(CFLAGS_DBG) -o $@ $^ -lilkeigenbackend
ur5_fk2_timing_dbg : ur5_fk2_timing.cpp ur5.cpp
	$(CC) $(CFLAGS_DBG) -o $@ $^ -lilkeigenbackend
ur5_fk__ik1_test_dbg : ur5_fk__ik1_test.cpp ur5.cpp
	$(CC) $(CFLAGS_DBG) -o $@ $^ -lilkeigenbackend
ur5_fk__ik1_timing_dbg : ur5_fk__ik1_timing.cpp ur5.cpp
	$(CC) $(CFLAGS_DBG) -o $@ $^ -lilkeigenbackend
ur5_ik1_test_dbg : ur5_ik1_test.cpp ur5.cpp
	$(CC) $(CFLAGS_DBG) -o $@ $^ -lilkeigenbackend
ur5_ik1_timing_dbg : ur5_ik1_timing.cpp ur5.cpp
	$(CC) $(CFLAGS_DBG) -o $@ $^ -lilkeigenbackend
ur5_ik2_test_dbg : ur5_ik2_test.cpp ur5.cpp
	$(CC) $(CFLAGS_DBG) -o $@ $^ -lilkeigenbackend
ur5_ik2_timing_dbg : ur5_ik2_timing.cpp ur5.cpp
	$(CC) $(CFLAGS_DBG) -o $@ $^ -lilkeigenbackend

clean:
	rm -f $(LIB)
