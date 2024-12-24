# Compile options
CC=g++
CCV=-std=c++23
INC=-I./include
DTYPES="FIXED(32, 16),FAST_FIXED(32, 16), FIXED(64, 32)"
DSIZES="S(36, 84)"

# Run options
PT="FIXED(64, 32)"
VT="FIXED(64, 32)"
VFT="FAST_FIXED(32, 16)"
FILE="field2.txt"

all:
	$(CC) $(CCV) $(INC) -o fluid.out fluid.cpp -DTYPES=$(DTYPES) -DSIZES=$(DSIZES)

run:
	./fluid.out --p-type=$(PT) --v-type=$(VT) --v-flow-type=$(VFT) --file-name=$(FILE)

build_and_run: all run

clean:
	rm -f *.out