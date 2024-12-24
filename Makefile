# Compile options
CC=g++
CCV=-std=c++23
INC=-I./include
TYPES=-DTYPES=""
SIZES=-DSIZES="S(36, 84)"

# Run options
PT=--p-type="FIXED(64, 32)"
VT=--v-type="FIXED(64, 32)"
VFT=--v-flow-type="FIXED(64, 32)"
FILE=--file-name="field2.txt"

all:
	$(CC) $(CCV) $(INC) -o fluid.out fluid.cpp $(TYPES) $(SIZES)

run:
	./fluid.out $(PT) $(VT) $(VFT) $(FILE)

build_and_run: all run

clean:
	rm -f *.out