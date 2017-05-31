OBJ_FILES = obj/Lab1_Main.o

bin: bin/lab1

# Here the executable lab1 is made
bin/lab1 : $(OBJ_FILES)
	g++ -Wall -o bin/lab1 $(OBJ_FILES)

# Here all the .o files are made
obj/Lab1_Main.o: src/Lab1_Main.cpp
	g++ -Wall -c -o obj/Lab1_Main.o src/Lab1_Main.cpp -I inc

# Remove all generated files
clean:
	- rm -f obj/*
	- rm -f bin/*
	- rm -f lib/*
