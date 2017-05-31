LAB_1_OBJ = obj/Lab1_Main.o
LAB_2_OBJ = obj/Lab2_Main.o

bin: bin/lab1
bin2: bin/lab2

# Here the executables are made
bin/lab1 : $(LAB_1_OBJ)
	g++ -Wall -o bin/lab1 $(LAB_1_OBJ)

bin/lab2 : $(LAB_2_OBJ)
	g++ -Wall -o bin/lab2 $(LAB_2_OBJ)

# Here all the .o files are made
obj/Lab1_Main.o: src/Lab1_Main.cpp
	g++ -Wall -c -o obj/Lab1_Main.o src/Lab1_Main.cpp -I inc

obj/Lab2_Main.o: src/Lab2_Main.cpp
	g++ -Wall -c -o obj/Lab2_Main.o src/Lab2_Main.cpp -I inc

# Remove all generated files
clean:
	- rm -f obj/*
	- rm -f bin/*
	- rm -f lib/*
