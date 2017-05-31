LAB_1_OBJ = obj/Lab1_Main.o
LAB_2_OBJ = obj/Lab2_Main.o
LAB_3_OBJ = obj/Lab3_Main.o

INCLUDES = -I./inc/ -I/home/fox/armadillo/usr/include
LIBS = -L/home/fox/armadillo/usr/lib -larmadillo


bin3: bin/lab3
bin2: bin/lab2
bin: bin/lab1

# Here the executables are made
bin/lab1 : $(LAB_1_OBJ)
	g++ -Wall -o bin/lab1 $(LAB_1_OBJ)

bin/lab2 : $(LAB_2_OBJ)
	g++ -Wall -o bin/lab2 $(LAB_2_OBJ)

bin/lab3 : $(LAB_3_OBJ)
	g++ -Wall -o bin/lab3 $(LAB_3_OBJ) $(LIBS)

# Here all the .o files are made
obj/Lab1_Main.o: src/Lab1_Main.cpp
	g++ $(INCLUDES) -Wall -c -o obj/Lab1_Main.o src/Lab1_Main.cpp

obj/Lab2_Main.o: src/Lab2_Main.cpp
	g++ $(INCLUDES) -Wall -c -o obj/Lab2_Main.o src/Lab2_Main.cpp

obj/Lab3_Main.o: src/Lab3_Main.cpp
	g++ $(INCLUDES) -Wall -c -o obj/Lab3_Main.o src/Lab3_Main.cpp

# Remove all generated files
clean:
	- rm -f obj/*
	- rm -f bin/*
	- rm -f plot/*
	- rm -f *.txt
