all: simpleCurrentControl.o 
	cc simpleCurrentControl.o  -o simpleCurrentControl

simpleCurrentControl.o: CurrentControl/simpleCurrentControl.c
	cc -c CurrentControl/simpleCurrentControl.c -o simpleCurrentControl.o

.PHONY: clean
clean:
	rm -f *.o simpleCurrentControl
