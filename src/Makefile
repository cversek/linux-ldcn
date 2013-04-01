all: Ldcn.c Ldcn.h Ldcn.o Ldcn.so
Ldcn.o: Ldcn.c Ldcn.h
	gcc -Wall -fPIC -c Ldcn.c
Ldcn.so: Ldcn.o
	gcc -shared -Wl,-soname,lidLdcn.so -o libLdcn.so Ldcn.o
	mv libLdcn.so Ldcn
test: all
	python test_ldcn.py
clean:
	rm -f *.o
	rm -f *~
