
planContact: planContact.o 
	g++ -O planContact.o  -o planContact -Wl,-rpath ${SNOPT}/lib -L${SNOPT}/lib -lsnopt7_cpp -lsnopt7

planContact.o: planContact.cpp
	g++ -O -I${SNOPT}/include -c planContact.cpp -o planContact.o

all: planContact

clean: 
	rm planContact.o planContact

.DEFAULT_TARGET: all
