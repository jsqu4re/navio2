CXX ?= g++
NAVIO = ../../Navio
INCLUDES = -I ../../Navio

all:
	$(MAKE) -C ../../Navio all
	$(CXX) -c pid.cpp -o pid.o
	$(CXX) -std=c++11 $(INCLUDES) balance.cpp pid.o -L$(NAVIO) -lnavio -o BALANCE -lrt -lpthread

clean:
	rm -f pid.o
	rm -f BALANCE
