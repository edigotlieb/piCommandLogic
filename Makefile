all: projecton_wrapper
	
mavlink_test: mavlink_test.o
	g++ projecton_wrapper.o -o projecton_wrapper

mavlink_test.o: mavlink_test.cpp
	g++ -I mavlink/v1.0 -c projecton_wrapper.cpp

clean:
	rm -rf *o projecton_wrapper