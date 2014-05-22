all: wrapper

wrapper: projecton_wrapper.o ImageProcessing.o
	g++ ImageProcessing.o projecton_wrapper.o -o wrapper `pkg-config --cflags --libs opencv`

projecton_wrapper.o: projecton_wrapper.cpp ImageProcessing.h wrapperFunctions.h
	g++ -I mavlink/v1.0 -c projecton_wrapper.cpp `pkg-config --cflags --libs opencv`

ImageProcessing.o: ImageProcessing.h ImageProcessing.cpp
	g++ -c ImageProcessing.cpp `pkg-config --cflags --libs opencv`

clean:
	rm -f *o wrapper
