CC = g++
ifeq ($(shell sw_vers 2>/dev/null | grep Mac | awk '{ print $$2}'),Mac)
	CFLAGS = -g -DGL_GLEXT_PROTOTYPES -I./include/ -I/usr/X11/include -DOSX
	LDFLAGS = -framework GLUT -framework OpenGL \
    	-L"/System/Library/Frameworks/OpenGL.framework/Libraries" \
    	-lGL -lGLU -lm -lstdc++
else
	CFLAGS = -g -DGL_GLEXT_PROTOTYPES -I./glut-3.7.6-bin
	LDFLAGS = -lglut -lGLU -lGL -lm
endif
	
RM = /bin/rm -f 
all: main 
main: clothsim.o 
	$(CC) $(CFLAGS) -o clothsim clothsim.o $(LDFLAGS) 
clothsim.o: clothsim.cpp
	$(CC) $(CFLAGS) -c clothsim.cpp -o clothsim.o
clean: 
	$(RM) *.o clothsim
 
