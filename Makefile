CXX = g++
CXXFLAGS = -Iinclude -g -DGL_GLEXT_PROTOTYPES -Iglut-3.7.6-bin -O2
LDFLAGS = -lm -lGL -lglut -lGLU

ifeq ($(shell sw_vers 2>/dev/null | grep Mac | awk '{ print $$2}'),Mac)
	CFLAGS = -g -DGL_GLEXT_PROTOTYPES -I./include/ -I/usr/X11/include -DOSX -I ./eigen/
	LDFLAGS = -framework GLUT -framework OpenGL \
    	-L"/System/Library/Frameworks/OpenGL.framework/Libraries" \
    	-lGL -lGLU -lm -lstdc++
endif

SRCDIR = src
	
all: $(SRCDIR)/main.o
	$(CXX) $(CXXFLAGS) -o invkin $(SRCDIR)/main.o $(LDFLAGS)

$(SRCDIR)/test.o:
	$(CXX) -Iinclude -g -O2 $(SRC) -c -o $(SRCDIR)/test.o $(SRCDIR)/test.cc

.PHONY: clean check

clean:
	rm $(SRCDIR)/*.o invkin test

check: $(SRCDIR)/test.o
	$(CXX) -Iinclude -g -O2 -o test $(SRCDIR)/test.o;
	./test
