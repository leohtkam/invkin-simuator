CXX = g++
CXXFLAGS = -Iinclude -g -DGL_GLEXT_PROTOTYPES -Iglut-3.7.6-bin -O2
LDFLAGS = -lm -lGL -lglut -lGLU

SRCDIR = src
	
all: $(SRCDIR)/main.o
	$(CXX) $(CXXFLAGS) -o invkin $(SRCDIR)/main.o $(LDFLAGS)



.PHONY: clean check

clean:
	rm $(SRCDIR)/*.o invkin