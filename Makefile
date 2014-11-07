# This is needed only for *compiling* the dotstar.so file (Python module).
# Not needed for just running the Python code with a precompiled .so file.

all: dotstar.so

dotstar.so: dotstar.o
	gcc -s -shared -Wl,-soname,libdotstar.so -o $@ $<

.c.o:
	gcc -fPIC -O3 -fomit-frame-pointer -funroll-loops -c $<

clean:
	rm -f dotstar.o dotstar.so
