from distutils.core import setup, Extension
setup(name='dotstar', version='0.1', ext_modules=[Extension('dotstar', ['dotstar.c'])])
