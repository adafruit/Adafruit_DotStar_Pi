from distutils.core import setup, Extension
setup(name='dotstar', version='0.2', ext_modules=[Extension('dotstar', ['dotstar.c'], include_dirs=['/opt/vc/include'])])
