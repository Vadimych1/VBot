#!/usr/bin/env python

'''
setup.py - Python distutils setup file for BreezySLAM package.

Copyright (C) 2014 Simon D. Levy

This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.

This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http://www.gnu.org/licenses/>.
'''

from platform import machine
import shutil

ok = False
for x in ["breezyslam/build", "build", "vslam/breezyslam/build"]:
    try:
        shutil.rmtree(x)
        ok = True
        break
    except: pass
if not ok:
    print("cd to 'vslam'")
    quit()

OPT_FLAGS  = []
SIMD_FLAGS = []

arch = machine()

print(arch)

if  arch in ['i686', 'x86_64']:
    SIMD_FLAGS = ['-msse3']
    arch = 'i686'

elif arch == 'armv7l':
    OPT_FLAGS = ['-O3']
    SIMD_FLAGS = ['-mfpu=neon']

else:
    arch = 'sisd'

SOURCES = [
    'pybreezyslam.c', 
    'pyextension_utils.c', 
    'coreslam.c', 
    'coreslam_' + arch + '.c',
    'random.c',
    'ziggurat.c'
]

from distutils.core import setup, Extension

module = Extension('miniros_breezyslam_cpack', 
    sources = SOURCES, 
    extra_compile_args = ['-std=gnu99', '-Iinclude'] + SIMD_FLAGS + OPT_FLAGS,
)


setup (name = 'BreezySLAM',
    version = '1.0.0',
    description = 'Simple, efficient SLAM in Python',
    packages = ['miniros_breezyslam'],
    ext_modules = [module],
    author='Simon D. Levy and Suraj Bajracharya',
    author_email='simon.d.levy@gmail.com',
    url='https://github.com/simondlevy/BreezySLAM',
    license='LGPL',
    platforms='Linux; Windows; OS X',
    long_description = 'Provides core classes Position, Map, Laser, Scan, and algorithm CoreSLAM'
)