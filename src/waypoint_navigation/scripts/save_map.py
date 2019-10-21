#!/usr/bin/env python
from __future__ import print_function

from os import path
from subprocess import call
from sys import exit

result = call(['rosservice call orb_slam2_rgbd/save_map --wait "name: map.bin"'], shell=True)
if result != 0:
    exit(result)


source = path.expanduser('~/.ros/map.bin')
dest = path.realpath(path.join(path.abspath(path.dirname(__file__)), '..', 'maps', 'orbslam', 'map.bin'))
print('Copying {} -> {}'.format(source, dest))
call(['cp {} {}'.format(source, dest)], shell=True)
