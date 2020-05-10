#!/usr/bin/python
import argparse
import subprocess
import os
import re

parser = argparse.ArgumentParser()
parser.add_argument("repo",
                    help="source repo of the package to install")

args = parser.parse_args()

# get package name
package = re.search('.*\/(.+)', args.repo).group(1)
print("installing package " + package)

# go to workspace src space
os.chdir("/home/pi/catkin_ws/src")

# acquire a copy of the sources of the package that you want to build.
# For a package hosted on github, you'd do something like (where
# $organisation and $package obviously depend on what it is that you
# want to build):
print("clone repo")

subprocess.call(["git", "clone", args.repo])


# then check (and install) for any missing dependencies
# NOTE: we ask rosdep to check for ROS Melodic here,
# if you are using another ROS distribution, substitute
# that there (ie: for Kinetic, use 'kinetic', without the quotes)

print("managing dependencies")
#subprocess.call(["rosdep", "install", "--from-paths", "/home/pi/catkin_ws/src/", "--ignore-src", "--rosdistro", "melodic"])
subprocess.call(["rosdep", "install", package, "--rosdistro", "melodic"])
# now build

os.chdir("/home/pi/catkin_ws")
subprocess.call("catkin_make")
