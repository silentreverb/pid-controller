# pid-controller  <img alt="Travis CI Build Status" src="https://travis-ci.org/silentreverb/pid-controller.svg"/> <a href="https://scan.coverity.com/projects/silentreverb-pid-controller"><img alt="Coverity Scan Build Status" src="https://scan.coverity.com/projects/6254/badge.svg"/></a>

A simple C++-based PID controller.

## Installation
This library utilizes the `cmake` cross-platform utility. To build and install the library on UNIX-like systems, execute the following steps in a new terminal.

Create the build directory:
```
mkdir build
cd build
```
Generate build scripts:
```
cmake ..
```

Build and install the library:
```
make
sudo make install
```
