# pid-controller  <a href="https://travis-ci.org/silentreverb/pid-controller"><img alt="Travis CI Build Status" src="https://travis-ci.org/silentreverb/pid-controller.svg?branch=master"/></a> <a href="https://scan.coverity.com/projects/silentreverb-pid-controller"><img alt="Coverity Scan Build Status" src="https://scan.coverity.com/projects/6254/badge.svg"/></a>

A simple C++-based PID controller. Requires modules from the Boost library.

## Installation
This library utilizes the `cmake` cross-platform utility. To build and install the library on Debian-based distributions, execute the following steps in a new terminal.

Install Boost dependencies, if not installed already:

```
sudo apt-get install libboost-system-dev libboost-chrono-dev libboost-timer-dev
```

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
