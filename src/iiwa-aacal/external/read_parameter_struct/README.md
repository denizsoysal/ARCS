# ReadParameterStructC

Template package for reading and storing data from input file to a generic parameters struct in c/cpp.

## Dependencies
- cJSON
```
[git clone https://github.com/AprilRobotics/apriltag.git](https://github.com/DaveGamble/cJSON.git)
```

## Usage 
Link ```readfile``` library in CMake.


## Example
Running the CMakeFile generates an example program from the [example](example) folder. 
The template files [test_struct.c](src/test_struct.c) and [test_struct.h](inlude/test_struct.h) are self explanatory.

```console
mkdir build
```
```console
cd build
```
```console
cmake ..
```
```console
make
```
