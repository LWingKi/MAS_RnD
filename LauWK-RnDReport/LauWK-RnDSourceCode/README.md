## MAS_RnD
The documentation of RnD project

# Kinova API example Build instruction:
1. in terminal, input:
```
cd kortex/api_cpp/examples
```

2. then type: (Choose debug)
```
./scripts/build-gcc.sh {release|debug}
```
# Kinova API run instruction
1. make sure the device is connected to a static ip port (eg:192.168.1.11/24)
2. in terminal, at ./example directory, input:
```
./<your_build_folder>/<example_name>
```


# Making each external source as dependency:

## In each EXTERNAL SOURCE:
During Build: 
1. cmake -DCMAKE_INSTALL_PREFIX="/"path to instal"/install" ..  (The path can be anywhere of the install folder)
2. make
3. make install

To the main (or the folder that you need to use the external source as dependency)'s build folder:
1. cmake -DCMAKE_INSTALL_PREFIX="/"path to instal"/install" ..
2. make
3. make install
(make -j4 to parallel build the source)

Issues about orocos:
In orocoskdl/CMakeLists.txt , line 113:
when you run "make" to build the library, it will create a PACKAGE at:  ~/.cmake/packages
In package folder, all the installed package will be shown as folder, inside each folder there is file with random numbers, use command: cat "the number" to know what project is using the package
may need to delete it to do a clean build neccessory


## Build kdl_parser:
1. cmake -DCMAKE_INSTALL_PREFIX="/"path to install"/install" ..  (The path can be anywhere of the install folder)
2. make
3. make install

2. in TARGET_LINK_LIBRARIES: change kdl_parser to ${KDL_PARSER} or add ${KDL_PARSER} if you have it


## Build robif2b:
Robif2b require kortexapi since it is a wrapper, in robif2b/cmake/FindKortexApi.cmake:
edit the include directory to: "${CMAKE_INSTALL_FULL_INCLUDEDIR}/kortex_api"  (so as all the include directories)
edit the include library to:   "${CMAKE_INSTALL_FULL_LIBDIR}/libKortexApiCpp.a" (so as all the include libraries)

Copy all the file/folders mentioned in the FindKortexApi.cmake to :
For libraries: install/lib/
For directories: install/include/kortex_api

The reason of doing the above steps is kortex_api has no build files 

1. cmake -DCMAKE_INSTALL_PREFIX="/"path to install"/install" -DENABLE_KORTEX=ON ..  (from robif2b/src/nbx/CMakeLists.txt)
2. ccmake .. to confirm the install directory
3. make
4. make install

In CMakeLists.cmake of the main folder:
1. add: find_library(KDL_PARSER kdl_parser)
2. in TARGET_LINK_LIBRARIES: add robif2b::kinova_gen3 (project::namespace)
	can check the name of the namespace from : robif2b/src/nbx/kortex/CMakeLists.txt
	(You can add others such as kelo, just check "robif2b/src/nbx/(what you want to add)"
	the project::namespace structure can refer to robif2b/CMakeLists.txt , line 69-72



To make sure it is a cleam build:
Go to the following directories:
/usr/local/share
/usr/local/include
/usr/local/share/pkgconfig

"package".pc this help find the package in system

for example if we want to clean eigen3, make sure everything about eigen3 in these directories are deleted

After installing all dependencies and build the library successfullt. To run the code:
1. Go to project_directory/build
2. make
3. ./(any execution file)