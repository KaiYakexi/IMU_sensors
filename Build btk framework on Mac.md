**

# Build btk framework on Mac

**
## btk
BTK is an open-source and cross-platform library for biomechanical analysis. BTK read and write acquisition files and can modify them. All these operations can be done by the use of the C++ API or by the wrappers included (Matlab, Octave, and Python).      https://code.google.com/archive/p/b-tk

### btk Building instruction on Mac
This section is only necessary if you downloaded the sources of BTK. If you use a binary version, go directly to the tutorial Getting Started.To use the BTK bindings into Python, we need to download the following softwares
#### Prerequisite:  
* CMake 2.6.4 or later to generate the building files;
* Python 2.7 or later (note that Python 3.0 or later is not yet officially supported)
* SWIG (2.0 or later) to create the Python bindings;
* NumPy (1.6 or later) used by the Python bindings to retrieve native data;
* A compiler compatible with Python:
                  * GCC for Linux / MacOS X ;
                  * Microsoft Visual Studio C++ (Express Edition) for Windows XP/Vista/7 ;

With CMake or CMake-gui, we need only to activate the option **BTK_WRAP_PYTHON**. CMake will then try to find Python, SWIG and NumPy. For more details on the options of BTK available during the compilation, you can check the file README.html located in the source of BTK. 
(http://biomechanical-toolkit.github.io/docs/Wrapping/Python/_build_instructions.html)
(https://biomechanical-toolkit.github.io/docs/API/_build_instructions.html)

#### download btk library:![在这里插入图片描述](https://img-blog.csdnimg.cn/20200105071245569.png)### activate BTK_WRAP_PYTHON option.
1) Create a new folder named “build” in the extracted core file.
2) Fill in the path of core file and build folder in two spaces of CMake main interface. Take me example:
3) where is the source code:     /Users/shengkaimao/Desktop/btk-core_src
4) where to build the binaries:     /Users/shengkaimao/Desktop/btk-core_src/build
5) When click Configure, specify the generator for this project, select UNIX makefiles-Use default native compilers
6) Chose btk_wrap_python option, configure again

+ An error came out:
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200105071300466.png)

+ Error description:

`CMake Error at /Applications/CMake.app/Contents/share/cmake-3.16/Modules/FindPackageHandleStandardArgs.cmake:146 (message):
  Could NOT find NumPy (missing: NUMPY_VERSION NUMPY_INCLUDE_DIR)`

+ Solution:

Modify this file  ：/Users/shengkaimao/Desktop/btk-core-0.3.0_src/CMake/FindNumPy.cmake
by adding the following two redline codes:
`INCLUDE(FindPackageHandleStandardArgs)set(NUMPY_VERSION 1.18.0)
set(NUMPY_INCLUDE_DIR”/Library/Frameworks/Python.framework/Versions/3.7/lib/python3.7/site-packages/numpy”)`

-do it all over again
-condigure-generate
-Open the terminal, use cd command into build, make install as follows:

```
$ cd /Users/shengkaimao/Desktop/btk-core-0.3.0_src/build
$ rm -rf*
$ cmake ..
$ make -j
$ make install
```

Finally, six files will be obtained in the folder named bin. The file name with the suffix ". a" appears to indicate that binary is successful.
![在这里插入图片描述](https://img-blog.csdnimg.cn/2020010507132820.png)
Put all the files in Bin folder into directory (/Library/Frameworks/Python.framework/Versions/3.7/lib/python3.7/site-packages/ ), and name it Btk. Then select Btk interpreter in pycharm to import.
