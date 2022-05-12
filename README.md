HaxBall Group-5
===================================

Course project for ARL 2021.
The goal is to train a reinforcement learning agent to play Haxball (a simplified football simulation), using different algorithms seen in class.


Some instructions
-----------------

After cloning you have to add the submodule to get the HaxBall code:

```console
git submodule update --init
```

The Makefile is just a shortcut to manage and test all projects at once.

Use the target 'compile' for compiling everything:

```console
make compile -j 8
```

This target is just a shortcut for the normal procedure:

```console
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j 8
```

Hopefully, you can start now the agent in the build directory:

```console
./HaxBallAgentGroup-5
```

Cleaning removes the build and documentation directories (and everything in there!):

```console
make clean
```

Use e.g. QtCreator to open the full project:

```console
qtcreator CMakeLists.txt
```

And a small warning: The other targets in the Makefile handle the initial commit and uploading the repository to our gitlab server. If you want to mess up your project run them again ;)

Qt and QtCreator, CMake
-----------------------

On Mac and Linux you can most likely use your paket manager for all three.

For Windows, download from the (Qt Website)[https://www.qt.io/offline-installers] the offline installer with the latest Qt 5.15.x version (matching the EIKON). Disable during the installation your internet to avoid creating a Qt Account. During the setup select also MinGW to get a running C++ Compiler on your computer. Check the QtCreator to instlal the IDE.

CMake is available at (Kitware's Website)[https://cmake.org/download/], just install it.

Now, you can start Qtcreator and open directly the CMakeLists.txt as C++ project.
