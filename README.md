# Terminal Software Renderer
A WIP software renderer for terminals. Requires a terminal with true color (24 bit colors)

# Building - (CMake, C++20 compiler)
## Windows

    git clone https://github.com/Spacerulerwill/Terminal-Software-Renderer.git --recursive
    cd Terminal-Software-Renderer/tools/windows
    configure.bat
    cd ../../build
    TerminalSoftwareRenderer.sln

Now you can compile it like any other visual studio project

## Linux / MacOS

    git clone https://github.com/Spacerulerwill/Terminal-Software-Renderer.git --recursive
    cd Terminal-Software-Renderer/tools/linux

From here you can either run `configure_debug.sh` or `configure_release.sh`

    ./configure_debug.sh
    ./build.sh

And then to run the program

    ./run.sh
