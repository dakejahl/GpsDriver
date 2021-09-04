rm -rf build/

# pushd ${PWD}

mkdir build && cd build
cmake ..
make

# popd