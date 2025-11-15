To build and test, do the following:
```
mkdir build
cd build
cmake .. -DCMAKE_PREFIX_PATH=/path/to/drake/installation
cmake --build . --target _iiwa_ik
cd ..
python3 test/test.py
```