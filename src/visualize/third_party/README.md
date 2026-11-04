matplotlib-cpp is vendored from the existing local third-party checkout at
`/home/ld/pure-nav/src/thirdparty/matplotlib-cpp`. Its MIT license is preserved.
The subplot wrapper passes Python integers (rather than floats) for row/column/
subplot indices, as required by current Matplotlib. The build uses WITHOUT_NUMPY
and the Python sequence interface, avoiding a NumPy C ABI dependency.
