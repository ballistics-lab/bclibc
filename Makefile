.PHONY: all clean build ffi core

all: build

build:
	mkdir -p build
	cd build && cmake -DCMAKE_BUILD_TYPE=Release ..
	cd build && make -j$(nproc)

core:
	mkdir -p build
	cd build && cmake -DCMAKE_BUILD_TYPE=Release ..
	cd build && make bclibc_core -j$(nproc)

ffi:
	mkdir -p build
	cd build && cmake -DCMAKE_BUILD_TYPE=Release ..
	cd build && make bclibc_ffi -j$(nproc)

clean:
	rm -rf build