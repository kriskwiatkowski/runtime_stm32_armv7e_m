# Runtime for stm32f4 platform

## Building

```
cmake -S cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=toolchains/gcc-armv7e_m-unknown-none-eabi.toolchain
cmake --build build
cmake --build build --target install
```

Libraries will be compiled and placed in the ``build/pack`` directory.