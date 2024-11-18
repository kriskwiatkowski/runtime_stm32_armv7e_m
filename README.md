# Runtime for stm32f4 platform

Creates runtime environment that allows running apps on STM32F4. One needs to build the solution, link resulting static library with the application and push to the board. Solution contains implementation of `printf` that sends data over UART to the terminal.

## Building

```
cmake -S cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=toolchains/gcc-armv7e_m-unknown-none-eabi.toolchain
cmake --build build
cmake --build build --target install
```

Libraries will be compiled and placed in the ``build/pack`` directory.
