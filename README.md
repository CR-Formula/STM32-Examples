# Cyclone Racing Formula SAE
## Iowa State University

This Repository stores the C/C++ code for our telemetry system. The system is built off of an STM32H7 MCU. This system pulls data from the ECU and other sensors around the car and sends it in real-time to a wireless receiver, and logs it locally. The system uses FreeRTOS to schedule telemetry-related tasks as well as ARM's CMSIS files to interface with registers. Most of the example code is built for a NUCLEO test board, while the actual system runs a custom PCB with a slightly different STM32 MCU.

## Dev and Build Tools
These are the required tools to develop and build this code
- [VSCode](https://code.visualstudio.com/download) -- The IDE to write code in
- [CMake](https://cmake.org/download/) -- Another set of build tools
- [w64Dev](https://github.com/skeeto/w64devkit) -- Better Terminal
- [Arm-Toolchain](https://developer.arm.com/downloads/-/gnu-rm) -- Arm Compiler used by Make
- [Cortex Debug](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug) -- Used for debugging in VSCode
- [Keil Extension](https://marketplace.visualstudio.com/items?itemName=Arm.keil-studio-pack) -- VSCode Cortex Development Extensions
- [STM32 Programmer](https://www.st.com/en/development-tools/stm32cubeprog.html) -- STM32 Programmer Software

Install all of the above tools for your operating system and make sure to add CMake and ARM none-eabi-gcc toolchain to your PATH. Once installed, you should be able to open an SMT32 Make project in VSCode without errors. In order to build your code, open w64dev.exe and cd into the project directory. Once in the project directory, run the command make. This will create a build directory and creat the .elf file that will be used to program the board. The .elf file is used in STM32CubeProgrammer to load the code onto the boards.

# Links
- [Linktree](https://linktr.ee/cycloneracing)
- [LinkedIn](https://www.linkedin.com/company/cyclone-racing/)
- [Facebook](https://www.facebook.com/CycloneRacingUS/)
- [Twitter](https://twitter.com/cycloneracingus?lang=en)
- [TikTok](https://www.tiktok.com/@cycloneracing)
- [Instagram](https://www.instagram.com/cycloneracingus/)
- [Youtube](https://www.youtube.com/channel/UCQaE_Bqq185kTRbl6uPepTg/videos)

## License

GNU GPLv3
