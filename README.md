# Cyclone Racing Formula SAE
## Iowa State University

This Repository stores the C/C++ code for our realtime telemetry system. The system is built off of an STM32H7 MCU. This system pulls data from the ECU and other sensors around the car and sends it in real-time to a wireless receiver, and logs it locally to an SD card. The system uses FreeRTOS to schedule telemetry-related tasks as well as ARM's CMSIS files to interface with registers on the MCU. Most of the example code is built for a NUCLEO test board, while the actual system runs a custom PCB with a slightly different STM32 MCU.

## Useful Links
- [Git Cheat Sheet](https://education.github.com/git-cheat-sheet-education.pdf)
- [C Struct Packing](http://www.catb.org/esr/structure-packing/)
- [STM32 Wiki](https://wiki.stmicroelectronics.cn/stm32mcu/wiki/Category:Getting_started_with_STM32_:_STM32_step_by_step)
- [Embedded Systems Design](https://iowastate.sharepoint.com/sites/CycloneRacingFormulaSAE/Shared%20Documents/Forms/AllItems.aspx?id=%2Fsites%2FCycloneRacingFormulaSAE%2FShared%20Documents%2FElectrical%2FResources%2FEmbedded%20Systems%20Introduction%20to%20Arm%20Cortex%2DM%20Microcontrollers%20Fifth%20Edition%2Epdf&parent=%2Fsites%2FCycloneRacingFormulaSAE%2FShared%20Documents%2FElectrical%2FResources)

## Dev and Build Tools
These are the required tools to develop and build this code. The instructions following this list will tell you how to set everything up.
- [VSCode](https://code.visualstudio.com/download) -- The IDE to write code in
- [arm-none-eabi-gcc](https://developer.arm.com/downloads/-/gnu-rm) -- Arm Compiler, make sure to add to PATH and use the .exe for Windows Install
- [CMake](https://cmake.org/download/) -- Build C Code, make sure to add to PATH and use the binary for easy Windows install
- [w64Dev](https://github.com/skeeto/w64devkit) -- THIS CAN BE REMOVED??
- [STM32 Programmer](https://www.st.com/en/development-tools/stm32cubeprog.html) -- STM32 Programmer Software
- [WSL](https://learn.microsoft.com/en-us/windows/wsl/install) -- Linux Subsytem for Windows
- [usbipd](https://github.com/dorssel/usbipd-win/releases) -- Used to pass USB information to WSL
- 
## VSCode Extensions
- [Cortex Debug](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug) -- Used for debugging in VSCode
- [Keil Extension](https://marketplace.visualstudio.com/items?itemName=Arm.keil-studio-pack) -- VSCode Cortex Development Extensions

The first thing to install should be VSCode. This will be the IDE that code gets written into. Next, you should go about installing arm-none-eabi-gcc. Make sure that you add this to your PATH when promtped during setup. CMake is the next utility to install. The installation wizard for this should be fairly simple, just make sure that you add this tool to your PATH variable as well. The third program is STM32Cube Programmer. This piece of software allows you to upload code to the boards using the ST-Link USB device. These programs are all basic windows installers. 
In order to install WSL, open a windows Powershell terminal as administrator and run the command `WSL --install`. This will start the download and setup of a Linux Distribution inside of Windows. You will have to enter a new username and password for your linux terminal and may need to restart.
Next we will install usbipd. This is a program that allows WSL to see USB devices, such as the ST-Link. Download the package and run the installer. Once this is complete, run the commands `sudo apt install linux-tools-generic hwdata` and `sudo update-alternatives --install /usr/local/bin/usbip usbip /usr/lib/linux-tools/*-generic/usbip 20` on your WSL terminal. [This Link](https://learn.microsoft.com/en-us/windows/wsl/connect-usb) provides more detailed instructions on usbipd installation and setup.

## Building code

## Uploading Code

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
