# Cyclone Racing Formula SAE
## Iowa State University

This Repository stores the embedded code for our realtime telemetry system. The system is built off of an STM32H7 MCU and pulls data from the ECU and other sensors around the car and sends it in real-time to a wireless receiver, and logs it locally to an SD card. The system uses FreeRTOS to schedule telemetry-related tasks as well as ARM's CMSIS files to interface with registers on the MCU. Most of the example code is built for a NUCLEO test board, while the actual system runs a custom PCB.

## Useful Links
- [Git Cheat Sheet](https://education.github.com/git-cheat-sheet-education.pdf)
- [C Struct Packing](http://www.catb.org/esr/structure-packing/)
- [STM32 Wiki](https://wiki.stmicroelectronics.cn/stm32mcu/wiki/Category:Getting_started_with_STM32_:_STM32_step_by_step)
- [Embedded Systems Design](https://iowastate.sharepoint.com/sites/CycloneRacingFormulaSAE/Shared%20Documents/Forms/AllItems.aspx?id=%2Fsites%2FCycloneRacingFormulaSAE%2FShared%20Documents%2FElectrical%2FResources%2FEmbedded%20Systems%20Introduction%20to%20Arm%20Cortex%2DM%20Microcontrollers%20Fifth%20Edition%2Epdf&parent=%2Fsites%2FCycloneRacingFormulaSAE%2FShared%20Documents%2FElectrical%2FResources)
- [Basic Linux Guide](https://www.tecmint.com/free-online-linux-learning-guide-for-beginners/)

## Dev and Build Tools
The Firmware for this system is developed in WSL using Arm compliers and make files. To get started, follow the steps below.

1. Install WSL by opening up a Powershell terminal and running the command `wsl --install Ubuntu`. This command will setup WSL using an Ubuntu Linux distribution. It will ask for you to create a username and password to complete the setup. Once installed, you can access the Ubuntu terminal by searching for Ubuntu in Windows, or it will be avialable in VSCode once the setup is complete.

2. Install VSCode by going to the [VSCode download page](https://code.visualstudio.com/download) and select the correct package for your system. Follow the directions in the installer.

3. Next, we will link VSCode to WSL using the [WSL Extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-wsl). Navigate to the extensions tab in VSCode and search for WSL. VSCode has [detailed instructions](https://code.visualstudio.com/docs/remote/wsl) on how to set this up and how to open a remote connection to WSL.

4. Next, we will install a series of build tools that are needed to compile the firmware. Open up a WSL terminal and run the following command:
`sudo apt install make gcc-arm-none-eabi gdb-multiarch stlink-tools`
Here is a short explination of each of these tools:
    - Make: a tool that helps manage the compilation and building process of software projects.
    - GCC-Arm-none-eabi: embedded Arm specific build tools and compiler.
    - GDB-Multiarch: adds support for microcontroller debugging
    - STLink-Tools: ST specific tools for developing and uploading embedded code
5. Finally, we will need to install a handful of VSCode extensions, these give us some tools to upload code to the boards as well as see registers and variables while debugging. These extensions are included in `extensions.json` file in the `.vscode` directory on the `main` branch.

Once youve completed these steps, you are ready to get started developing firmware. The best place to start will be the Microcontroller Reference Manual. Read through the section about the peripheral you are trying to work with.

## Building code
In order to build the code, use Make and Makefiles. Open up an Ubuntu terminal and navigate to the project directory. Next, run the command `make`. This will build the code using the instructions in the makefile. Make will only build files that you change, if you would like to rebuild the whole project, then run the command `make clean` before running `make`.

## Uploading Code (WIP)
To upload code to the board, use Cortex-Debug and the ST-Link programmer. The Cortex-Debug extension uses a file in the .vscode directory. There will also need to be additional paths that are added to he Cortex-Debug settings.

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
