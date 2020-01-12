# How to set up On-Chip Debugging for VESC on VS-Code
## Installation
1. Make sure you have the packages necessary to build and develop with the STM32
```bash
sudo apt-get install build-essential git flex bison libgmp3-dev libmpfr-dev libncurses5-dev libmpc-dev autoconf texinfo libtool libftdi-dev libusb-1.0-0-dev zlib1g zlib1g-dev python-yaml
```

2. Install OpenOCD either using your package manager or building it from source. You can find more information here: http://openocd.org/getting-openocd/

3. Install the [Cortex-Debug VS Code Extension](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug) in your instance of Visual Studio Code.

4. Add the following to your *launch.json* under *configurations* in the *.vscode* folder in your VS Code workspace:
```json
{
    "name": "VESC Debug",
    "type": "cortex-debug",
    "request": "launch",
    "cwd": "${workspaceFolder}",
    "executable": "${workspaceRoot}/build/BLDC_4_ChibiOS.elf",
    "servertype": "openocd",
    "device": "stm32f4x",
    "configFiles": ["${workspaceFolder}/stm32-vesc-openocd-vscode.cfg"]
}
```

If you do not have anything additional in your *launch.json* file, your file should look like this:
```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "VESC Debug",
            "type": "cortex-debug",
            "request": "launch",
            "cwd": "${workspaceFolder}",
            "executable": "${workspaceRoot}/build/BLDC_4_ChibiOS.elf",
            "servertype": "openocd",
            "device": "stm32f4x",
            "configFiles": ["${workspaceFolder}/stm32-vesc-openocd-vscode.cfg"]
        },  
    ]
}
```
## Testing
1. Ensure you have built your code, so the latest executable is used.
2. To test the debugger, place a breakpoint in your code. Then, click on the *debug* tab, or hit *CTRL+ALT+D* on your keyboard.
3. At the top of the *debug* tab, click the carrot for the dropdown menu (next to *DEBUG AND RUN*) and select the name of your configuration. The name from the example above is "VESC Debug"
4. Ensure your VESC is properly connected to your computer using the ST-Link. If you need help doing this, please see the following link and scroll down to **Software Installation and Configuration Tool**: http://vedder.se/2015/01/vesc-open-source-esc/ 
5. Select the green RUN button next to the dropdown menu. This should open up the debug menu and your bottom bar should turn orange (as long as you do not have a theme installed which changes this color)
