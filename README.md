After you download this repo, a general workflow is like this:
1. Install STM32CubeMX
2. Open .ioc file with STM32CubeMX and hit generate to get the C project setup (Cmakefile, library code)
3. Setup openOCD toolchain if you haven't done so. You can either base on CLion or VSCode depends on your preference.
4. Change any variables or code you see fit, and flash the firmware to the board when finished.

Note:
- Make sure you only edit inside the comment block (if there is any in the file, especially main.c). Since these files are shared managed by stm32cubemx too so any code outside of the block can be deleted when regenerating.

========================================Ben's Readme========================================
# motorcontrol
Written specifically for these motor controllers
https://github.com/bgkatz/3phase_integrated
but intended to be easy to port.
 
Written/compiled with ST's Cube IDE:
https://www.st.com/en/development-tools/stm32cubeide.html

Most hardware configuration can be done through hw_config.h

I think it's now fully compatible with the old mbed-based firmware, but let me know if you find bugs.
