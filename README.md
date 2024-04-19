After you download this repo, a general workflow is like this:
1. Install STM32CubeMX
2. Open .ioc file with STM32CubeMX and hit generate to get the C project setup (Cmakefile, library code)
3. Setup openOCD toolchain if you haven't done so. You can either base on CLion or VSCode depends on your preference.
4. Change any variables or code you see fit, and flash the firmware to the board when finished.

Note:
- Make sure you only edit inside the comment block (if there is any in the file, especially main.c). Since these files are shared managed by stm32cubemx too so any code outside of the block can be deleted when regenerating.

Suggested Setup of ESC:
    Configuration Options
    prefix parameter                       min   max    current value

    Motor:
    g    Gear Ratio                      0     -      1.000
    k    Torque Constant (N-m/A)         0     -      0.04000

    Control:
    b    Current Bandwidth (Hz)          100   2000   100.000
    l    Current Limit (A)               0.0   75.0   8.000
    p    Max Position Setpoint (rad)     -     -      12.500
    v    Max Velocity Setpoint (rad)/s   -     -      65.000
    x    Max Position Gain (N-m/rad)     0.0   1000.0 1000.000
    d    Max Velocity Gain (N-m/rad/s)   0.0   5.0    5.000
    f    FW Current Limit (A)            0.0   33.0   0.000
    c    Continuous Current (A)          0.0   40.0   8.000
    a    Calibration Current (A)         0.0   20.0   2.000

    CAN:
    i    CAN ID                          0     127    1
    m    CAN TX ID                       0     127    0
    t    CAN Timeout (cycles)(0 = none)  0     100000 1000

========================================Ben's Readme========================================
# motorcontrol
Written specifically for these motor controllers
https://github.com/bgkatz/3phase_integrated
but intended to be easy to port.
 
Written/compiled with ST's Cube IDE:
https://www.st.com/en/development-tools/stm32cubeide.html

Most hardware configuration can be done through hw_config.h

I think it's now fully compatible with the old mbed-based firmware, but let me know if you find bugs.
