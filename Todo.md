Changes / features I want to implement.

- move pwm and adc app pin defines completely to board .h file (complete pinout defined in .h file)
- 
- add define adc app pins in conf.h
    - use swd pins as digital inputs  [swdio(pa13), swclk(pa14), adc1, adc2, servo,
        rx, tx(serial pins disabled as inputs if adc+serial mode disabled. greyed out in vesc tool?), gpio1, gpio2, etc]
    - send list of adc app pins to vesc tool and allow it to define pin function (also allows user to see how to connect things)
    - mode where adc1= throttle (can have bottem end regen), adc2 = brake to max brake current in adc app,
        - digital: reverse, reverses current vector.  brake= when low apply a set current to brake.
        - have "invert pin" toggle to invert behavior
        - allow user to define pullup / pulldown
        - add pulldown mode to adc throttle (and brake?) pins for safety if disconnected
    - trim unneeded modes and add better description of mode function.

- for custom app. define varialbe names, and min/max to send to vesc tool and create config boxes in a row. makes for extensible configuration of custom apps or variables.
