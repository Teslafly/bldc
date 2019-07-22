Changes / features I want to implement.
fourm thread:
https://vesc-project.com/node/1046


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



Vesc adc app, digital and analog values visible in adc app tab when rt app enabled

Digital inputs show up as green boxes when high?  (inverting affects this?

Also be able to select if pin has pull up pull down or float. Default = pullup

Pin configuration page saying if pin is clobbered, inverted, or has pullup/down? Also reading rt pin state? 

-per vedder, the adc mode of pins doesnt allow pullups/pulldowns? if you try to do it anyways what happens? fix is on new hw if actually impossible. may be able to switch modes rapidly to actually detect 