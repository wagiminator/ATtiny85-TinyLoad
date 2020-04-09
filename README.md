# TinyLoad - Simple Electronic Dummy Load based on ATtiny85
The ATtiny85 measures voltage, current and temperature of the heat sink, calculates power, energy and battery capacity, controls the fan and displays all relevant data on the OLED. The button is used to switch between power/restistance and energy/capacity display.

The ADC of the ATtiny does its best to make the tinyLoad a pretty accurate tool, but it might need a little calibration.

How to calibrate:
- Set ULCAL and ILCAL to "1" in the sketch, compile and upload.
- Choose a stable input voltage of around 5V and turn the poti until the display shows a current of around 0.7A. Measure the voltage and the current with a trusty multimeter or a good lab bench power supply.
- Calculate the calibration factors as follows:
- ULCAL = voltage measured with multimeter / voltage shown on OLED.
- ILCAL = current measured with multimeter / current shown on OLED.
- Set the ULCAL and ILCAL value in the sketch, compile and upload again.

Notes:
- Use a good heatsink with a 5V fan for the MOSFET !
- Be careful with high power loads ! This device is called "tinyLoad" for a reason !
- Always turn the POTI full counter-clockwise before connecting the load !
- Due to the input offset voltage of the OpAmp the minimum load current is 17mA.
- The maximum load current is 4.5A, however for small voltages it might be less.
- Do not exceed the maximum voltage of 26V !

Project on EasyEDA: https://easyeda.com/wagiminator/y-attiny85-electronic-load

![IMG_20200409_133602_x.jpg](https://image.easyeda.com/pullimage/idF4nEBSxHMoRsFqdBv2tVOhwem6n1S4n9iPAhVX.jpeg)

![IMG_20200409_133709_x.jpg](https://image.easyeda.com/pullimage/jd1wluc54d1eXohNjokzkOAsJymzH5ixjnlDlrCd.jpeg)

![IMG_20200401_180140_x.jpg](https://image.easyeda.com/pullimage/KBjzbXSX2q9HdlKo6d0NRewWeuKPdOpT36tDqTWc.jpeg)
