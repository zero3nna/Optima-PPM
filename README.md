# Hitec Optima PPM

**Only attempt this modification if you know what you are doing and understand the risks.**

The lack of PPM output on Hitec Optima receivers is disappointing. To fill the void I have written firmware that will work with the regular Optima series (Optima Lite not tested) with a combined PPM output. The project origin is hosted (read only) [@code.google](https://code.google.com/p/untestedprototype/)

For more infomation please visit the blog entry from [untestedprototype](http://untestedprototype.com/2012/08/hitec-optima-ppm/)

First of all there are two options to go for:
  * Flash onto the existing slave AVR of the receiver.
  * Solder to the RX line and intercept the signal with another microcontroller.
  
*Personaly I decided to go for the first option, but the process of flashing is still the same.*

**Flashing this firmware is not reversible and will void your warranty. I'm not responsible for anything that happens. Use at your own risk!**

Typical usecases are Optima 6 or 7 recievers because you can strip the body and keep them really minimal. A Optima  9 is kind of waste cause they will remain big and bulky. Small Optimas can be stripped like this:
![Optima 7 Reciever](http://tomsik.eu/sites/tomsik.eu/files/IMG_0592.JPG)

and you can replace the big (BODA) antenna with a small one like this: [FrSky 15cm Antenna](http://www.hobbyking.com/hobbyking/store/__16666__FrSky_Receiver_antenna_15_cm.html)

There are two variations of the firmware:

1. MIXED MODE
  * Regular PWM is output on the lower channels of the receiver (1 to N-1) and combined PPM on the highest channel (N).
2. SELECTABLE MODE (Not tested extensively, only bench tested)
  * Regular PWM on all channels or
  * Connect the signal pin to ground (with a bind plug) on channel 1. Enables combined PPM on the highest channel of the receiver.
  
All builds are available as .hex-file: [Releases](https://github.com/Zero3nna/untestedprototype/releases/)
If you want to build the firmware yourself, you will need to set up a proper build environment and use the provided Rakefile.

**Make sure you flash the correct microcontroller. You want the slave, not the controller for wireless transfer.** Optima 6 and 7 have the pads in similar locations.
![Optima 6 Reciever](http://untestedprototype.com/wp-content/uploads/2013/12/hitec.jpeg)



