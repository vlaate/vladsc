# Very Low Accuracy Digital Setting Circles

Amateur astronomers want to know where their telescope is pointing at. For this reason, many commercial telescopes come equipped with "push to" or "go to" features, often based on high precision optical rotary encoders attached to the telescope mount, and a hand control device with a database of coordinates of thousands of stars and other sky objects.

There are also commercial Digital Setting Circles, which are kits sold for telescope owners to adapt to their existing telescope, in order to give it "push to" features. Commercial DSCs are very accurate, mostly based on optical encoders, and usually cost a few hundred dollars.

This project is an implementation of a simple Digital Setting Circles that achieves the following:
* Easy and inexpensive to build, for a DIY maker or Arduino enthusiast.
* Can be attached to almost any telescope mount, including tripods, tabletops and other designs that can't house optical rotary encoders.
* Instead of a wired handheld control, it communicates with the Skysafary 5 (plus or pro) app (iOS or Android) via WiFi.
* Can be powered with a simple 5V USB powerbank.

In order to achieve the above, the following sacrifices had to be made:
* **Very** Low Accuracy, because of the use of accelerometer and magnetometer sensors instead of optical encoders, the high resolution of optical encoders is lost, along with their high precision and repeatability.
* Because of the above, it's best used with a low magnification eyepiece that provides a *True Field Of View* of at least 2º.
* It's more of an aiding device for exploring the sky via star hopping, than a "push to" feature to zero in on difficult sky objects.
* One star alignment with SkySafari is critical, so some basic knowledge of a sky object to align with is necessary (moon, Orion's belt, Sirius, etc).

##The Hardware
The Circuit is very simple, and looks like this:

![alt text](http://atehortua.com/vladi/wp-content/uploads/2017/03/schematic.jpg "VLADSC Mark I schematic")

These are the components to build it:
* Microcontroller: ESP8266 development board (a.k.a. "NodeMCU", or "12E"). Tested with [HiLetgo](https://www.amazon.com/gp/product/B010O1G1ES) brand.
* Accelerometer: MMA7455 sensor module, like [this](https://www.amazon.com/gp/product/B00UJ67SBE) one.
* Magnetometer: HMC5883 sensor module (a.k.a. GY-271), like [this](https://www.amazon.com/gp/product/B00UAIY698) one.
* Two resistors, to pull-up the I2C bus, tested with 2.2k Ohm (red red red)
* USB Power bank.
* Electronic dupont cables, or UTP cables, long enough to connect the microcontroller to the sensors.
* Arduino programming environment (PC or Mac, Arduino IDE, data micro USB cable, etc).
* Android or iOS Tablet with the SkySafari 5 Plus or Pro app installed.

The accelerometer sensor should be attached sideways (Z axis horizontal) to the telescope optical tube, so that its X and Y readings change with telescope movement, but Z reading remans at or near to zero. It should be aligned so that the Y axis points upwards when the telescope is aimed at Zenith.

The magnetometer should be laid horizontal and aligned with the mount, but away from the microcontroller's WiFi antenna and from the battery (USB power bank).

On a tabletop reflector telescope, the sensors would look like this:
![alt text](http://atehortua.com/vladi/wp-content/uploads/2017/03/assembly.jpg "Sensors on a tabletop Reflector")



##Roadmap

This is still a work in progress, and many features are not yet implemented, such as:
* Compass tilt compensation, so that the compass does not need to be horizontal and can be attached to the telescope's optical tube, thus providing support for telescope mounts like the "ball dobsonian" or travel tripods.
* Support for other sensors, and Integrated Measurement Units, making it easier to use with smaller tripods.
* WiFi access point emulation, so that the DSC creates its own WiFi network and can be used away from routers. 
* Web configuration menu, so that SSID, password, and even calibration/smoothing parameters can be entered via web browser (and saved to EEPROM) instead of being hardcoded.
* Hardware user interface, such as LCD and buttons, to make it easier to operate for end-users who are not Arduino enthusiasts.
* Integration with the free SkyEye app, which is starting to support sensors.
