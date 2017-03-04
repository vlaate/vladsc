# Very Low Accuracy Digital Setting Circles

Amateur astronomers want to know where their telescope is pointing at. For this reason, many commercial telescopes come equipped with "push to" or "go to" features, often based on high precision optical rotary encoders attached to the telescope mount, and a hand control device with a database with the coordinates of thousands of stars and other sky objects.

There are also commercial Digital Setting Circles, which are kits sold for telescope owners to adapt to their existing telescope, in order to give it "push to" features. Commercial DSCs are very accurate, mostly based on optical encoders, and usually cost a few hundred dollars.

This project is an implementation of a simple Digital Setting Circles that achieves the following:
* Easy and inexpensive to build, for any DIY maker.
* Can be attached to almost any telescope, including tripods, tabletops and other designs that can't house optical rotary encoders.
* Instead of a handheld control, communicates with the Skysafary 5 (plus or pro) app (iOS or Android) via WiFi.
* Can be powered with a simple 5V USB powerbank.

In order to achieve the above, the following sacrifices had to be made:
* **Very** Low Accuracy, because of the use of accelerometer and magnetometer sensors instead of optical encoders, the high resolution of optical encoders is lost, along with their high precision and repeatability.
* Because of the above, it's best used with a low magnification eyepiece that provides a *True Field Of View* of at least 2º.
* It's more of an aiding device for exploring the sky via star hopping, than a "push to" feature to zero in on difficult sky objects.
* One star alignment with Skysafari is critical, so some basic knowledge of a sky object to align with is necessary (moon, Orion's belt, Sirius, etc).

##The Hardware
The Circuit looks like this:

(TBD)

These are the components to build it:
* Microcontroller: ESP8266 development board (a.k.a. "NodeMCU", or "12E"). Tested with [HiLetgo](https://www.amazon.com/gp/product/B010O1G1ES) brand.
* Accelerometer: MMA7455 sensor module, like [this](https://www.amazon.com/gp/product/B00UJ67SBE) one.
* Magnetometer: HMC5883 sensor module (a.k.a. GY-271), like [this](https://www.amazon.com/gp/product/B00UAIY698) one.
* Two resistors, to pull-up the I2C bus, tested with 2220 Ohm (red red red)
* USB Power bank.
* Electronic dupont cables, or UTP cables, long enough to connect the microcontroller to the sensors.
* Arduino programming environment (PC or Mac, Arduino IDE, data micro USB cable, etc)
* Telescope

The accelerometer sensor should be attached sideways (Z axis horizontal) to the telescope optical tube, so that its X and Y readings change with telescope movement, but Z reading remans at or near to zero.

The magnetometer should be laid horizontal and aligned with the mount, but away from the Microcontroller's WiFi antenna and from the battery (USB power bank).


##Roadmap

This is still a work in progess
