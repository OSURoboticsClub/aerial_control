Aerial Control Firmware
=======================
_To control anything that ~~flies~~ moves, for now._

Building
--------

    git submodule update --init
    tup variant variants/*.config # Generate variant build directories
    tup build-apollo              # To build variant 'apollo'

Adding another unit/platform
----------------------------
To add another unit `UUU` of platform type `PPP`, add the following:

  * `variants/UUU.config`
  * `variants/units/UUU/Tupfile`
  * `variants/platforms/PPP/Tupfile`

And add the following includes to the root Tupfile:

  * `include variants/units/UUU/Tupfile`
  * `include variants/platforms/PPP/Tupfile`

As necessary, further add the following:

  * Appropriate system code in `src/system`

Conventions
-----------
* Right-handed coordinate system
