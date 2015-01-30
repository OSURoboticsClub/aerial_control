Aerial Control Firmware
=======================
_To control anything that ~~flies~~ moves, for now._

Building
--------

    git submodule update --init
    tup variant variants/*.config # Generate variant build directories
    tup build-apollo              # To build variant 'apollo'

Adding another platform
-----------------------
To add another unit `UUU` of platform type `PPP` as variant `VVV`, add the following:

  * `variants/units/UUU/Tupfile`
  * `variants/platforms/PPP/Tupfile`
  * `variants/VVV.config`

As necessary, further add the following:

  * Appropriate system code in `src/system`

By convention, we name variants after units, so `UUU` and `VVV` are currently
the same in all cases.

Conventions
-----------
* Right-handed coordinate system
