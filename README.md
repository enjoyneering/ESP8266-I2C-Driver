[![license-badge][]][license] [![stars][]][stargazers] [![hit-count][]][count] [![github-issues][]][issues]

# ESP8266-I2C-Driver
Fixed built-in Master I²C driver for Arduino ESP8266 core. Sorry it doesn't support slave I²C mode.

| Board | SDA | SCL | Level |
| ---- | ---- | ---- | ---- |
| ESP8266 | GPIO4 | GPIO5 | 3.3v/5v |
| ESP8266 ESP-01 | GPIO0/D5 | GPIO2/D3 | 3.3v/5v |
| NodeMCU 1.0, WeMos D1 Mini | GPIO4/D2 | GPIO5/D1 | 3.3v/5v |

Copy and replace "**twi.h**", "**core_esp8266_si2c.cpp**" in folder %USERPROFILE%\AppData\Local\Arduino15\packages\esp8266\hardware\esp8266\2.5.2\cores\esp8266

Copy and replace "**Wire.h**", "**Wire.cpp**" in folder %USERPROFILE%\AppData\Local\Arduino15\packages\esp8266\hardware\esp8266\2.5.2\libraries\Wire

If you want to keep the original files - just change the file extension* to "**twi.h.OLD**" and etc. Then you always can go back.

*_if you change the file name you get an error at compile time_

Special thanks to [Tech-TX(StanJ)](https://github.com/Tech-TX)

[license-badge]: https://img.shields.io/badge/License-GPLv3-blue.svg
[license]:       https://choosealicense.com/licenses/gpl-3.0/
[stars]:         https://img.shields.io/github/stars/enjoyneering/ESP8266-I2C-Driver.svg
[stargazers]:    https://github.com/enjoyneering/ESP8266-I2C-Driver/stargazers
[hit-count]:     http://hits.dwyl.io/enjoyneering/ESP8266-I2C-Driver/badges.svg
[count]:         http://hits.dwyl.io/enjoyneering/ESP8266-I2C-Driver/badges
[github-issues]: https://img.shields.io/github/issues/enjoyneering/ESP8266-I2C-Driver.svg
[issues]:        https://github.com/enjoyneering/ESP8266-I2C-Driver/issues/
