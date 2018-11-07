Version of the self balancing robot based on the ESP 8266 


This version is much faster because uses the asynch web server and web socket for remote controls


You just need to upload the sketch into your ESP8266 and then accessing to it, the ssid is ESP8266SBR and the password is Algorithm, then digit 192.168.4.1 as ip, and then you'll be free to set all parameters for use your self balancing robot.


It also manage a sr04 sensor wich will help you to not hit object around you while remote controllling it.

This version has OTA handler wich be managed during the stand-by, during that you can access to the website of the robot or upload the firmware remotely without cable.


The stl files can be found here in case you want to print my design https://www.thingiverse.com/thing:3084400 or if you click on Inverted-Pendulum and you can download the entire project which also contain the stl files.

External libraries you need to import before you can compile the sketch:

ESPAsyncTCP  ---> https://github.com/me-no-dev/ESPAsyncTCP

ESPAsyncWebServer ---> https://github.com/me-no-dev/ESPAsyncWebServer

PID_v1 ---> https://github.com/br3ttb/Arduino-PID-Library/

ArduinoJson ---> https://github.com/bblanchon/ArduinoJson (This is the 5.13stable version)

