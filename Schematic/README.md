# Inverted-Pendulum
Circuit presented as on breadboard for the Self balancing robot based on ESP8266

Note that this is not the real positioning of the components, it just need to let you and me in the future to understand the circuit, even the battery and the motors are just for "understand" infact I use a 2S LIPO by the husban h502e/s for my project, while the motors are n20 12V 200 or 300 rpm kind. 

If you are using a breadboard I personally suggest you to supply with 5V the ESP8266 first. I also have got a lot of issues by using  prototyping cables from the bread board to the hc-sr04, because when the servo is moving, the cables are stressed and they tend to break.   

The motor driver used is the L298, but you can also use the L9110S(for the n20) but with the L298 you need to set the XL6009 output to 13,4V because of the protection diode. 

You need to set the trimmer that is used for the battery status to 3.3V (for wemos d1 mini, 1V for the nodeMCU) when the battery is 8.6V.
While the trimmer for the hc-sr04 must be set to 3.3V when the input is 5V. 

Set the trimmers before to connect them to the ESP
