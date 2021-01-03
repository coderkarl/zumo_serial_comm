# ROS Zumo 32u4 Robot
This is for the [Pololu Zumo 32u4 robot](https://www.pololu.com/category/170/zumo-32u4-robot)  

## ROS2 foxy package
`sudo apt install python3-serial`  
Assuming your ros2 workspace dir is ros2_ws/  
Clone this repo to your ros2_ws/src/ directory on your raspberry pi.  

`git clone https://github.com/coderkarl/zumo_serial_comm.git`  

`cd ros2_ws`  

`colcon build --symlink-install --packages-select zumo_serial_comm --cmake-args ' -DCMAKE_BUILD_TYPE=Release'`  

`source ros2_ws/install/setup.bash`  

`ros2 run zumo_serial_comm zumo_serial`  

Modify the port /dev/ttyACM0 as needed (add a udev rule).  

## Zumo 32u4 Microcontroller Code
Upload the arduino/zumo_ros_comm/ file to the 32u4 microcontroller.  

## Power to the raspberry pi
I had a random set of round pin and socket connectors (larger than standard dupont) with crimp ends.  
The socket is similar to this [molex connector on digikey](https://www.digikey.com/en/products/detail/molex/0008700001/2404811)  
I flattened the crimp end and removed the wire insulation grips. This left me with a "mini-spade" on one end and the socket on the other. I soldered the "mini-spade" to the battery terminals at the back of the Zumo PCB. My goal was to put the "mini-spade" in the PCB hole between the battery case terminals and the PCB. If I did it again, I would first remove the solder with a sucker then try to fit my connector in and re-solder.  

I crimped the pin connector to 18 AWG wire and that went to an [adafruit 5V 3A converter](https://www.adafruit.com/product/1385?gclid=EAIaIQobChMI9ciLq8T-7QIVx0XVCh21AgkhEAQYASABEgJ8T_D_BwE) for a temporary initial solution.  

The final design will have first a [boost converter to 7.5 V](https://www.pololu.com/product/2893)  that goes into the adafruit 5V 3A converter.  
