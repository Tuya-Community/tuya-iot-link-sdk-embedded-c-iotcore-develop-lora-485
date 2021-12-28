
## Overview
The status data of 485 temperature and humidity sensor and 485 door status sensorare collected by the raspberry PI gateway and reported to the cloud;  At the same time, after receiving the command from the cloud, raspberry PI gateway will control the relay action, and the relay will return the current state to the cloud.  The Raspberry PI Gateway collects the temperature, humidity, and illumination data received by LORA sub-devices and reports the data to the cloud.  


## Get started

### Prerequisites

Ubuntu and Debian
```sh
sudo apt-get install make cmake libqrencode-dev
```

### Compile the code
```sh
mkdir build && cd build
cmake ..
make
```

### Run the demo
```sh
./bindata_model_basic_demo
```

## other

```c
This demo(C) is to communicate with LORA sub-devices and interact with the Internet of Things platform, and the other demo(Python) is the 485 sub-device acquisition program. these two demos are used to build a composite Raspberry PI gateway.  
This demo download address：https://github.com/Tuya-Community/tuya-iot-link-sdk-embedded-c-iotcore-develop-lora-485.git
Another demo that communicates with 485 sub-devices (Python) download address:https://github.com/Tuya-Community/485_sub-devices_connect_to_the_Raspberry_PI_gateway.git
Reference Document Address：https://blog.csdn.net/sandwich_iot/article/details/122195659
```

