实现功能：通过树莓派网关采集485温湿度传感器以及485门磁开关状态数据上报到云端；
同时收到云端的指令，树莓派网关处理之后，控制继电器动作，同时继电器返回当前的状态给云端；
树莓派网关采集LORA子设备接收到的温湿度以及光照度数据上报到云端。

Functions: The status data of 485 temperature and humidity sensor and 485 door status sensor
are collected by the raspberry PI gateway and reported to the cloud;  
At the same time, after receiving the command from the cloud, raspberry PI gateway will control the relay action, 
and the relay will return the current state to the cloud.  
The Raspberry PI Gateway collects the temperature, humidity,
 and illumination data received by LORA's sub-devices and reports the data to the cloud.  