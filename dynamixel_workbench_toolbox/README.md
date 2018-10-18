How to run dynamixel_workbench in linux without ROS

1. Downloads DynamixelSDK

```
$ git clone https://github.com/ROBOTIS-GIT/DynamixelSDK
```

2. Build DynamixelSDK

http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/library_setup/cpp_linux/#build-the-library


3. Downloads Dynamixel-Workbench

```
$ git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench
```

4. Build Dynamixel-Workbench and Run examples

```
$ cd ${YOUR_DOWNLOAD_PATH}/dynamixel_workbench/dynamixel_workbench_toolbox/examples
$ mkdir build && cd build
$ cmake ..
$ make
$ sudo chmod a+rw ${USB_PORT}
$ ./monitor
```