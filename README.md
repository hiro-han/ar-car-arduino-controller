# ar-car-arduino-controller

## Build
```
$ cd {catkin_workspace}/src
$ git clone https://github.com/hiro-han/ar-car-arduino-controller.git arduino_controller
$ cd {catkin_workspace}
$ catkin_make
$ catkin_make arduino_controller_firmware_arduino_controller
$ catkin_make arduino_controller_firmware_arduino_controller-upload
$ source devel/setup.bash
```

## Run
```
<launch>
  <node name="arduino_controller" pkg="rosserial_python" type="serial_node.py">
    <param name="port" value="/dev/ttyACM0" />
  </node>
</launch>
```

