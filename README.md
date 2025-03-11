# voxl-mpa-to-ros2

ROS2 Foxy Nodes that takes in mpa data and published it to ROS2

## Build Instructions if running directly on target (VOXL2)
1. Ensure you have ros2 foxy installed on target - if not please execute the `./install_foxy.sh` command which will take about 20 minutes to run
2. Once installed, proceed to `cd` into the `colcon_ws` directory and run a `colcon build`
3. Once done, proceed to source the install/setup.script: `source install/setup.bash`
4. Once this is done the user can now run the voxl-mpa-to-ros2 code base by running `ros2 run voxl_mpa_to_ros2 voxl_mpa_to_ros2`

## Build Instructions if running in qrb5165-emulator and pushing to voxl2 post build

1. Requires the qrb5165-emulator (found [here](https://gitlab.com/voxl-public/support/voxl-docker)) to run docker ARM image
    * (PC) ```cd [Path To]/voxl-mpa-to-ros2```
    * (PC) ```git submodule update --init --recursive```
    * (PC) ```voxl-docker -i qrb5165-emulator```
2. Build project binary:
    * (qrb5165-emulator) ```./install_build_deps.sh qrb5165 stable```
    * (qrb5165-emulator) ```./clean.sh```
    * (qrb5165-emulator) ```./build.sh qrb5165```
    * (qrb5165-emulator) ```./make_package.sh```


### Installation
Install mpa-to-ros2 by running (VOXL):

(VOXL2/RB5):

```apt install voxl-mpa-to-ros2```

### Start Installed MPA ROS2 Node

Run the following commands(on voxl2):

Source the ros2 foxy setup script:

```source /opt/ros/foxy/setup.bash```

You can then run the nodes with: 

```ros2 run voxl_mpa_to_ros2 voxl_mpa_to_ros2_node```

##### Supported Interfaces
The current supported mpa->ros2 translations are:  

-cameras from voxl-camera-server or any services that publish an overlay (other than encoded images - only raw)

-IMUs

-6DOF data from voxl-qvio-server

-Point Clouds from the TOF sensor or services providing point clouds such as voxl-dfs-server

### Expected Behavior
```
voxl:/$ ros2 launch voxl_mpa_to_ros2 voxl_mpa_to_ros2.launch

MPA to ROS app is now running

Found new interface: tf_static
Found new interface: stereo
Found new interface: tof_conf
Found new interface: tof_depth
Found new interface: tof_ir
Found new interface: tof_noise
Found new interface: tracking
Found new interface: tof_pc
```
