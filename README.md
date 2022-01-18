# cheetah_inekf_lcm
Contact-Aided Invariant Extended Kalman Filtering for Mini Cheetah with Lightweight Communications and Marshalling (LCM) interface.
The InEKF takes in data from IMU, joint encoders, and foot contact events to perform state estimation.
This repository has the following features:
* Use LCM for light-weight communication with other programs.
* Allow asynchronous input from IMU, joint encoders, and contact events.
* Allow user to customize for different input source. (Including the contact events.)
* Real-time performance (around 430 Hz) on the upboard computer inside Mini Cheetah. (Tested alongside with the [MIT Controller](https://github.com/mit-biomimetics/Cheetah-Software).)
**Note:** This repository contains a pure cmake project with LCM as the communication interface. If you wish to use ROS, you can refer to [cheetah_inekf_realtime](https://github.com/UMich-CURLY/cheetah_inekf_realtime).

## Dependencies
* [lcm 1.4.0](https://github.com/lcm-proj/lcm/releases/tag/v1.4.0)
* Eigen3
* Boost
* YAML

## Setup
1. cd to the downloaded folder and run the following commands:
```
mkdir build
cd build
cmake ..
make -j2
```
2. To run the program, you can use the following command:
```
./cheetah_estimator
```

## Configuration
### `config/settings.yaml`:
1. `lcm_enable_debug_output` to `false` if no debug output is wanted for receiving lcm messages
2. `project_root_dir` to filepath to your installation directory for this repo
3. `estimator_enable_debug` to `true` if you want to view the state of the inekf in the terminal while it is running
4. `estimator_publish_lcm` to `true` if you want to publish the robot pose over LCM channel "LCM_POSE_CHANNEL"
5. `estimator_lcm_pose_channel` to `true` if you want to change the name of the LCM channel that the robot pose is published over
6. `estimator_static_bias_initialization` to `true` if you want to initialize static bias for the IMU
7. `system_enable_pose_publisher` to `true` if you want to save the robot pose to file and publish the robot pose over ROS
8. `system_inekf_pose_filename` and `system_inekf_tum_pose_filename` to different filepaths to specify which files you would like the robot poses (paths) to be saved to (the second is a tum syntax)
9. `run_synced` to `true` if you have a synced message channel that contains contact estimation results; Otherwise, leave it as `false`
10. `lcm_*_channel` to the corresponding channels your robot is publishing

### In other `.yaml` files:
Change the parameters to the values you know or you want to test with.
## Try the demo data for testing:
## Some helpful functions :
1. Two formats of path *.txt* file will be saved. One is in kitti format, and the other is in tum format. You can use [evo](https://github.com/MichaelGrupp/evo) to visualize the result. The command will be like:
```
evo_traj tum *_tum.txt -p
```
2. The program subscribes to an `lcm_reinitialize_channel`. If the message received in this channel is `true`, the program will be reinitialized and the previous results will be discarded, **including the saved path files**. This LCM message can be published by the RC you are using to control the robot.
## Helpful Commands:
### Generating LCM Types:
1. cd cheetah_inekf_lcm_root_directory/scripts
2. bash ./make_types.sh
### Running Cheetah Estimator
1. cd ~/pathto/catkin_ws
2. In a new terminal in the catkin_ws, do catkin_make (perhaps multiple times)
3. Run `source ~/devel/setup.bash`
5. In the same terminal, run `rosrun cheetah_inekf_lcm cheetah_estimator`
6. Run `lcm file lcm-logplayer --speed=1.0 --lcm-url=udpm://239.255.76.67:7667?ttl=2 NAME_OF_LCM_LOG_FILE_HERE`
7. The terminal should begin printing out the robot state if the settings.yaml output variables are enabled
### Debugging Inekf Output
1. Start running the cheetah estimator using the instructions above
2. Enter `rviz` in the terminal
2. Select `Add by topic` setting and select path
3. Changed fixed frame to the same value as `map_frame_id` in config/settings.yaml
4. The robot pose should begin being drawn in rviz