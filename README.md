# cheetah_inekf_lcm
Contact-Aided Invariant Extended Kalman Filtering for Mini Cheetah with Lightweight Communications and Marshalling (LCM) interface.
The InEKF takes in data from IMU, joint encoders, and foot contact events to perform state estimation.

This repository has the following features:
* Use LCM for lightweight communication with other programs.
* Allow asynchronous input from IMU, joint encoders, and contact events.
* Allow user to customize for different input sources. (Including the contact events.)
* Real-time performance (around 430 Hz) on the UP Board computer inside Mini Cheetah. (Tested alongside with the [MIT Controller](https://github.com/mit-biomimetics/Cheetah-Software).)

**Note:** This repository uses LCM to achieve lightweight communication. If you wish to use ROS, you can refer to [cheetah_inekf_realtime](https://github.com/UMich-CURLY/cheetah_inekf_realtime).

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
### Parameters can be modified in `config/settings.yaml`:
* `run_synced`: Set to `true` if you wish to subscribe from a synced input topic.
* `estimator_enable_debug`: Enable debug print on the screen.
* `estimator_publish_lcm`: Enable LCM publisher for the estimated pose.
* `estimator_lcm_pose_channel`: Name of the LCM channel for output robot pose.
* `estimator_static_bias_initialization`: Enable static bias initialization using the first several measurements from IMU.
* `system_enable_pose_log_txt`: Enable pose logger. Enable this will write down the estimated pose in a txt file.
* `system_inekf_kitti_pose_filename`: Path for the logged txt file. Kitti means the file will be recorded following the [Kitti format](http://www.cvlibs.net/datasets/kitti/eval_odometry.php). 
*  `system_inekf_tum_pose_filename`: Path for the logged txt file in [TUM format](https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats). 
* `lcm_leg_channel`: LCM channel for joint encoder inputs. It contains q, qd, p, v, tau.
* `lcm_imu_channel`: LCM channel for IMU inputs. It contains acceleration, omega, rpy, quaternion.
* `lcm_contact_est_channel`: LCM channel for indicating contact events.  
* `lcm_reinitialize_channel`: LCM channel for reinitialization command. It contains a boolean value indicating if the filter needs to be reset.

## Resetting the Filter
This program enables users to reset the filter whenever is needed. 
* To reset the filter, publish a `true` signal in the `reinitialization_lcmt` LCM type. (Don't forget to change the channel name in `config/settings.yaml`.)
* Note that when the filter is reset, all previous paths, including the saved txt files will be reset.

## Using Customized Contact Estimation
* If you wish to use a customized source of contact events, publish your contact estimation results in `contact_est[4]` under LCM type `wbc_test_data_t`.
* `contact_est[4]` denotes the contact event for each leg of the Mini Cheetah. 0 indicates no contact, and 1 indicates a firm contact.
* If you would like to have reliable contact estimations, check out our recent work [deep-contact-estimator](https://github.com/UMich-CURLY/deep-contact-estimator) and [cheetah_inekf_realtime](https://github.com/UMich-CURLY/cheetah_inekf_realtime).


## Citation
If you find this work useful, please kindly cite our publication in 2021 Conference on Robot Learning:

* Tzu-Yuan Lin, Ray Zhang, Justin Yu, and Maani Ghaffari. "Legged Robot State Estimation using Invariant Kalman Filtering and Learned Contact Events." In Conference on robot learning. PMLR, 2021
```
@inproceedings{
   lin2021legged,
   title={Legged Robot State Estimation using Invariant Kalman Filtering and Learned Contact Events},
   author={Tzu-Yuan Lin and Ray Zhang and Justin Yu and Maani Ghaffari},
   booktitle={5th Annual Conference on Robot Learning },
   year={2021},
   url={https://openreview.net/forum?id=yt3tDB67lc5}
}
```
