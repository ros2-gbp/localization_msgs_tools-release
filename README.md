# Bridge and adapt messages for robot localization

This package offers two nodes to convert various localization-related messages.

# `pose_to_tf`: forward pose messages to tf tree

This node subscribes to a pose topic and forwards it to `/tf`.
It can be used to simulate perfect localization from a ground truth topic published e.g. by a simulator. Supported messages are:

- `geometry_msgs/Pose`
- `geometry_msgs/PoseStamped`
- `geometry_msgs/Transform`
- `geometry_msgs/TransformStamped`
- `nav_msgs/Odometry`
- `sensor_msgs/Imu` (assumes null translation)

## Parameters

- `topic`: topic to subscribe to (defaults to `pose_gt`)
- `parent_frame`: parent frame to be used in published messages (defaults to `world`)
- `child_frame`: child frame to be used in tf publisher (defaults to empty)
- `inverse` (default False): publish the inverse of the received transform, can be useful to calibrate sensors

The frame parameters are only to complement messages that do not include the information:

- `Pose` and `Transform` do not convey any frame, so both parameters are required;
- `PoseStamped` and `Imu` only convey `child_frame` in the header, the `parent_frame` parameter is thus mandatory;
- `TransformStamped` and `Odometry` convey both `child_frame` explicitely and `parent_frame` in the header.

In any case, if frame parameters are not empty they will override the frames from the incoming messages.

## Running the node

The node is available:

- as an executable:  `pose_to_tf`
- as a component: `localization_msgs_tools::Pose2TF`


# `with_covariance`: add covariance to localization messages

This node is meant to bridge ROS messages by adding or replacing covariance info. A classical use is messages
coming out of simulation that may have no covariance. Some frameworks or sensors also publish covariance-free messages, making them unsuitable for use with classical ROS tools. It can also help tuning the covariance at runtime.

Supported messages are:

- `geometry_msgs/PoseWithCovarianceStamped`
    - can also take in `geometry_msgs/Pose` or `geometry_msgs/PoseStamped`
- `geometry_msgs/TwistWithCovarianceStamped`
    - can also take in `geometry_msgs/Twist` or  `geometry_msgs/TwistStamped`
- `sensor_msgs/Imu`
- `sensor_msgs/NavSatFix`
- `nav_msgs/Odometry`

## Parameters

Similarly to the well-known `robot_localization` nodes, the `with_covariance` node takes in possibly various messages, each of them having to be associated:

- an input type, defined by the name of the parameter (`pose0`, `imu0`, etc.)
- input and output topics
- covariance information as length-3 vectors
    - `xyz` and `rpy` for pose
    - `linvel` and `angvel` for twist
    - `accel` for acceleration
    - covariances that are not set from the parameters will be copied from the incoming message, if any
    - covariance parameters can be changed at runtime
- `frame_id` if they are not part of the incoming messages (e.g. `Pose` and `Twist`)

An example is provided for all supported messages:

```
/**:
    ros__parameters:
        # subscribe to some Pose, publish as PoseWithCovarianceStamped
        pose0: pose_gt
        pose0.frame_id: base_link
        pose0.out: pose_with_cov
        pose0.cov:
            # linear covariance
            xyz: [.1, .1, .1]
            # angular covariance
            rpy: [.1, .1, .1]

        # IMU
        imu0: imu_raw
        imu0.out: imu_with_cov
        imu0.cov:
            # angular covariance
            rpy: [.1, .1, .1]
            # angular velocity covariance
            angvel: [.1, .1, .1]
            # acceleration covariance
            accel: [.1, .1, .1]

        # Odometry
        odom0: odom_raw
        odom0.out: odom_with_cov
        odom0.cov:
            # xyz: [] # do not change incoming covariance for pose
            # rpy: []
            linvel: [.1, .1, .1]
            angvel: [.1, .1, .1]

        # Some kind of Twist
        twist0: twist_raw
        twist0.out: twist_with_cov
        twist0.frame_id: base_link
        twist0.cov:
            linvel: [.1, .1, .1]
            angvel: [.1, .1, .1]

        # NavSat
        navsat0: navsat_raw
        navsat0.out: navsat_with_cov
        navsat0.cov.xyz: [5.,5.,1.]
```

## Changing covariances at runtime

All covariance parameters can be updated at runtime by setting the corresponding parameter:

```
ros2 param set /with_covariance pose0.cov.xyz [.1,.1,.1]
```


## Running the node

The node is available:

- as an executable:  `with_covariance`
- as a component: `localization_msgs_tools::WithCovariance`
