Handeye Calibration
===================
We provide a hand-eye calibration toolkit designed to determine the spatial transformation between the robot arm and the camera(i.e., camera extrinsics). The tool supports both eye-in-hand and eye-to-hand configurations. By utilizing teleoperation (e.g., ALOHA) or a predefined set of waypoints, the system captures the ArUco marker pose at various poses within the camera's field of view and the robot arm end-effector pose at the same time. This process yields a series of pose pairs, from which the final calibration matrix is computed.

.. figure:: ../_static/images/calib.gif
   :alt: handeye calibration
   :align: center

   Sample Data Collection Process

- Get aruco_ros package and build it

.. code-block:: bash

   cd ros2_package
   git clone https://github.com/pal-robotics/aruco_ros -b humble-devel thirdparty
   cd thirdparty
   colcon build
   source install/setup.bash

- Configure the parameters in launch_handeye_calib.sh in the handeye_calib directory to your actual usage:

.. list-table:: Handeye Calibration Parameters Configuration
   :widths: 30 70
   :header-rows: 1

   * - Arguments
     - Explain
   * - MODE
     - 'eye_in_hand' or 'eye_to_hand'
   * - ARUCO_MARKER_SIZE
     - size of aruco marker you used
   * - ARUCO_MARKER_ID
     - ID of aruco marker you used
   * - CAMERA_FRAME_NAME
     - frame name of the camera to be calibrated, which must match the actual frame name published by your camera driver
   * - ARUCO_MARKER_FRAME_NAME
     - The frame name to be assigned to the ArUco marker. You can typically keep the default 'marker_frame'
   * - CAMERA_INFO_TOPIC
     - The ROS2 topic name of your camera info
   * - CAMERA_RAW_TOPIC
     - The ROS 2 topic name for your camera info/raw image
   * - END_EFFECTOR_FRAME_NAME
     - frame name of your EEF, which can typically be found in your URDF file
   * - BASE_FRAME_NAME
     - The base frame name of your robot arm (e.g., ``base_link``), which can typically be found in your URDF file
   * - END_EFFECTOR_POSE_TOPIC
     - The ROS 2 topic name where the end-effector (EEF) pose is published
   * - RESULT_FILE
     - path to save calibration result json file

- The final output is a json file in the ros tf format, which specifies the parent and child frames, and represents the result with quaternion. You could see a sample output of eye-in-hand calibration result in the following:

.. note::
  - Frame is defined in ROS standard
  - Quaternion is **scalar-last**, i.e., position: [x, y, z], orientation: [qx, qy, qz, qw]

.. code-block:: json

   {
       "parent_frame": "left_end_effector",
       "child_frame": "left_wrist_camera",
       "result": {
           "position": [
              -0.07111201370909159,
              0.005933229997508317,
              0.026299147506149152            
           ],
           "orientation": [
              0.12614812772318,
              -0.15849564864236187,
              0.704431212421235,
              -0.6802517520742759            
           ]
       }
   }


- If your configed the launch_handeye_calib.sh correctly, you will see the following visualization in foxglove studio, if the result is good, the marker_frame will not change much.

.. grid:: 2
   :gutter: 3

   .. grid-item::
      .. figure:: ../_static/images/eye_in_hand_vis.gif
         :alt: eye in hand visualization
         :align: center

         eye in hand visualization

   .. grid-item::
      .. figure:: ../_static/images/eye_to_hand_vis.gif
         :alt: eye to hand visualization
         :align: center

         eye to hand visualization