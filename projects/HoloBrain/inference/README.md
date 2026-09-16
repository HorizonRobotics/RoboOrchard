1. Generate Config file

    Modify the script that generates the configuration file in `gen_sync_config.py` or `gen_async_config.py`

    The Piper command names match the dual-arm launch defaults:
    `left_joint1` through `left_joint6`, `left_gripper`, and the corresponding
    `right_` names. If you override the driver's `joint_names` list, update
    these command names, any observation selectors, and Pico's expected
    joint lists together. The default gripper label is `gripper`, not
    `joint7`; another label requires consistent explicit configuration.

    Joint observations now send the driver's `name` and `position` together
    as JSON under the existing channel key. Update the model server to parse
    this format before upgrading the client. Image/RTC binary arrays and
    response action arrays remain unchanged.

2. Launch deploy node

    ```bash
    # sync deployment
    bash launch_sync_infer.sh
    # async deployment
    bash launch_async_infer.sh
    ```

3. Control deployment in inference_app

    Now you could control deployment by inference_app:

    - Start inference.
    - Stop inference.
    - Reset all arm controllers listed in the inference app launch configuration.

    In the default HoloBrain setup, the reset service list includes both master and puppet arms on the left and right sides.

    If your deployment needs a different reset scope, update `reset_arm_service_name` in `projects/HoloBrain/app/gen_inference_app_launch_config.py`.
