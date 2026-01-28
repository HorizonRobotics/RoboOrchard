1. Generate Config file

    Modify the script that generates the configuration file in `gen_sync_config.py` or `gen_async_config.py`

2. Launch deploy node

    ```bash
    # sync deployment
    bash launch_sync_infer.sh
    # async deployment
    bash launch_async_infer.sh
    ```

3. Set instruction

    deploy node will set defualt instruction as "Do something.", you could get current instruction in ROS param server by `ros2 param get /robot/inference_service/sync_node instruction` or `ros2 param get /robot/inference_service/async_rtc_node instruction`, you could also set it by `bash scripts/set_instruction.sh sync/async "xxxxxx"`

4. Enable/Disable deployment

    Now you could enable/disable deployment by inference_app