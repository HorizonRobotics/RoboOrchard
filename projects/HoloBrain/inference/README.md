1. Generate Config file

    Modify the script that generates the configuration file in `gen_sync_config.py` or `gen_async_config.py`

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
