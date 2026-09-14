# RoboOchard Deploy ROS2 Package

## Asynchronous trajectory handover

When trajectory stitching is enabled, the node solves for a future control
step and keeps publishing the old chunk until that step. The lead time is
the previous solve duration plus one control period, limited by the available
actions and `max_delay_horizon`. Inference waits while a solved handover is
pending, so its trajectory cannot be overwritten before the switch.

If the solve misses its planned step, the node retains the current chunk
instead of switching at a point that was not constrained by the solver.
With stitching disabled, responses continue to switch without this wait.

A moving handover must occur before the old chunk runs out. Once it is
exhausted, the next solve starts from its final joint-position commands with
zero velocity and acceleration, rather than extrapolating its old motion.
These are planned command states, not measurements of the robot's physical
position or velocity; downstream command limiting and robot tracking can
make them differ from the actual motion.

Short tails of one to three control steps use the same solver and keep
their requested starting index. The solver has its own integration grid;
there is no four-action-sample requirement and no need to move the start
backwards or skip stitching solely because the tail is short.
