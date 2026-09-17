The default HoloBrain teleoperation setup targets ALOHA hardware. Pico VR
uses a separate DAgger entry point.

| Script | Purpose |
| --- | --- |
| `aloha_dagger.sh` | ALOHA DAgger control through the global manager. |
| `pico_dagger.sh` | Pico VR DAgger control through the global manager. |

Both launches start exactly one Control Manager. To customize topics, reset
scope, or replay behavior, pass `control_manager_config_file:=<path>` with a
validated Control Manager configuration.

For the default generated configuration, edit the command topics in
`../inference/control_channels.py` and the `enable_services`, `reset_services`,
`inference_disable_services`, `inference_node_candidates`, and `replay_time_s`
values in `gen_control_manager_config.py`. The `aloha` and `pico` branches list their
hardware service scopes explicitly. Model-server fields belong to the
inference configuration, not the Manager's runtime logic.

`REPLAY_TIME_S` is no longer a launch override and is ignored by these
scripts. Set `replay_time_s` in the selected Control Manager configuration
instead.

## Reset with an optional Deploy node

The default Manager configuration lists the fully qualified sync and async
Deploy node names in `inference_node_candidates`. These are alternative names
for the separately launched inference participant, not nodes the Manager
starts itself. Update them when changing the Deploy node name or namespace.

On every accepted reset, Manager first enters `RESETTING` and gates commands.
It then checks the currently discovered nodes and inference-disable services
without waiting for Deploy to appear. If neither a candidate node nor a
configured disable service is discovered, it logs the skip and immediately
proceeds to the configured hardware resets.
If either is discovered, **all** `inference_disable_services` must succeed,
even if inference is already paused. An unavailable service, failed response,
or response timeout blocks hardware reset. The Manager ends in `STOP` on both
success and failure; it does not restart inference or resume takeover.
`service_wait_timeout_s` still applies when waiting for a required service,
not when checking whether optional Deploy is present.

An empty or omitted `inference_node_candidates` list keeps all configured
disable services mandatory, preserving existing Manager configurations. Use
that strict setting when Deploy must be present. Discovery only describes the
local ROS graph; it cannot prove a remote process is stopped during a network
partition or before discovery has completed. Do not start Deploy concurrently
with reset; use strict configuration if its presence must be guaranteed.
Presence is checked again on each reset, so a later-started Deploy is not
permanently skipped. Configuring candidate names requires at least one
disable service.

This logic belongs to Manager, so reset behaves the same whether requested by
the App, Pico, keyboard, or another service client. Regenerate the Manager
configuration and restart the Manager to apply these settings.
