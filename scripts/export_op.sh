#!/bin/bash

export AUTOWARE_ROOT=/home/${USER}/autoware
export SCENARIO_RUNNER_ROOT=/home/${USER}/docker_share/git_pkgs/simulators/carla_awr/scenario_runner
export CARLA_ROOT=/home/${USER}/docker_share/git_pkgs/simulators/carla
export OP_BRIDGE_ROOT=/home/${USER}/docker_share/git_pkgs/simulators/carla_awr/op_bridge
export OP_AGENT_ROOT=/home/${USER}/docker_share/git_pkgs/simulators/carla_awr/op_agent
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/util
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla/agents
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg
