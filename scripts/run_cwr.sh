#!/bin/bash

${PHA_HOME}/docker_scripts/run-compose.sh \
    -b ${CARLAWARE_PHA}/envs/cwr-compose.yaml \
    -e ${CARLAWARE_PHA}/envs/op-user.env