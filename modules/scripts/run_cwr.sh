#!/bin/bash

${PHA_HOME}/docker_scripts/run-compose.sh \
    -b ${CARLAWARE_PHA}/modules/envs/cwr-compose.yaml \
    -e ${CARLAWARE_PHA}/modules/envs/op-user.env