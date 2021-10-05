#!/bin/bash

source ~/.bashrc

rosrun master_discovery_fkie master_discovery _mcast_group:=224.0.0.251 & rosrun master_sync_fkie master_sync >/dev/null 2>&1 &
