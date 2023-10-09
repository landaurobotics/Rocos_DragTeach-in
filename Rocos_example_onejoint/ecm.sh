#!/bin/bash
#sleep 3&&(cd /home/sia/YuLin_lab/src/DucoCobotAPI_py&&python3 demo.py)

# if [ "$?" = 0 ]; then

cd /opt/rocos/ecm/bin
sudo sh initECM.sh
sudo ./rocos_ecm -flagfile ../config/ecm.flagfile

# fi
