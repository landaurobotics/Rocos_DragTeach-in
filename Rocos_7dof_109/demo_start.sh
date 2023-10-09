#!/bin/bash
#sleep 3&&(cd /home/sia/YuLin_lab/src/DucoCobotAPI_py&&python3 demo.py)

# if [ "$?" = 0 ]; then

cd ./build
cmake ..
make -j8
cd ..
sudo ./build/bin/demo


# fi
