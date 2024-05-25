#!/bin/bash
#Author: David Hu
#Date: 2022-02
floder_name=$1
null_name=" "
if [ ${floder_name} -eq ${null_name} ];then
    echo "please input \"./ws_deploy.sh floder_name\""
else
    floder_name="${floder_name}_`date +%Y%m%d-%H-%M`"
    mkdir ${floder_name}
    cp ./*.md ./${floder_name}
    cp ./*.txt ./${floder_name}
    cp ./*.xml ./${floder_name}
    cp ./LICENSE ./${floder_name}
    cp ./launch ./${floder_name} -a
    cp ./rviz  ./${floder_name} -a
    cp ./src ./${floder_name} -a
    cp ./include ./${floder_name} -a
    cp ./scripts ./${floder_name} -a
    zip -r ${floder_name}.zip ./${floder_name}
    rm -rf ./${floder_name}/
    echo "create ./${floder_name}.zip "
fi