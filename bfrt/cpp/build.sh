SETUP_PATH=$(pwd)
cd /root/bf-sde-9.2.0/
source set_sde.bash; sde
cd $SETUP_PATH
make
