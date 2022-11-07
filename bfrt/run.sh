SETUP_PATH=$(pwd)
cd /p4sde/bf-sde-9.0.0/
source set_sde.bash; sde
./run_bfshell.sh -b $SETUP_PATH/setup.py 