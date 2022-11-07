SETUP_PATH=$(pwd)
cd /root/bf-sde-9.2.0/
source set_sde.bash; sde
cd $SETUP_PATH
sudo ./cp --install-dir /root/bf-sde-9.2.0/install/ --conf-file /root/bf-sde-9.2.0/build/p4-build/ur/tofino/ur/ur.conf
