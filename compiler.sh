#/bin/bash/
sudo rmmod dm9051;
make clean;
make;
sudo make uninstall;
sudo make install;
#sudo dmesg --clear;
sudo modprobe dm9051;
