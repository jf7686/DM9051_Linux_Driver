#/bin/bash
sudo dtc -I dts -O dtb -o $1.dtbo $1.dts;
sudo mv $1.dtbo /boot/overlays;
exit 0
