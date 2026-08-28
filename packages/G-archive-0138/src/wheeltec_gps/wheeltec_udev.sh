echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0005", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_gps"' >/etc/udev/rules.d/wheeltec_gps.rules

service udev reload
sleep 2
service udev restart


