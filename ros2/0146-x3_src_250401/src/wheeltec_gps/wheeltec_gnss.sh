echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_gnss"' >/etc/udev/rules.d/wheeltec_gnss.rules
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0005", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_gnss"' >/etc/udev/rules.d/wheeltec_gps.rules

service udev reload
sleep 2
service udev restart


