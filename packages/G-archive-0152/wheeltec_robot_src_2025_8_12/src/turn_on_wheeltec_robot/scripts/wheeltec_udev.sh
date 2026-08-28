#CP2102 串口号
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0001", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_laser1"' >/etc/udev/rules.d/wheeltec_laser1.rules
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0002", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_laser2"' >/etc/udev/rules.d/wheeltec_laser2.rules
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0003", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_controller"' >/etc/udev/rules.d/wheeltec_controller.rules

#CH9102 串口号
echo  'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0001", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_laser1"' >>/etc/udev/rules.d/wheeltec_laser1.rules
echo  'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0002", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_laser2"' >>/etc/udev/rules.d/wheeltec_laser2.rules
echo  'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0003", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_controller"'>>/etc/udev/rules.d/wheeltec_controller.rules

#CH9102，同时系统安装了对应驱动 串口号0004 设置别名为wheeltec_mic
echo  'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0004", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_mic"' >/etc/udev/rules.d/wheeltec_mic.rules
echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0004", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_mic"' >>/etc/udev/rules.d/wheeltec_mic.rules

echo  'SUBSYSTEM=="video4linux",ATTR{name}=="Integrated Webcam: Integrated W",ATTR{index}=="0",MODE:="0777",SYMLINK+="RgbCam"' >/etc/udev/rules.d/camera.rules
echo  'SUBSYSTEM=="video4linux",ATTR{name}=="Integrated Webcam",ATTR{index}=="0",MODE:="0777",SYMLINK+="RgbCam"' >>/etc/udev/rules.d/camera.rules
#echo  'SUBSYSTEM=="video4linux",ATTR{name}=="USB 2.0 Camera: USB Camera",ATTR{index}=="0",MODE:="0777",SYMLINK+="Astra_Gemini"' >>/etc/udev/rules.d/camera.rules
echo  'SUBSYSTEM=="video4linux",ATTRS{idProduct}=="0511",ATTRS{serial}=="AY27833003A",MODE:="0777",SYMLINK+="Astra_Gemini_car"' >/etc/udev/rules.d/wheeltec_gemini.rules
echo  'SUBSYSTEM=="video4linux",ATTRS{idProduct}=="0511",ATTRS{serial}=="AY0H8011058",MODE:="0777",SYMLINK+="Astra_Gemini_arm"' >>/etc/udev/rules.d/wheeltec_gemini.rules


service udev reload
sleep 2
service udev restart