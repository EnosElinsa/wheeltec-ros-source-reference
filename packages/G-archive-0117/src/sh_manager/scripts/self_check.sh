#!/bin/bash
############################## Devices 	List ##############################
car_list=(mini_akm senior_akm top_akm_bs top_akm_dl
           senior_mec_bs senior_mec_dl top_mec_bs top_mec_dl senior_mec_EightDrive top_mec_EightDrive
           flagship_mec_bs flagship_mec_dl
           mini_omni senior_omni,top_omni 
           mini_4wd senior_4wd_bs senior_4wd_dl top_4wd_bs top_4wd_dl flagship_4wd_bs flagship_4wd_dl 
           mini_tank mini_diff senior_diff four_wheel_diff_bs four_wheel_diff_dl flagship_four_wheel_diff_dl flagship_four_wheel_diff_bs 
           brushless_senior_diff
           mini_tank_moveit_four mini_4wd_moveit_four mini_mec_moveit_four
           mini_mec_moveit_six mini_4wd_moveit_six)

camera_list=(Astra_Pro Astra_Dabai Astra_Gemini Dabai_DCW2 RgbCam)

lidar_list=(ls_M10_uart ls_M10P_PLUS_uart ls_M10_net 
	ls_M10P_net ls_M10P_PLUS_net ls_N10 ls_N10P ld_14 ld_19 
	rplidar_A1_A2 rplidar_A3 rplidar_S1 LS_16 LS_32)

car_mode=$(zenity --entry ${car_list[@]} --entry-text="mini_mec" \
--title="Car Model" --text="选择小车型号(下拉选择):" --width=300 --height=180)
camera_mode=$(zenity --entry ${camera_list[@]} --entry-text="Astra_S" \
--title="Camera Model" --text="选择相机型号(下拉选择):" --width=300 --height=180)
lidar_mode=$(zenity --entry ${lidar_list[@]} --entry-text="ls_M10P_uart" \
--title="Lidar Model" --text="选择雷达型号(下拉选择):" --width=300 --height=180)
################################# Param ######################################
VID="2bc5"


################################# 测试组修改 ######################################
if [ ${Lidar Model} == "LS_16" or "LS_32" ];then
	lidar_name=""
	
else 
	lidar_name="wheeltec_lidar"
fi
#lidar_name="wheeltec_lidar"
################################# 测试组修改 ######################################



usart_port_name="wheeltec_controller"
rospath=`rospack find turn_on_wheeltec_robot`
camera_file="$rospath/launch/wheeltec_camera.launch"
lidar_file="$rospath/launch/wheeltec_lidar.launch"
wheeltec_robot_file="$rospath/launch/turn_on_wheeltec_robot.launch"
############################## Modifi Launch ###################################
Modifi(){
  if [ -f "$camera_file" ];then
    content=$(cat "$camera_file")
    if [ ! -z  "$content" ] && [ ! -z  "$camera_mode" ]; then
      sed -i '1,4s/default=".*"/default=\"'$camera_mode'\"/' "$camera_file"
      if [[ "$camera_mode" =~ "RgbCam" ]]; then
      	sed -i '8,10s/default=".*"/default=\"true\"/' "$camera_file"
      else
      	sed -i '8,10s/default=".*"/default=\"false\"/' "$camera_file"
      fi
    else
      echo -e "\033[1;31mcamera_content or camera_mode is empty! \033[0m"
    fi
  else 
    echo -e "\033[1;31mcamera file is not exist! \033[0m"
  fi

  if [ -f "$lidar_file" ];then
    content=$(cat "$lidar_file")
    if [ ! -z  "$content" ] && [ ! -z  "$lidar_mode" ]; then
      sed -i '1,4s/default=".*"/default=\"'$lidar_mode'\"/' "$lidar_file"
    else
      echo -e "\033[1;31mlidar_content or lidar_mode is empty! \033[0m" 
    fi
  else
    echo -e "\033[1;31mlidar file is not exist! \033[0m" 
  fi

  if [ -f "$wheeltec_robot_file" ];then
    content=$(cat "$wheeltec_robot_file")
    if [ ! -z  "$content" ] && [ ! -z  "$car_mode" ]; then
      sed -i '1,4s/name=\"car_mode\"  default=".*"/name=\"car_mode\"  default=\"'$car_mode'\"/' "$wheeltec_robot_file"
    else
      echo -e "\033[1;31mcar_content or car_mode is empty! \033[0m" 
    fi
  else
    echo -e "\033[1;31mlidar file is not exist! \033[0m" 
  fi
}
############################## Close check ###############################################
Close_Lidar_Check(){
	ROS_PID=$(ps -ef | grep "wheeltec_lidar.launch" | grep -v "grep" | awk '{print $2}')
    TOPIC_PID=$(ps -ef | grep "rostopic" | grep -v "grep" | awk '{print $2}')
    if [ ! -z "$ROS_PID" ]; then
      kill -9 "$ROS_PID"
    fi
    if [ ! -z "$TOPIC_PID" ]; then
      kill -9 "$TOPIC_PID"
    fi
}

Close_Camera_Check(){
	ROS_PID=$(ps -ef | grep "wheeltec_camera.launch" | grep -v "grep" | awk '{print $2}')
    TOPIC_PID=$(ps -ef | grep "rostopic" | grep -v "grep" | awk '{print $2}')
    if [ ! -z "$ROS_PID" ]; then
      kill -9 "$ROS_PID"
    fi
    if [ ! -z "$TOPIC_PID" ]; then
      kill -9 "$TOPIC_PID"
    fi
}

Close_STM32_Check(){
	ROS_PID=$(ps -ef | grep "turn_on_wheeltec_robot.launch" | grep -v "grep" | awk '{print $2}')
    TOPIC_PID=$(ps -ef | grep "rostopic" | grep -v "grep" | awk '{print $2}')
    if [ ! -z "$ROS_PID" ]; then
      kill -9 "$ROS_PID"
    fi
    if [ ! -z "$TOPIC_PID" ]; then
      kill -9 "$TOPIC_PID"
    fi
}
################################ Stm32 Check ###############################################
Modifi
if [ -e "/dev/$usart_port_name" ]; then
  echo -e "\033[1;32mFound the Stm32 Serial port! usart_port_name:$usart_port_name \033[0m"
  if [ ! -z  "$car_mode" ]; then
  	dbus-launch gnome-terminal -- bash -c "roslaunch $wheeltec_robot_file"
	dbus-launch gnome-terminal -- bash -c "rostopic hz /odom /imu"
	zenity --question --text="检测是否正常?" --width=250 --height=150
	if [ $? -eq 0 ];then
	  Close_STM32_Check
	  stm32_result="√检测正常"
	else
	  Close_STM32_Check
	  stm32_result="x检测失败"
	  echo -e "\033[1;31m串口检测不通过! \033[0m"
	fi
  else
  	stm32_result="建议正确输入小车型号后重检!"
  	echo -e "\033[1;31m输入小车型号不正确或为空! \033[0m"
  fi
else
  zenity --error --text="未检测到Stm32串口连接！请检查串口是否正常插入或别名." --width=300 --height=150 --timeout=2
  stm32_result="x检测失败(请检查串口)"
  echo -e "\033[1;31m未检测到Stm32串口连接！请检查串口是否正常插入或别名. \033[0m"
fi
################################ Lidar Check ##################################################
if [ -e "/dev/$lidar_name" ]; then
  echo -e "\033[1;32mFound lidar device! lidar name:$lidar_name \033[0m"
  if [ ! -z  "$lidar_mode" ]; then
  	dbus-launch gnome-terminal -- bash -c "roslaunch $lidar_file"
	dbus-launch gnome-terminal -- bash -c "rostopic hz /scan"
	zenity --question --text="检测是否正常?" --width=250 --height=150 
	if [ $? -eq 0 ];then
	  Close_Lidar_Check
	  lidar_result="√检测正常"
	else
	  Close_Lidar_Check
	  lidar_result="x检测失败"
	  echo -e "\033[1;31m雷达检测不通过! \033[0m"
	fi
  else
  	lidar_result="建议正确输入雷达型号后重检!"
  	echo -e "\033[1;31m输入雷达型号不正确或为空! \033[0m"	
  fi
else
  zenity --error --text="未检测到雷达设备！请检查是否正常连接或别名." --width=300 --height=150 --timeout=2
  lidar_result="x检测失败(请检查雷达连接)"
  echo -e "\033[1;31m未检测到雷达设备: $lidar_mode! 请检查是否正常连接或别名. \033[0m"
fi
################################ Cemera Check ####################################################
if [ ! -z "$camera_mode" ] && [ "$camera_mode" != "RgbCam" ]; then #Orbbec Caemra
	for dev in /sys/bus/usb/devices/*; do
	  if [ -e "$dev/idVendor" ]; then
	    vid=$(cat "$dev/idVendor")
	    if [ "$vid" == "${VID}" ]; then
	      port=$(basename $dev)
	      product=$(cat "$dev/product" 2>/dev/null) 
	      serial=$(cat "$dev/serial" 2>/dev/null) 
	      echo -e "\033[1;32mFound Camera device $product, usb port $port, serial number $serial \033[0m"
	    fi
	  fi
	done
	if [ -z "$product" ] && [ -z "$serial" ];then
	  zenity --error --text="未检测到相机设备！请检查是否正常连接." --width=300 --height=150 --timeout=2
	  camera_result="x检测失败(请检查相机连接)" 
	  echo -e "\033[1;31m未检测到相机设备: $camera_mode! 请检查是否正常连接. \033[0m"  
	else
	  if [ "$camera_mode" == "Dabai_DCW2" ];then
	    dbus-launch gnome-terminal -- bash -c "roslaunch $camera_file"
	    dbus-launch gnome-terminal -- bash -c "rostopic hz /camera/color/image_raw /camera/depth/image_raw"
	  else
	  	dbus-launch gnome-terminal -- bash -c "roslaunch $camera_file"
	    dbus-launch gnome-terminal -- bash -c "rostopic hz /camera/rgb/image_raw /camera/depth/image_raw"
	  fi
	  zenity --question --text="检测是否正常?" --width=250 --height=150 
	  if [ $? -eq 0 ];then
		Close_Camera_Check
		camera_result="√检测正常"
	  else
		Close_Camera_Check
		camera_result="x检测失败" 
		echo -e "\033[1;31m相机检测不通过! \033[0m"
	  fi
	fi
elif [ ! -z "$camera_mode" ] && [ "$camera_mode" == "RgbCam" ]; then #RGB Caemra
	dbus-launch gnome-terminal -- bash -c "roslaunch $camera_file"
    dbus-launch gnome-terminal -- bash -c "rostopic hz /image_raw/compressed"
    zenity --question --text="检测是否正常?" --width=250 --height=150 
    if [ $? -eq 0 ];then
	  Close_Check
	  camera_result="√检测正常"
    else
	  Close_Check
	  camera_result="x检测失败" 
	  echo -e "\033[1;31m相机检测不通过! \033[0m" 
    fi	
else
	camera_result="建议正确输入相机型号后重检!"
	echo -e "\033[1;31m输入相机型号不正确或为空! \033[0m"	
fi
################################ Result ####################################################
zenity --list \
--title="Self-Check Result" \
--column="检查设备" --column="检查结果" \
"Stm32" "$stm32_result" \
"雷达" "$lidar_result" \
"相机" "$camera_result" --width=500 --height=300
#############################################################################################