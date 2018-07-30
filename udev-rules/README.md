# rules 说明

这几个规则需要复制到`/etc/udev/rules.d `文件夹中

- **51-usb-flir-one.rules**

  这个规则是`flir_one_node`使用的，要想运行节点，之前必须添加这个规则

  ```bash
  SUBSYSTEMS=="usb", ATTRS{idVendor}=="09cb", ATTRS{idProduct}=="1996", MODE:="0666" OWNER="aicrobo"
  ```

  

- **99-camtest.rules**

  这个规则是给摄像头用的……



- **20-usb-serials.rules**

  给串口默认777权限

  ```bash
  KERNEL=="ttyUSB*"  MODE="0777" 
  KERNEL=="ttyS*"  MODE="0777" 
  KERNEL=="ttyACM*"  MODE="0777" 
  ```

  

- **serial.rules**

  读取底盘串口和思岚激光雷达的硬件ID，然后分别映射到`serial`和`radar`，这样子就不受插入顺序的影响

  ```bash
  KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE:="0777", SYMLINK+="serial"
  KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="radar"
  ```

  