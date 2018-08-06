# explorer_ui

基于QT4编写的用于连接和调试机器人的界面。

建议使用ssh-keygen配置免密登录以方便使用。

### 本地调试
在登陆界面，用户名和地址为空进入，可以进入本地调试模式。本地调试模式为所有的节点和功能都在本地电脑上运行。

### 远程登录
用用户名和远程工控机地址登录。通过界面启动各个功能。

安装编译 可能缺少qt4 package
`sudo apt-get install ros-kinetic-qt-build`

使用
`rosrun explorer_ui explorer_ui`
