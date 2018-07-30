# _explorer\_launch_

_explorer_ 机器人的启动文件

本包集中保存所有的启动文件,旨在减少启动时需要输入的命令量

在 _start\_explorer\_client.launch_ 中包含所有需要在控制台使用的节点的启动命令,其中必须包括:
- 手柄的读取文件
- 一些计算量过大的程序(避免降低工控机的性能)

在 _start\_explorer\_server.launch_ 中包含所有需要在工控机上运行的节点的启动命令,其中必须包括:
- 底层串口通信的节点
- 在机器人上直接链接工控机的传感器的驱动节点
- 实时控制安全保护的相关程序(保证安全第一)

在 _start\_explorer\_test.launch_ 中包含以上两个 _launch_ 的功能,用于直接通过 PC 调试机器人