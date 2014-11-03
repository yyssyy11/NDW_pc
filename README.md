##NDW_pc 

本项目目的是测试传感网数据。

编译之后，使用超级权限运行  
>./ndw_main *数据名字*

**注意**：	fd1 = open("/dev/ttyUSB0", O_RDWR|O_NONBLOCK);//打开串口

这里的"/dev/ttyUSB0"要与Bastation节点对应的串口一致。