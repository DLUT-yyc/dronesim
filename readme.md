## dronesim

此库为airsim（https://github.com/Microsoft/Airsim）的二次封装版，对无人机飞行控制和图像处理的底层函数进行了更高层封装，精简后使基于视觉的无人机自主感知开发更为简便直接！

motion.py为封装后的控制代码文件，可通过向flyCmd函数传递不同参数完成起飞、直飞、定高飞等效果动作。使用了增量式PID控制算法，对pitch，roll，yaw和throttle进行更加稳定的闭环控制，实现了定高稳定飞行。

image.py为封装后的图像代码文件，调用其中函数可直接获得处理好的BGR（opencv格式）彩色图像或二维深度点阵图。

fly_by_ keyboard.py为用键盘按键控制无人机飞行的代码文件（需要用命令行运行）。

(另外请注意，由于airsim官方库在持续更新中，部分函数名、参数数量、弃用更新类和函数、setting文件等需额外注意同步）

