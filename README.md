# Rocos_DragTeach-in
借助力矩传感器实现机械臂关节拖动示教；
测试版代码-尚未整理。
力矩传感器的曲线拟合：https://rocos-sia.feishu.cn/docx/JLqpdVNRloWoZ0xMQLVcVdoOnAc?from=from_copylink
## 1. 机械臂示教
### 1.1 机械臂示教原理
```bash
基于导纳控制的力拖动示教核心伪代码
外力=力矩传感器的数值-动力学理论值
If （外力<力死区阈值）//目标力为0
{
    acc=(0-B*Vel)/M; // B：阻尼；M ：质量
    vel=vel+vel*dt;
    pose=pose+vel*dt;
}
else               //目标力为估计外力
{
    acc=(外力-B*Vel)/M;
    vel=vel+vel*dt;
    pose=pose+vel*dt;
}
if(abs(pose-lastpose)<位置阈值)
{
    vel=0;
    pose=lastpose;            //机械臂停止
}
```
### 1.2 代码备注
1. Rocos_7dof_109是10月9号调整机械臂拖动示教的版本代码。其中demo_teach_stop.cpp是最终的调试版本，其中的参数都为最终调试的参数。demo_teach_stop.cpp中的代码为机械臂示教的核心代码，其余的代码为机械臂的不同调试参数代码。
2. Rocos_example_onejoint是单关节的拖动示教代码，demo_torque_servoj_2.cpp是最终的调试版本，其中的参数都为最终调试的参数。

## 2. 力矩传感器
### 2.1 力矩传感器标定
1. demo_torque_7joint.cpp为重新标定力矩传感器后的代码，结合matlab代码使用.


