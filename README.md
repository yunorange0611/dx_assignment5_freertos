# 电控组第五次作业 - FreeRTOS
## 任务要求
1. 学习FreeRTOS相关内容
2. 阅读工程模板（电机底层），学习其CAN通信，电机驱动等内容的实现，使用markdown编写你的阅读笔记，无字数要求。以下给出一些可以包括的内容供参考：
   1. 模板中你之前不了解的c/c++功能（如函数指针，class类等）
   2. 模板中关于FreeRTOS的使用
   3. 相比自己电机任务/整车调试中的代码，这份模板相对于你的代码有什么优点
   4. 如何使用这个模板
3. 在模板基础上编写FreeRTOS任务：
   1. LED流水灯（所有参数自己定，只要是流水灯就行）
   2. 陀螺仪移植
   3. 电机控制任务，用**位置环**让3508电机不断转动，其速度是正弦函数（PID参数不重要，能转就行）
   4. 自定义CAN通信任务，接收校内赛需要使用的功率计信息
   5. 遥控器接收与数据解算
4. 校内赛的代码可以并建议使用这个模板
5. 作业第一次提交截止时间11.8日，把工程提交到自己的github上，并将仓库网址发到1459054506@qq.com

## 关于AGENT使用
1. 模板中有一个AGENTS.md文件，它会让你的agent不直接帮你写代码，而是指导你自己怎么写。
2. 禁止修改，删掉，删掉后再加回AGENTS.md的行为（防君子不防小人）
3. 如发现存在绕过AGENTS.md使用AI的行为，此次作业为0分并打回重做，若11.15前未通过这次作业，降一档录取（正式——梯队，梯队——取消入队资格）


## 在这里编写你的阅读笔记
我把每行注释加代码里了
- 这块把can的初始化init放在main里面啊，我习惯把init在前面包好了在这就写一行，不过这不重要
### Class是啥
- 第一次接触类，以前都是用结构体的。class是设计图object是实物。
- 我看不懂。高级语言太烦了
- class分三类数据：public可以被内外调用，private只能内部调用，protect可以本类和派生类调用。
- 比起struct：
- 一块把对象数据和需要它的函数写进去可以不用在写函数的时候用指针指向这个数据。
- 再一个，设定了三个p之后可以避免不想被改的数据被改了
- 然后可以写同名函数
- 其实还是没太懂，没准用用就懂了。

### 1. motor_driver.h
- 职责：定义了一个class作为所有电机的公共接口，把可能需要的几个数据都放进去了。public上层调用，处理过的数据。protect自己用。

- 对比 project4：`motor_3508.c` / `motor_dm.c` 的全局结构体 + 自由函数，好像是可以少写几遍，而且分了内外。

### 2. motor_driver.cpp
- 职责：
1. 把几个类别的接受和发送的取反写了包装好后面需要来调用。
2. 后面把虚函数都写一下，保证不会报错，不能只生命不定义
- 对比 project4：方向系数散在控制层（`FEEDER_DIR` 等），不用每次都乘一下，调用改好的forward就行了。
- 在链里的位置：保证符号永远适配。

### 3. pid_driver.h
- 职责：提供pid的参数包（struct），把pid参数和需要的函数打包（class)。把几个重要的调试用量打包成protect。
- 注意事项：构造参数顺序是 (max_out, integral_limit, kp, ki, kd)，第一个不是 kp。
  例如 DJI_motor 里：
velPid(0.5f, 0.1f, 10, 0.002f, 0.1f);
意思是：最大输出 0.5、积分限幅 0.1、kp=10、ki=0.002、kd=0.1。
pid_reset(...)：名字叫 reset，但它主要是重设增益和限幅，不是清空历史误差。真正清积分的是 pid_clear()，而且 pid_clear() 只清 iout，不动增益和限幅。
构造函数有默认参数，所以可以写 pid p;，但模板里实际都显式传参。
- 在链里的位置：`DJI_motor` 的位置环/速度环pid velPid/pid posPid;

### 4. pid_driver.cpp
- 职责：进行pid计算，积分限幅、输出限幅。
- 计算流程：pid_calculate 的流程：误差 → P → I 累加 → D 误差差分 → 积分限幅 → 三项相加 → 输出限幅；
  没有 dt：iout += ki * err、dout = kd * (err - last_err) 都按“每次调用”算
  pid_reset 是改参数，pid_clear 只清积分 iout
- C/C++ 点：成员实际初始化顺序按类里声明的顺序，不是按初始化列表写的顺序。pid 类里 enable、set、get 等声明在 param 前面，所以实际先初始化它们，再初始化 param。
- 对比 project4：project4 有 dt、微分测量、低通、积分分离、抗饱和。我感觉我原来那个更好啊，没有dt按照每次调用做积分微分不是容易出现调用时间不一样的问题吗？我感觉单独配一个来计算更好吧。
- 咋用：感觉pid计算逻辑还是用我原来的。然后结构按照5这个改一下
- 修改原本pid思路：
 1. 参数和状态分开：pid_param 放 kp/ki/kd/限幅，类的私有成员放 err/last_err/iout。project4 的 PID_TypeDef 把两者混在一个结构体里，调参和复位不够清晰。
 2. 每个轴/电机持有自己的 PID：project5 的 DJI_motor 里是 velPid、posPid 成员；project4 是全局 pid_yaw_pos、pid_pitch_spd 等。对象化后不容易把“yaw 的 PID”传给“pitch”。
 3. 构造函数保证初始化：project5 的 pid 构造时把参数和状态都设好；project4 依赖每个地方记得调 PID_Init。
 4. pid_reset 和 pid_clear 分工：project5 里 pid_reset 改参数、pid_clear 只清积分；project4 的 PID_Reset 是整体清状态，没有“只清积分”的接口。
 5. 方法绑定对象：pid.pid_calculate(...) 比 PID_Calculate(&pid, ...) 少一个指针参数，少一类传错指针的错误。

### 5. CAN_bsp.h
- 职责：作为can的接口清单
- 关键结构/函数：
1. `can_device_transmit_member`CAN 句柄、ID、长度、数据指针。放一个结构体
2. class`can_device_transmit`一个“发”的对象，保存一条待发消息的参数。
3. class`can_device_receive`一个“收”的对象，对应一个 CAN ID / 一个设备。
4. `can_rx_callback`、
5. `extern "C"`extern "C" 让这些函数使用 C 的链接名，不做 C++ 的 name mangling。
   原因：
   1. can1_sending_service 等任务入口是 freertos.c（C 文件）按名字创建的；
   2. HAL_CAN_TxMailboxXCompleteCallback 等是 HAL 里的弱 C 函数，要在这里覆盖。
- 对比 project4：每个接收设备一个 can_device_receive 对象，自己注册过滤器和回调； 每个发送源一个 can_device_transmit 对象，自己保存参数并入队；加电机只用加对象然后传给回调，不用再回调里面写

### 6. CAN_bsp.cpp
- 职责：进行can的发送和接收
- 关键函数：
    - 发送：
      - 弄了两个表，一个专门放设备，还有一个放过滤器id，还有一个专门记数用来填的。
      - `can_device_transmit`member 里存 CAN 句柄、ID、长度、数据指针。构造函数只负责填这张单，真正的发送动作在 can_send_msg 里。
      - `can_send_msg`按 member.hcan 选 CAN1 还是 CAN2 的队列，用 osMessageQueuePut 把整个 member（16 字节）拷贝进队列。它只入队，不碰 HAL；超时参数是 0，队列满就丢帧。
      - `can1/2_sending_service`两个 FreeRTOS 任务。死循环里先 osSemaphoreAcquire 拿一个邮箱 token（共 3 个），再 osMessageQueueGet 取一条消息，填好 CAN 头，调 HAL_CAN_AddTxMessage 交给硬件。CAN2 版本只是换了队列、信号量、句柄。
      - `can_tx_complete_callback`发送完成中断里的回调。硬件发完一个邮箱，HAL 会调 HAL_CAN_TxMailbox0/1/2CompleteCallback，这三个都转到它；它按 hcan 释放对应的计数信号量 token，让发送任务能继续发下一帧。
    - 接收：
      - 全局表：每条 CAN 14 个 bank × 每 bank 4 个 ID = 最多 56 个设备；can_device_rx_list 存对象指针。
      - 构造函数：index / 4 选 bank，index % 4 选槽位，<< 5 把标准 ID 放进 16 位字段的 bit5～bit15；最后 HAL_CAN_ConfigFilter 写硬件。
      - 中断：HAL_CAN_GetRxMessage 取帧 → FilterMatchIndex 查设备表 → memcpy → dev->callback(dev, rx_data)。
      - `HAL_CAN_RxFifo0/1MsgPendingCallback`：帧到了，进哪个中断函数；
        `FilterMatchIndex`：这帧属于哪个设备槽位
- 对比 project4：
    - project4：掩码全收 + `main.c` 中央分发 + 直接发送 + 诊断统计
    - project5：IDLIST 过滤 + 每设备回调 + 队列/信号量发送。project5 是异步发： 控制任务只写组缓冲； DJI_motor_service 入队； can1_sending_service 等 token 再交给 HAL。
- 优劣：
  - 发送
    - project5：控制任务不被 CAN 阻塞，队列能吸收 16 帧的突发；但多一个任务、一次 16 字节拷贝，最坏多等 1 ms。
    - project4：延迟低、代码直接；但控制任务要进临界区，邮箱满时立刻丢帧。
    - project4 有 tx_drop/tx_err 统计；project5 没有，丢了也不知道。
  - 接收：
    - project5：加设备 = 构造对象；BSP 和业务解耦；硬件过滤减少中断里的软件分发。但每条 CAN 最多 56 个设备，过滤器配置是全局可变状态，memcpy 判空顺序有问题
    - project4：加设备 = 在中央回调加一个 else if；没有 56 个槽位限制；所有 ID 在一个地方看得见。但每帧都要进软件分发，BSP 和业务 ID 耦合，中央函数会越来越长。
    - 感觉结构可以用这个，加设备更方便，不过要加上我原本代码里面那一堆掉线保护和检查。这个把控制和发送分开了，确实更好。接收是硬件过滤和软件按照对象回调。
    - 拼帧的时候各自写，更适合用多种类型电机。
### 7. DJI_motor_driver.h
- 职责：
 - “电机 + CAN + PID”的拼装说明书。把需要的量都用结构体打包了，需要的量和函数用class打包了。
  - dji_motor_data 是一个电机的全部反馈状态；
  - DJI_motor : public motor_dev，把 CAN 收发和两个 PID 组合进对象；
  - extern "C" 的 DJI_motor_service 是 C 生成代码调用的任务入口。
 - DJI_motor_service这是 1 ms 发送任务的入口，实现在对应c文件里。
- 对比 project4：project5 是一个类，把“状态 + CAN + PID + 命令”都绑在一个对象上。 加电机类型 = 继承；每个电机自己的状态和 PID 不会串；方向处理在基类。


### 8. DJI_motor_driver.cpp
- 职责：工程的“电机总装”：构造函数、接收解析、控制接口、发送服务都在这里
- 关键函数：
    - 构造函数：初始化顺序、CAN ID 公式、50 帧等待、选发送组
    - 回调：`container_of`、编码器过零点、`total_round`
    - 控制：`motor_set_current/speed/rounds` 的串级
    - `motor_reset` / `motor_control` / `DJI_motor_service`
- C/C++ 点：实际初始化顺序按成员声明顺序，不按列表顺序。can_rx 的构造会注册过滤器，can_tx 只是保存发送 ID，velPid/posPid 构造 PID 对象。列表顺序和声明顺序不一致，
- 对比 project4：
    - 把“初始化、注册 CAN、等反馈、选发送组”都塞进了构造函数，所以对象创建完就处于可发送状态
    - 字节提取和过零点算法基本一样；project4 的 norm_total_angle 是输出轴圈数（除了减速比），project5 的 total_round 是转子圈数
    - 不阻塞	每个对象构造函数，等 50 帧
    - 如果要调多电机的pid，在构造后改 param，或者把参数加到构造函数里。

### 9. 串联
#### 9.1 发送链
`motor_set_current` → 组缓冲 → `DJI_motor_service` → 队列 → `can1_sending_service` → `HAL_CAN_AddTxMessage` → 完成中断 → 信号量

#### 9.2 接收链
CAN 中断 → `HAL_CAN_RxFifo0/1MsgPendingCallback` → `FilterMatchIndex` → `dev->callback` → `container_of` → `motor_data`

#### 9.3 控制链
`motor_set_rounds` → `posPid` → `motor_set_speed` → `velPid` → `motor_set_current`

#### 9.4 初始化链
`DJI_motor` 构造 → `can_rx` 注册过滤器 → `can_tx` 选组 → PIDs 构造 → 等 50 帧 → `enable=1`
