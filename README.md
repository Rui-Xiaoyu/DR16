# DR16

## 1. 模块作用
DR16 遥控解析模块。将 DBUS 数据转换为 CMD 输入和事件。
Manifest 描述：Receiver parsing

## 2. 主要函数说明
1. ThreadDr16: UART 线程读取并分发遥控数据。
2. ParseRC: 解析原始帧并生成 CMD::Data。
3. CheckoutOffline / Offline: 离线检测与失控处理。
4. GetEvent: 对外暴露事件绑定入口。

## 3. 接入步骤
1. 添加模块并确保 uart_dr16 配置正确。
2. 把解析结果喂给 CMD。
3. 在 EventBinder 中绑定开关和按键事件。

标准命令流程：
    xrobot_add_mod DR16 --instance-id dr16
    xrobot_gen_main
    cube-cmake --build /home/leo/Documents/bsp-dev-c/build/debug --

## 4. 配置示例（YAML）
module: DR16
entry_header: Modules/DR16/DR16.hpp
constructor_args:
  - CMD: '@cmd'
  - task_stack_depth_uart: 2048
template_args:
[]

## 5. 依赖与硬件
Required Hardware:
- dr16
- dma
- uart

Depends:
[]

## 6. 代码入口
Modules/DR16/DR16.hpp
