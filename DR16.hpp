#pragma once

/* clang-format off */
/* === MODULE MANIFEST ===
module_name: DR16
module_description: Receiver parsing
constructor_args:
  - cmd: '@CMD'
  - task_stack_depth_uart: 2048
  - cmd_data_tp_name: "cmd_raw_data_"
required_hardware: dr16 dma uart
=== END MANIFEST === */
// clang-format on

#include "CMD.hpp"
#include "app_framework.hpp"
#include "uart.hpp"


/**
 * @brief DR16遥控器通道值范围定义
 */
#define DR16_CH_VALUE_MIN (364u)  /* 通道最小值 */
#define DR16_CH_VALUE_MID (1024u) /* 通道中间值 */
#define DR16_CH_VALUE_MAX (1684u) /* 通道最大值 */

/**
 * @class DR16
 * @brief DR16遥控器数据解析类
 * @details 负责接收和解析DR16遥控器的数据，包括摇杆���拨杆和按键等信息
 */
class DR16 : public LibXR::Application {
 public:
  /**
   * @brief 控制源枚举
   */
  enum class ControlSource : uint8_t {
    DR16_CTRL_SOURCE_SW = 0x00,
    DR16_CTRL_SOURCE_MOUSE = 0x01,
  };

  /**
   * @brief 拨杆开关位置枚举
   */
  enum class SwitchPos : uint8_t {
    DR16_SW_L_POS_TOP = 0x00,
    DR16_SW_L_POS_BOT = 0x01,
    DR16_SW_L_POS_MID = 0x02,
    DR16_SW_R_POS_TOP = 0x03,
    DR16_SW_R_POS_BOT = 0x04,
    DR16_SW_R_POS_MID = 0x05,
    DR16_SW_POS_NUM
  };

  /**
   * @brief 按键枚举
   */
  enum class Key : uint8_t {
    KEY_W = static_cast<uint8_t>(SwitchPos::DR16_SW_POS_NUM),
    KEY_S,
    KEY_A,
    KEY_D,
    KEY_SHIFT,
    KEY_CTRL,
    KEY_Q,
    KEY_E,
    KEY_R,
    KEY_F,
    KEY_G,
    KEY_Z,
    KEY_X,
    KEY_C,
    KEY_V,
    KEY_B,
    KEY_L_PRESS,
    KEY_R_PRESS,
    KEY_L_RELEASE,
    KEY_R_RELEASE,
    KEY_NUM,
  };

  /**
   * @brief 计算Shift组合键的编码值
   * @param key 基础按键
   * @return Shift组合键的编码值
   */
  constexpr uint32_t ShiftWith(Key key) {
    return static_cast<uint8_t>(key) + 1 * static_cast<uint8_t>(Key::KEY_NUM);
  }

  /**
   * @brief 计算Ctrl组合键的编码值
   * @param key 基础按键
   * @return Ctrl组合键的编码值
   */
  constexpr uint32_t CtrlWith(Key key) {
    return static_cast<uint8_t>(key) + 2 * static_cast<uint8_t>(Key::KEY_NUM);
  }

  /**
   * @brief 计算Shift+Ctrl组合键的编码值
   * @param key 基础按键
   * @return Shift+Ctrl组合键的编码值
   */
  constexpr uint32_t ShiftCtrlWith(Key key) {
    return static_cast<uint8_t>(key) + 3 * static_cast<uint8_t>(Key::KEY_NUM);
  }

  constexpr uint32_t RawValue(Key key) {
    return 1 << static_cast<uint8_t>(Key::KEY_NUM);
  }

  typedef struct __attribute__((packed)) {
    uint16_t ch_r_x : 11;
    uint16_t ch_r_y : 11;
    uint16_t ch_l_x : 11;
    uint16_t ch_l_y : 11;
    uint8_t sw_r : 2;
    uint8_t sw_l : 2;
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t press_l;
    uint8_t press_r;
    uint16_t key;
    uint16_t res;
  } Data;

  /**
   * @brief DR16构造函数
   * @param hw 硬件容器引用
   * @param app 应用管理器引用
   * @param cmd 控制命令对象引用
   * @param task_stack_depth_uart UART任务栈深度
   * @param cmd_data_tp_name CMD数据主题名称
   */
  DR16(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app, CMD& cmd,
       uint32_t task_stack_depth_uart, const char* cmd_data_tp_name)
      : cmd_(&cmd),
        uart_(hw.Find<LibXR::UART>("uart_dr16")),
        sem(0),
        op(sem),
        cmd_data_tp_(cmd_data_tp_name, sizeof(CMD::Data)) {
    uart_->SetConfig({100000, LibXR::UART::Parity::EVEN, 8, 1});

    /* 注册CMD到正确的主题 */
    cmd_->RegisterController<CMD::Data>(cmd_data_tp_);

    /* 创建UART线程 */
    thread_uart_.Create(this, Thread_Dr16, "uart_dr16", task_stack_depth_uart,
                        LibXR::Thread::Priority::HIGH);
    app.Register(*this);
  }

  /**
   * @brief 监控函数重写
   */
  void OnMonitor() override {}

  /**
   * @brief DR16 UART读取线程函数
   * @param dr16 DR16实例指针
   */
  static void Thread_Dr16(DR16* dr16) {
    dr16->uart_->read_port_->Reset();

    while (true) {
      dr16->uart_->Read(dr16->data_, dr16->op);
      if (dr16->DataCorrupted()) {
        dr16->uart_->read_port_->Reset();
        LibXR::Thread::Sleep(3);
      } else {
#ifdef LIBXR_DEBUG_BUILD
        dr16->DataviewToData(dr16->data_view_, dr16->data_);
#endif
        dr16->PraseRC();
      }
    }
  }

  /**
   * @brief 检查接收数据是否损坏
   * @return true 数据损坏，false 数据正常
   */
  bool DataCorrupted() {
    /* 检查各通道值是否在有效范围内 */
    if ((this->data_.ch_r_x < DR16_CH_VALUE_MIN) ||
        (this->data_.ch_r_x > DR16_CH_VALUE_MAX)) {
      return true;
    }
    if ((data_.ch_r_y < DR16_CH_VALUE_MIN) ||
        (data_.ch_r_y > DR16_CH_VALUE_MAX)) {
      return true;
    }
    if ((data_.ch_l_x < DR16_CH_VALUE_MIN) ||
        (data_.ch_l_x > DR16_CH_VALUE_MAX)) {
      return true;
    }
    if ((data_.ch_l_y < DR16_CH_VALUE_MIN) ||
        (data_.ch_l_y > DR16_CH_VALUE_MAX)) {
      return true;
    }
    /* 检查拨杆值 */
    if (data_.sw_l == 0) {
      return true;
    }

    if (data_.sw_r == 0) {
      return true;
    }

    return false;
  }

  /**
   * @brief 解析遥控器数据并转换为控制命令
   */
  void PraseRC() {
    if (DataCorrupted()) {
      uart_->read_port_->Reset();
      LibXR::Thread::Sleep(3);
      return;
    }

    /* 初始化命令数据 */
    cmd_data = CMD::Data();

    /* 检测拨杆开关状态变化 */
    if (this->data_.sw_l != this->last_data_.sw_l) {
      this->event_.Active(static_cast<uint32_t>(SwitchPos::DR16_SW_L_POS_TOP) +
                          this->data_.sw_l - 1);
    }

    if (this->data_.sw_r != this->last_data_.sw_r) {
      this->event_.Active(static_cast<uint32_t>(SwitchPos::DR16_SW_R_POS_TOP) +
                          this->data_.sw_r - 1);
    }

    uint32_t tmp = 0;

    /* 检测组合键 */
    /* 检测Shift键 */
    if (this->data_.key & RawValue(Key::KEY_SHIFT)) {
      tmp += static_cast<uint32_t>(Key::KEY_NUM);
    }

    /* 检测Ctrl键 */
    if (this->data_.key & RawValue(Key::KEY_CTRL)) {
      tmp += 2 * static_cast<uint32_t>(Key::KEY_NUM);
    }

    /* 检测按键变化 */
    for (int i = 0; i < 16; i++) {
      if ((this->data_.key & (1 << i)) && !(this->last_data_.key & (1 << i))) {
        this->event_.Active(static_cast<uint32_t>(Key::KEY_W) + i + tmp);
      }
    }

    /* 控制模式切换 */
    if (((this->data_.key & (RawValue(Key::KEY_SHIFT) |
                             RawValue(Key::KEY_CTRL) | RawValue(Key::KEY_Q))) ==
         (RawValue(Key::KEY_SHIFT) | RawValue(Key::KEY_CTRL) |
          RawValue(Key::KEY_Q)))) {
      this->ctrl_source_ = ControlSource::DR16_CTRL_SOURCE_SW;
    }

    /* 切换到键鼠控制模式 */
    if (((this->data_.key & (RawValue(Key::KEY_SHIFT) |
                             RawValue(Key::KEY_CTRL) | RawValue(Key::KEY_E))) ==
         (RawValue(Key::KEY_SHIFT) | RawValue(Key::KEY_CTRL) |
          RawValue(Key::KEY_E)))) {
      this->ctrl_source_ = ControlSource::DR16_CTRL_SOURCE_MOUSE;
    }

    constexpr float FULL_RANGE =
        static_cast<float>(DR16_CH_VALUE_MAX - DR16_CH_VALUE_MIN);

    /* 键鼠控制模式 */
    if (this->ctrl_source_ == ControlSource::DR16_CTRL_SOURCE_MOUSE) {
      /* 处理鼠标按键事件 */
      if (this->data_.press_l && !this->last_data_.press_l) {
        this->event_.Active(static_cast<uint32_t>(Key::KEY_L_PRESS));
      }
      if (this->data_.press_r && !this->last_data_.press_r) {
        this->event_.Active(static_cast<uint32_t>(Key::KEY_R_PRESS));
      }
      if (!this->data_.press_l && this->last_data_.press_l) {
        this->event_.Active(static_cast<uint32_t>(Key::KEY_L_RELEASE));
      }
      if (!this->data_.press_r && this->last_data_.press_r) {
        this->event_.Active(static_cast<uint32_t>(Key::KEY_R_RELEASE));
      }

      /* 处理底盘控制 */
      if (this->data_.key & RawValue(Key::KEY_A)) {
        cmd_data.chassis.x -= 0.5;
      }
      if (this->data_.key & RawValue(Key::KEY_D)) {
        cmd_data.chassis.x += 0.5;
      }
      if (this->data_.key & RawValue(Key::KEY_S)) {
        cmd_data.chassis.y -= 0.5;
      }
      if (this->data_.key & RawValue(Key::KEY_W)) {
        cmd_data.chassis.y += 0.5;
      }

      /* 加速处理 */
      if (this->data_.key & RawValue(Key::KEY_SHIFT)) {
        cmd_data.chassis.x *= 2;
        cmd_data.chassis.y *= 2;
      }

      cmd_data.chassis.z = 0.0f;

      /* 云台控制 - 鼠标模式 */
      cmd_data.gimbal.pit =
          -static_cast<float>(this->data_.y) / 32768.0f * 1000.0f;
      cmd_data.gimbal.yaw =
          -static_cast<float>(this->data_.x) / 32768.0f * 1000.0f;
    }
    /* 遥控器控制模式 */
    else if (this->ctrl_source_ == ControlSource::DR16_CTRL_SOURCE_SW) {
      /* 底盘控制 */
      cmd_data.chassis.x =
          2 * (static_cast<float>(this->data_.ch_l_x) - DR16_CH_VALUE_MID) /
          FULL_RANGE;
      cmd_data.chassis.y =
          2 * (static_cast<float>(this->data_.ch_l_y) - DR16_CH_VALUE_MID) /
          FULL_RANGE;
      cmd_data.chassis.z =
          -2 * (static_cast<float>(this->data_.ch_r_x) - DR16_CH_VALUE_MID) /
          FULL_RANGE;

      /* 云台控制 */
      cmd_data.gimbal.yaw =
          -2 * (static_cast<float>(this->data_.ch_r_x) - DR16_CH_VALUE_MID) /
          FULL_RANGE;
      cmd_data.gimbal.pit =
          2 * (static_cast<float>(this->data_.ch_r_y) - DR16_CH_VALUE_MID) /
          FULL_RANGE;
    }

    cmd_data.online = true;
    cmd_data.ctrl_source = CMD::CTRL_SOURCE_RC;

    this->cmd_data_tp_.Publish(cmd_data);

    memcpy(&(this->last_data_), &(this->data_), sizeof(Data));
  }

#ifdef LIBXR_DEBUG_BUILD
  /**
   * @brief 用于调试的数据视图结构体（非位域）
   */
  struct DataView {
    uint16_t ch_r_x; /* 右摇杆X轴 */
    uint16_t ch_r_y; /* 右摇杆Y轴 */
    uint16_t ch_l_x; /* 左摇杆X轴 */
    uint16_t ch_l_y; /* 左摇杆Y轴 */
    uint8_t sw_r;    /* 右拨杆 */
    uint8_t sw_l;    /* 左拨杆 */
    int16_t x;       /* 鼠标X轴移动 */
    int16_t y;       /* 鼠标Y轴移动 */
    int16_t z;       /* 鼠标Z轴移动 */
    uint8_t press_l; /* 鼠标左键状态 */
    uint8_t press_r; /* 鼠标右键状态 */
    uint16_t key;    /* 键盘按键状态 */
    uint16_t res;    /* 保留字段 */
  };

  /**
   * @brief 将位域数据转换为普通结构体数据（调试用）
   * @param data_view 输出的数据视图
   * @param data 输入的位域数据
   */
  void DataviewToData(DataView& data_view, Data& data) {
    data_view.ch_r_x = data.ch_r_x;
    data_view.ch_r_y = data.ch_r_y;
    data_view.ch_l_x = data.ch_l_x;
    data_view.ch_l_y = data.ch_l_y;
    data_view.sw_r = data.sw_r;
    data_view.sw_l = data.sw_l;
    data_view.x = data.x;
    data_view.y = data.y;
    data_view.z = data.z;
    data_view.press_l = data.press_l;
    data_view.press_r = data.press_r;
    data_view.key = data.key;
    data_view.res = data.res;
  }
#endif

 private:
  CMD* cmd_; /* CMD模块指针 */
  ControlSource ctrl_source_ =
      ControlSource::DR16_CTRL_SOURCE_SW; /* 当前控制源 */

  Data data_;           /* 当前数据 */
  Data last_data_{};    /* 上一帧数据 */
  CMD::Data cmd_data{}; /* 命令数据 */

#ifdef LIBXR_DEBUG_BUILD
  DataView data_view_; /* 调试用数据视图 */
#endif

  LibXR::UART* uart_;         /* UART接口指针 */
  LibXR::Event event_;        /* 事件处理器 */
  LibXR::Thread thread_uart_; /* UART线程 */
  LibXR::Semaphore sem;       /* 信号量 */
  LibXR::ReadOperation op;    /* 读操作 */
  LibXR::Topic cmd_tp_;       /* 命令主题 */
  LibXR::Topic cmd_data_tp_;  /* 命令数据主题 */
  LibXR::Topic dr16_data_tp_; /* DR16原始数据主题 */
};
