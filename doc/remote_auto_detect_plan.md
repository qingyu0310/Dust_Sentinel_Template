# Remote 自动识别遥控器改造规划

## 背景

当前 remote 模块已经支持 DR16、VT12、VT13 三种遥控器协议，但运行时协议类型是固定配置的：

- `modules/remotes/remote.hpp` 中 `RemoteType` 已包含 `DR16`、`VT12`、`VT13`、`None`。
- `modules/remotes/remote.cpp` 中 `kProtocols[]` 保存协议解析函数和固定帧长。
- `thread::remote::thread_init()` 里固定调用 `remote_.Init(RemoteType::DR16, rx)`。
- `Remote::Task()` 收到 UART 数据后，只按当前 `proto_` 的 `frame_size` 和 `func` 尝试解帧。

也就是说，当前设计是“启动时选定一种协议”，不是“从输入字节流里自动判断协议”。

## 改造目标

把 remote 模块改成自动识别遥控器：

1. 上电后无需手动指定 DR16/VT12/VT13。
2. 同一 UART 输入流中，remote 线程能根据帧特征自动锁定协议。
3. 遥控器断连、换遥控器、乱码输入后，系统能回到探测状态并重新识别。
4. 对外仍然发布同一个 `topic::remote_to::Message`，底盘、云台等消费者尽量不用改。
5. 后续新增协议时，只需要新增协议描述，不需要重写 remote 主循环。

## 当前代码约束

### 1. decoder 现在会直接发布 zbus

`dr16::dataprocess()`、`vt12::dataprocess()`、`vt13::dataprocess()` 当前职责较重：

- 校验帧是否合法。
- 解析原始数据。
- 映射为语义化 `Message`。
- 调用 `zbus_chan_pub()` 发布。

自动识别时不能直接拿这些函数对所有协议反复试探，否则一个误判成功的协议会直接发布错误数据。

建议先拆分职责：

```cpp
bool validate(const uint8_t* buffer, uint16_t len);
bool decode(const uint8_t* buffer, uint16_t len, topic::remote_to::Message& pub);
```

remote 主循环负责：

- 收包。
- 滑窗同步。
- 协议探测。
- 选择是否发布。

各协议模块负责：

- 判断当前窗口是否可能是本协议。
- 把合法帧解码到 `Message`。

### 2. 各协议帧特征强度不同

现有协议可用特征：

| 协议 | 帧长 | 强特征 | 弱特征 |
| --- | --- | --- | --- |
| DR16 | 18 | 通道值范围 364..1684，开关值 1/2/3 | 无固定帧头，误判风险较高 |
| VT12 | 16 | 疑似帧头 `0xA5`，长度/seq/crc8/cmd 字段 | 当前代码里校验逻辑需要确认是否完整启用 |
| VT13 | 21 | 帧头 `0xA9 0x53`，尾部 `crc16` 字段 | 当前未看到 crc16 校验实现 |

自动识别优先级建议：

1. 先匹配有固定帧头和校验的协议：VT13、VT12。
2. 再匹配无帧头但范围特征明显的 DR16。
3. DR16 至少需要连续多帧通过校验后才锁定，避免随机字节误判。

## 目标架构

### RemoteType 增加 Auto

```cpp
enum class RemoteType : uint8_t
{
    DR16 = 0,
    VT12,
    VT13,
    Auto,
    None,
};
```

`thread_init()` 改为：

```cpp
remote_.Init(RemoteType::Auto, rx);
```

也可以保留 Kconfig 或编译期宏，允许强制指定协议，方便调试。

### 协议描述表扩展

建议把 `Protocol` 从“解析函数 + 帧长”扩展成完整描述：

```cpp
struct Protocol {
    RemoteType type;
    const char* name;
    uint16_t frame_size;
    bool (*validate)(const uint8_t* buffer, uint16_t len);
    bool (*decode)(const uint8_t* buffer, uint16_t len, Message& pub);
    uint8_t lock_score;
};
```

其中：

- `validate()`：只判断帧是否合法，不发布。
- `decode()`：只把合法帧写入 `Message`，不自己决定协议选择。
- `lock_score`：连续命中几次后锁定协议。建议 VT13/VT12 为 1 或 2，DR16 为 3 或 5。

### Remote 内部状态机

建议增加状态：

```cpp
enum class DetectState : uint8_t
{
    Detecting,
    Locked,
};
```

核心成员：

```cpp
RemoteType configured_type_ = RemoteType::Auto;
RemoteType active_type_ = RemoteType::None;
DetectState detect_state_ = DetectState::Detecting;
uint8_t hit_count_[protocol_count] {};
uint32_t last_valid_ms_ = 0;
```

状态含义：

- `Detecting`：遍历所有协议，用滑动窗口查找可用帧。
- `Locked`：只按 `active_type_` 解析，效率更高。
- 超时或连续解析失败：清空缓存，回到 `Detecting`。

## 字节流同步策略

### Detecting 状态

输入 UART 数据追加到 `frame_buf_` 后：

1. 从 `frame_buf_[0]` 开始尝试所有协议。
2. 对每个协议检查 `frame_pos_ >= frame_size`。
3. 调用 `validate()`。
4. 命中后增加该协议 hit count。
5. 达到 `lock_score` 后锁定协议，调用 `decode()` 发布。
6. 若所有协议都不匹配，丢弃 1 字节并继续滑窗。

伪代码：

```cpp
while (frame_pos_ >= min_frame_size) {
    bool matched = false;

    for (auto& proto : kProtocols) {
        if (frame_pos_ < proto.frame_size) continue;
        if (!proto.validate(frame_buf_, proto.frame_size)) continue;

        matched = true;
        hit_count[proto_index]++;

        if (hit_count[proto_index] >= proto.lock_score) {
            active_type_ = proto.type;
            proto_ = proto;
            detect_state_ = DetectState::Locked;
            proto.decode(frame_buf_, proto.frame_size, pub_);
            consume(proto.frame_size);
        }
        break;
    }

    if (!matched) {
        drop_one_byte();
    }
}
```

### Locked 状态

锁定后只用当前协议解析：

1. `frame_pos_ >= proto_.frame_size` 时调用 `validate()`。
2. 合法则 `decode()` 并消费整帧。
3. 不合法则失败计数加一，丢弃 1 字节重新同步。
4. 连续失败超过阈值或超时无合法帧，切回 `Detecting`。

建议阈值：

- `kUnlockFailLimit = 5`
- `kRemoteTimeoutMs = 100`

## 协议模块改造建议

### DR16

新增：

```cpp
bool validate(const uint8_t* buffer, uint16_t len);
bool decode(const uint8_t* buffer, uint16_t len, Message& pub);
```

校验建议：

- `len == 18`。
- 四个通道值在 `364..1684`。
- `sw1`、`sw2` 必须是 1、2、3。
- 鼠标和键盘字段不作为强校验，只作为解析数据。

注意：DR16 无帧头，自动识别时必须多帧确认。

### VT12

新增：

```cpp
bool validate(const uint8_t* buffer, uint16_t len);
bool decode(const uint8_t* buffer, uint16_t len, Message& pub);
```

校验建议：

- `len == 16`。
- `buffer[0] == 0xA5`。
- 确认 `data_length`、`cmd_id` 的期望值。
- 若协议文档明确 crc8，补上 crc8 校验。

当前 `vt12.cpp` 内有帧头、长度、crc8、cmd_id 注释，建议先确认真实协议格式，再决定是否启用完整校验。

### VT13

新增：

```cpp
bool validate(const uint8_t* buffer, uint16_t len);
bool decode(const uint8_t* buffer, uint16_t len, Message& pub);
```

校验建议：

- `len == 21`。
- `buffer[0] == 0xA9 && buffer[1] == 0x53`。
- 通道值、wheel 值范围合理。
- 若 crc16 可确认，补上 crc16 校验。

VT13 有双字节帧头，适合作为自动识别的高优先级协议。

## Message 扩展建议

为了方便调试和上层判断，建议给 `topic::remote_to::Message` 增加来源字段：

```cpp
uint8_t type = static_cast<uint8_t>(RemoteType::None);
```

或者定义 topic 层自己的枚举，避免 topic 反向依赖 `remote.hpp`。

每次发布时填入当前协议：

```cpp
pub.type = static_cast<uint8_t>(active_type_);
pub.version++;
```

如果不希望改 zbus 消息结构，至少在 remote 内部通过日志输出当前锁定协议。

## 实施步骤

### 阶段 1：协议接口拆分

- 为 DR16、VT12、VT13 分别新增 `validate()` 和 `decode()`。
- 让原 `dataprocess()` 临时保留为兼容包装：

```cpp
bool dataprocess(uint8_t* buffer, uint8_t len, Message& pub)
{
    if (!validate(buffer, len)) return false;
    return decode(buffer, len, pub);
}
```

- 将 `zbus_chan_pub()` 从协议模块迁移到 `Remote::Task()`，避免探测时误发布。

### 阶段 2：引入 Auto 状态机

- `RemoteType` 增加 `Auto`。
- `Remote::Init()` 保存 `configured_type_`。
- 如果配置为固定协议，保持当前行为。
- 如果配置为 `Auto`，进入 `Detecting` 状态。
- 实现 `Detecting` / `Locked` 两种处理路径。

### 阶段 3：增强校验

- VT13 增加 crc16 校验，若协议文档确认。
- VT12 增加 crc8、长度、命令字校验，若协议文档确认。
- DR16 增加连续多帧命中锁定机制。

### 阶段 4：调试能力

- 增加协议锁定、解锁、超时日志。
- 可选 shell 命令：
  - 查看当前协议。
  - 强制切换协议。
  - 重新进入 Auto 探测。
- 可选统计字段：
  - 每个协议命中次数。
  - 连续失败次数。
  - 最近一次有效帧时间。

### 阶段 5：测试与验证

- 单元级测试：
  - 每种协议的合法帧能通过 `validate()`。
  - 错位帧、随机数据、错误帧头不能通过。
  - DR16 随机数据不应轻易锁定。
- 集成测试：
  - 上电接 DR16，自动锁定 DR16。
  - 上电接 VT12，自动锁定 VT12。
  - 上电接 VT13，自动锁定 VT13。
  - 运行中断开遥控器，超时后发布归零数据。
  - 运行中更换遥控器，回到 Detecting 后重新锁定。

## 验收标准

1. `thread_init()` 使用 `RemoteType::Auto` 后，三种已支持遥控器都能自动识别。
2. 锁定协议后，`Message.version` 正常递增，控制字段与原固定协议行为一致。
3. 输入随机字节流时不会持续发布错误控制数据。
4. 遥控器断连超过超时时间后，控制输出归零。
5. 换遥控器后能在有限时间内重新识别。
6. 固定协议模式仍可使用，便于问题定位和比赛现场兜底。

## 风险点

- DR16 没有帧头，误识别风险最高，需要连续多帧确认。
- 当前 VT12/VT13 代码里有 crc 字段或注释，但校验实现不完整，自动识别可靠性依赖补齐校验。
- 协议模块当前直接发布 zbus，必须先拆分发布职责，否则自动探测会引入错误发布。
- `frame_buf_` 当前大小为 64，足够容纳现有协议，但后续新增更长协议时需要同步调整。
- 如果 UART DMA 回调给出的数据切片很碎，状态机必须支持跨多次读取拼帧。

## 推荐最终形态

remote 主循环只做“字节流管理 + 状态机 + 发布”，协议模块只做“校验 + 解码”。这样自动识别、新增协议、强制协议调试三件事都能走同一套结构，后续维护成本最低。
