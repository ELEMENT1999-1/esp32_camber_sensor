# X 轴倾角传感器（外倾角测量）— ESP32-C6-Touch-AMOLED-2.16

基于微雪 ESP32-C6-Touch-AMOLED-2.16 开发板（QMI8658 IMU + 480×480 AMOLED 触摸屏）的
**X 轴倾角传感器**，用于赛车外倾角（camber）测量与动态监测。

## 功能

- **X 轴倾角实时显示**：板面倾角 θ = asin(az/|g|)，**竖直摆放 = 0°**
  （三轴合成，与板面内旋转无关），100px 大字
- **±5° 刻度尺**：显示**绕 Z 的左右倾斜**（面内侧倾，上沿右倾为正），粗绿指针
- **两点精度校准**：电脑端软件采集两个已知参考姿态，写入零点/比例系数到 NVS，
  修正传感器刻度与零位误差；另支持远程重标陀螺零偏
- **归零/实时切换**：「归零」把当前角度设 0°，再按回到实时读数，断电记忆
- **中英文界面**：触摸「中/EN」切换，选择存入 NVS
- **电池电量与充电状态**：右上角电量条 + 百分比 + 「充电中」标识
- **蓝牙 BLE**：连接电脑上位机（`pc_client/`），10Hz 遥测 + 命令控制
  （归零 / 重标陀螺 / 反向 / 改标题，标题同步为蓝牙设备名）
- **抗振动解算**：矢量窗口平均 + 互补滤波 + 陀螺零偏自标定
- **串口 CSV 输出**：50 Hz 数据流

电脑端上位机见 `../pc_client/`（车辆俯视图布局，FL/FR/RL/RR 轮位绑定、
System 管理、校准、标题同步）。

## 硬件

| 项目 | 说明 |
| --- | --- |
| 开发板 | Waveshare ESP32-C6-Touch-AMOLED-2.16 |
| IMU | QMI8658（I2C 地址 0x6B），加速度 ±2g / 陀螺 ±256dps / 500Hz |
| 显示 | 1.6"（2.16"）AMOLED 480×480 QSPI（SH8601），触摸 CST85217/CST9217 |
| 电源 | AXP2101 PMIC（板载，ALDO3 为显示复位控制） |

传感器安装要求（与角度定义一致）：

- 板上 U5（QMI8658）丝印坐标系：**+X = 屏幕右，+Y = 屏幕上，+Z = 出屏（朝观察者）**，右手系
- 传感器 **X 轴（屏幕左右方向）** 沿轮轴/车长方向，**Z 轴（板面法线）** 垂直于轮平面
- 用磁吸座或支架把板子贴在刹车盘面/轮毂端面，板面平行于轮平面
- 此时 θ = asin(az/|g|) 即轮平面相对铅垂面的夹角（外倾角）；符号反了按「反向」
- 竖直立放 = 0°（板面内旋转不影响读数）；向后仰（上沿后倒）读数为负

## 目录结构

```
camber-sensor/
├── CMakeLists.txt
├── sdkconfig.defaults          # esp32c6 / 16MB flash / LVGL 字体
├── partitions.csv
├── main/
│   ├── main.cpp                # 初始化 + 采样任务 + 校准/NVS
│   ├── ui_camber.cpp/.h        # LVGL 双语界面（角度、刻度尺、按钮）
│   ├── lv_font_cjk_16.c        # 中文 16px 字体（lv_font_conv 生成）
│   ├── lv_font_cjk_20.c        # 中文 20px 字体（lv_font_conv 生成）
│   ├── user_config.h           # 引脚定义（与官方例程一致）
│   └── idf_component.yml       # LVGL / SH8601 / 触摸 / esp_lvgl_adapter
└── components/
    ├── qmi8658/                # QMI8658 驱动（源自官方 Arduino 例程）
    ├── tilt/                   # X 轴倾角解算（核心算法）
    ├── port_bsp/               # I2C + 显示/触摸 BSP（官方例程）
    ├── pmicpower/              # AXP2101 电源管理（官方例程）
    └── app_bsp/                # LVGL 移植层（官方例程）
```

字体再生成（如需增补汉字）：

```powershell
# 汉字清单在 --symbols 里按需追加
# 注意：必须加 --no-compress --no-prefilter！
# lv_font_conv 默认做 RLE 压缩，而工程未启用 LVGL 的
# LV_USE_FONT_COMPRESSED 解压器：压缩数据当原始位图读 → 全屏乱码
node npm-cli.js exec --yes lv_font_conv -- --size 20 `
  --font C:\Windows\Fonts\simhei.ttf -r 0x20-0x7E `
  --symbols "轴倾角振动滤波峰值归零复位反向偏置正常中°" `
  --format lvgl --bpp 4 --no-compress --no-prefilter -o main/lv_font_cjk_20.c
```

生成后可运行 `python tools/dump_font_glyphs.py main/lv_font_cjk_20.c`
用 ASCII 预览字形，确认输出的是可读字而不是噪声。

## 编译烧录

已在本机验证通过：**ESP-IDF v5.5.6**（`C:\esp\v5.5.6\esp-idf`，工具链 `C:\Espressif\tools`，
Python 环境 `C:\Espressif\python_env\idf5.5_py3.11_env`）。

```powershell
cd camber-sensor
idf.py set-target esp32c6
idf.py build
idf.py -p COMx flash monitor
```

环境注意事项（本机踩过的坑）：
- 若 `export.bat` 报 "cannot find the batch label"：文件是 LF 行尾，需转为 CRLF
- 若报 Python virtual environment not found：先跑一次 `install.bat esp32c6`
  （工具已装在 `C:\Espressif` 时需设置 `IDF_TOOLS_PATH=C:\Espressif`）
- PATH 里的 Microsoft Store python 占位程序会导致 export 检测失败，
  需让 `C:\Espressif\tools\python` 排在前面

## 使用

1. **上电后前 1 秒保持板子静止**（陀螺零偏标定），随后正常使用
2. 主界面：大数字为当前 X 轴倾角，下方刻度尺为 ±5° 指示
3. **归零**：把板子放到目标零位（如水平平台）按「归零」
4. **峰值复位**：清掉 MAX/MIN 记录
5. **反向**：读数符号与期望相反时按此键（另一侧车轮镜像安装）
6. **中/EN**：切换界面语言，选择断电保存
7. 串口 115200-8N1，CSV 表头：

```
time_ms,angle_deg,accel_angle_deg,offset_deg,ax_g,ay_g,az_g,gyro_x_dps,gyro_bias_dps,battery_pct,charging
```

## 算法要点（components/tilt/tilt.c）

1. **角度定义**：θ = atan2(az, √(ax²+ay²)) = asin(az/|g|) —— 板面相对竖直面的
   倾角（Z 轴水平 = 0°），三轴合成，与板面内旋转无关。
   陀螺按运动学 du_z/dt = ωy·ux − ωx·uy 双轴配对，任意面内朝向一致。
   安装到轮面（板面平行于轮平面）时 θ 即外倾角。
2. **窗口矢量平均**：对 (ax, ay, az) 做 128 拍（256 ms @500Hz）滑动平均后，
   再解 atan2。周期振动在矢量域相互抵消——逐拍解 atan2
   会因非线性产生系统性偏差，窗口平均消除该偏差。
2. **陀螺零偏标定**：上电前 500 拍（1s）平均零偏并持续慢速跟踪温漂。
   实测该 QMI8658 X 轴零偏约 +5 °/s，不校正会造成约 1° 级稳态偏差。
3. **互补滤波**：`angle = wrap(angle + (ω_x-bias)·dt + (1-α)·diff)`，α = 0.99，
   陀螺负责动态桥接，加速度计提供长期基准；±180° 边界做环绕处理。
4. **自适应置信度**：
   - 平均矢量模长偏离 1g 超过 0.03g → 降低加速度权重（振动/线加速度）
   - 动态时平均角度与陀螺预测偏差 > 15° → 丢弃本拍加速度（冲击工况）
   - **静止时（|ω|<1.5dps 且重力正常）直接信任加速度计**，防止大偏差后死锁
   - 极端工况（如 +2g 纵向加速）下仅靠陀螺保持，0.4 s 内误差 < 0.1°

数值验证（`tools/verify_tilt.py`，与 C 实现同构，8 项全过）：

| 用例 | 结果 |
| --- | --- |
| 静态精度 ±3.5° 内 | 误差 0.000° |
| 陀螺零偏 +3.3dps 标定 | 误差 0.000°，零偏准确估出 |
| 大偏差静止收敛（死锁回归） | 误差 0.000° |
| ±180° 边界 | 误差 0.000° |
| 0→2° 阶跃收敛 | 1.06 s |
| 25Hz/0.3g 振动下 1.5° | 误差 0.012° |
| 多频振动（12/37/58Hz）下 -2° | 误差 0.019° |
| +2g 干扰 0.4s 后 | 误差 0.052° |

实机联调（2026-11-17）：X 陀螺零偏估计 5.17 dps，静止时角度与
atan2(ay,az) 平均偏差 0.12°。

## 标定建议

- **两点精度校准（推荐，上车前做一次）**：电脑端「校准」面板——
  1. 把传感器放到第一个已知姿态（如水平平台/0°），填参考值，点「采集A」
  2. 换第二个已知姿态（如 3° 角度块），填参考值，点「采集B」
  3. 点「写入校准」（数据存 NVS），随后按一次「归零」
- 静态零位：把板子放在目标基准上按「归零」
- 陀螺温漂：觉得读数发飘时点「重标陀螺」（静止 2 秒）
- 安装重复性：磁吸座每次装夹后重新归零，避免装夹误差进读数
