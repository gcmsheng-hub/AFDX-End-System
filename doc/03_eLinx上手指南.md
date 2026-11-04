# eLinx 上手指南（保姆级 · 面向零基础）

> 依据：官方手册 `D:\elinx\eLinx3.0\UG102_eLinx_Design_Suite_User_Guide_ehiway_v2024.9.pdf`（298 页）
> 本文中"手册 §x.x / 图 y"指该 PDF 的章节和插图编号，卡住时去翻对应节看截图。
> 已备好练习文件：`sim/hello_afdx/led_blink.v`（设计）+ `tb_led_blink.v`（仿真）

## 0. 启动软件

双击 `D:\elinx\eLinx3.0\eLinx3.0.exe`（建议右键→发送到→桌面快捷方式）。
启动后看到 **Quick Start** 欢迎页（手册 图1）。

## 1. 新建工程（手册 §2.1，6 步向导）

入口：菜单 `File → Project → New…`，或欢迎页 **Create Project** 按钮。

| 向导页 | 做什么 |
|---|---|
| ① 欢迎页 | 直接 **Next** |
| ② Project | Project name 填 `hello_afdx`；Project location 选 `<仓库根目录>\sim`；勾选 *Create project subdirectory*。**路径不要有中文和空格** |
| ③ Project Type | 选 **RTL Project**；新手不要勾 *Do not specify sources at this time*（我们要在下一步加文件） |
| ④ Add Sources | 点 **Add Files**，选 `sim/hello_afdx/led_blink.v`，勾选 *Copy sources into project*；这里也可以用 **Create File** 直接新建 .v |
| ⑤ Add Constraints | 练习工程不需要，直接 **Next**（约束文件是 *.edc 格式） |
| ⑥ Device / 器件选择 | 选竞赛指定芯片：**EQ6HL130P**，封装 **CSG484**（eHiChip6 系列） |
| 汇总页 | 核对无误 → **Finish** |

## 2. 认识主窗口（手册 §3，图22）

打开工程后：
- 左侧 **Flow Navigator**：整个流程入口，分 5 组 —— Project Manager / Simulation / Synthesis / Implementation / Program & Debug
- 左中 **Sources** 窗口：三个分组 —— Design Sources（设计源文件）、Constraints（约束）、Simulation Sources（仿真文件）
- 下方 **Tcl Console / Messages / Design Runs**：命令行、报错信息、运行状态都在这
- **报错先看 Messages 窗口，警告/错误双击可跳到源码行**

## 3. 跑综合（练习目标 1）

1. 在 Sources 窗口确认 `led_blink.v` 已在 Design Sources 下。
2. 设顶层：若 `led_blink` 没有自动成为 top，右键该文件 → **Set as Top Module**（手册 图64：综合前 eLinx 会强制检查）。
3. 入口（任选其一，手册 §3.1.3）：
   - 菜单 `Flow → Run Synthesis`
   - 工具栏 Run Synthesis 按钮
   - 双击 Flow Navigator 中的 **Run Synthesis**
4. 等待运行：Design Runs 窗口出现 running 图标（手册 图63）。
5. **验收**：状态变绿/完成无 error，Design Runs 行里出现资源使用信息（LUT/FF 数量，手册 图66）。

## 4. 跑实现（布局布线）与位流（练习目标 2、3）

1. `Flow → Run Implementation`（或 Flow Navigator 双击）——先跑完综合才能跑。
2. 练习工程没写引脚约束，可能对 I/O 有 warning，不影响跑通流程。
3. 位流：Flow 菜单下 **Generate Bitstream**（竞赛全程仿真，位流只是证明流程闭环，不下载硬件）。

**这三步跑通 = eLinx 编译流程入门完成。**

## 5. 行为仿真（练习目标 4，需要 ModelSim）

> 重要：eLinx 的行为仿真调用第三方 **ModelSim**（手册 §3.1.3 明确：前提是已安装 ModelSim，并在 Settings 里配置其目录）。Intel 现已用 **Questa - Intel FPGA Starter Edition**（免费）取代 ModelSim，同引擎同命令，装 Questa 即可（对接方法见下）。

1. 添加仿真文件（手册 §11.2）：Sources 窗口选中 **Simulation Sources** 节点 → 右键 **Add Sources** → *Add or Create Simulation Sources* → **Add Files** 选 `tb_led_blink.v` → Finish。
2. **配置仿真器（手册 §4.9 / §11.1，一次性）**：打开工程后菜单 **`Tools → Settings…`**，左侧页选 **3rd Party Simulators**——手册原文"**选择 modelSim.exe 所在目录**"（是**目录**，不是 exe 文件本身）：
   - 装 Questa：先到 Questa 安装目录（如 `C:\intelFPGA_LITE\25.1\questa_fse_win\win64\`）把 `vsim.exe` 复制一份改名 `modelsim.exe`（同目录 DLL 依赖自动满足），然后把该 win64 目录填进 eLinx；
   - 再切到左侧 **Simulation** 页（§4.2）：仿真库路径选 `D:\elinx\eLinx3.0\Simulation\elinx_lib`，**Simulation Resolution 选 1ns**（我们的 tb 用 `timescale 1ns/1ps`）。
3. 确认 tb 为仿真顶层：Simulation Sources 里右键 `tb_led_blink` → Set as Top（target）。
4. 运行（手册 §11.3 / 图105）：Flow Navigator 的 **SIMULATION** 组双击 **Run Behavioral Simulation**（或 Flow 菜单同名项），eLinx 自动拉起仿真器。
5. **验收**：波形里 led 每 20μs 翻转一次；控制台打印 `[TB] PASS`。
6. 若报库错误 → 手册 §11.1 原话："不同的 ModelSim 版本之间库文件不能通用……需要用户自己编译库文件"。处理：按 §11.4 新建文件夹 → 仿真器里 Change Directory → New Library → Compile `D:\elinx\eLinx3.0\Simulation\verilog\*.v` 全部 → 回 Settings 的 Simulation 页把库路径指到新文件夹。（注：elinx_lib 主要是器件原语库，我们纯 RTL 行为仿真通常用不到，报错了才需要处理。）

**没有装 ModelSim 时**：第 5 节先跳过，前三节跑通即达成阶段 0 主目标；ModelSim 装好后回来补（免费途径：ModelSim Intel FPGA Starter Edition，或 QQ 群 1107062816 问往届同学用什么版本）。

## 6. 把 AFDX_top.v 加入工程（与竞赛正式对接）

1. `File → Add Sources → Add or Create Design Sources`，添加 `rtl/AFDX_top.v`（勾 Copy into project）。
2. Sources 窗口确认出现 `AFDX_top`，此时 **Set as Top Module**。
3. 再跑一次 `Run Synthesis`——**综合通过（空壳）即完成与竞赛顶层模板的第一次握手**。

之后正式开发时，新建独立工程 `AFDX_ES_prj`（同样流程，源文件指到 rtl/ 全目录），练习工程可删可留。

## 7. 常见问题

| 现象 | 处理 |
|---|---|
| 提示设置 top module | Sources 里右键顶层文件 → Set as Top Module |
| 综合报中文路径/找不到文件 | 工程与源文件路径去掉中文和空格 |
| Run Behavioral Simulation 报找不到 modelsim | Settings→3rd Party Simulators 没配，或 ModelSim 未安装 |
| 仿真库不匹配错误 | 按手册 §11.4 重编库（见本文 §5.6） |
| 器件列表里找不到 EQ6HL130P | 器件页 series 选 eHiChip6 系列再找；实在没有就进 QQ 群问教学版器件范围 |
| **Generate Bitstream 失败，报 IO/pin 相关 error**（实现已通过）| 端口没分配管脚。`Tools → I/O Planning`（手册 §5.1）→ I/O Ports 窗口里双击每个端口的 **PackagePin** 列，各选一个合法管脚 → `Ctrl+S` 保存到 .edc（§5.6；若提示没有目标 edc，先 File→Add Sources→Add or Create Constraints→Create File 建一个）→ 重跑 Run Implementation → Generate Bitstream |
| 其他 | 手册对应章节 + QQ 群 1107062816 + 微信视频号「中科亿海微」 |

## 8. 阶段 0 验收清单

- [ ] eLinx 启动正常，hello_afdx 工程建立
- [ ] led_blink 综合通过（无 error）
- [ ] Implementation 通过
- [ ] Bitstream 生成成功
- [ ] （装好 ModelSim 后）tb_led_blink 仿真出 `[TB] PASS`
- [ ] AFDX_top.v 加入工程并综合通过

全部勾完 → 阶段 0 收官，告诉我仿真器用的是哪个（ModelSim 版本），我据此定 tb 语言方案（Verilog-2001 保守兼容 or SystemVerilog）并开始写验证平台。

## 9. eLinx 物理约束文件（.edc）语法实录

由 I/O Planning 图形化分配管脚后自动生成（`hello_afdx.edc`，Quartus 风格赋值语法）：

```tcl
# 管脚位置分配
set_location_assignment PIN_J1 -to clk
# 电平标准
set_instance_assignment -name IO_STANDARD "3.3V LVTTL" -to clk
# 其他实例属性：CURRENT_STRENGTH_NEW 12MA / SLEW_TYPE SLOW /
# OPEN_DRAIN_OUTPUT OFF / ENABLE_BUS_HOLD_CIRCUITRY OFF，均 -to <端口名>
```

时序约束（SDC 标准语法）另用 `create_clock` 等，写在同一 .edc 或独立文件，
正式工程阶段照此格式编写。图形化分配后保存若顶部出现
"Constraint file changed. Reload" 横幅，点 **Reload** 再重跑 Implementation。
