# eLinx 综合工程包（T8：AFDX 端系统 · Tri_Mode_Ethernet_Mac IP 集成）

> 用途：本目录承载 eLinx（中科亿海微）GUI 综合工程的工程文件与操作单。
> **本版为 2026-11-03 按已证明 GUI 流程改写（调度总令 docfix 第 9 项，取代第 1/2/4 项
> 旧冲突口径；第 3 项 afdx_ip_en 误述一并更正）**。历史演进见 GUI_RUN_LOG.md 与
> RPT-T8 系列；CLI 试探结论不变：工程综合内核为 GUI 内置（ehiway_map，Quartus II
> 同族），独立 `eLinx_synthesis.exe`（yosys 内核）无 EQ6 器件库，故走 GUI 流程。

## 一、现行综合输入口径（已证明，第 5~10 轮同口径成功）

- **实际输入 = GUI 工程 30 个仓库 RTL（引用原路径，未拷贝）+ IP Catalog 生成件
  `Tri_Mode_Ethernet_Mac_0`**（OOC：`afdx_es.srcs/sources_1/ip/Tri_Mode_Ethernet_Mac_0/`
  下 .eci/.eeo/.v/.vo/.vqm/.mif，解密发生在 IP Generate 环节）。
- **17 个加密 .v 不入工程**（`D:\elinx\eLinx3.0\bin\ip\RTL\Tri_Mode_Ethernet_Mac_RTL\`
  下的 Tri_Mode_Ethernet_Mac.v/…_Module.v/*_mac.v 等 17 文件为二进制密文——
  eLinx 工具链**并不**原生解密，直接入工程实测 `ehiway_map Error 10170`（第 1 轮
  实录）；旧口径"19 个加密文件、原生解密"作废：实数 17（另 ModuleList.dat 非源文件），
  且必须走 IP Catalog 解密生成。
- **mac_ip_behav.v / *_tb（仿真替身与 tb）绝不入 GUI 工程**——mac_ip_behav 只供
  iverilog 替身回归（同端口同行为口径），综合链接真实 IP。
- `mdio_phy_sim.v` / `mac_tx.v` / `mac_port_if.v` 在工程 30 RTL 内保留（历史沿革）：
  未被例化的模块不进网表，综合无害；`mdio_phy_sim` 仅 `AFDX_MAC_IP_SIM` 替身路径引用。
- **afdx_ip_en.v 不存在（历史误述更正，docfix 第 3 项）**：宏口径现行实现为
  `rtl/core/afdx_es_core.v` 与 `rtl/mac/mac_ip_top.v` 顶部硬编码
  `` `define AFDX_MAC_IP 1 `` / `` `define AFDX_MAC_LOOPBACK 1 ``（交付
  口径），无需额外宏文件。**兜底**（若 eLinx 版本不识别 qsf 的 VERILOG_MACRO
  assignment 且未来宏改为条件编译时）：新建两行 `define` 文件（如
  `rtl/core/afdx_ip_en.v`），**列为第一个源文件**并在 Settings 勾选
  **single compilation unit**。

## 二、工程文件

- `afdx_es.qsf`：工程设置（器件 EQ6HL130-CSG484_H、Top=AFDX_top、Verilog-2001）。
  **注意**：仓库 qsf 为 CLI 时代入档版，其 VERILOG_FILE 清单含 17 条 D:/elinx 加密
  条目（GUI 会话中的移除操作未落盘回 qsf——REV-F-05 事实链见 RPT-FREEZE-003 §二）；
  打开工程后以 GUI 实际源列表为准（30 仓库 RTL + IP Catalog 件），与
  GUI_RUN_LOG 成功态逐条核对后再跑。
- 宏开关（core/mac_ip_top 顶部硬编码）：`AFDX_MAC_IP=1` → core 例化 `mac_ip_top`
  （Tri_Mode_Ethernet_Mac_0 ×2 + TX 适配 + 片内回环）；`AFDX_MAC_LOOPBACK=1` →
  mac_ip_top 片内 1 拍回环电缆，纯仿真必需。

## 三、GUI 综合操作单（负责人执行）

1. 打开 eLinx → 打开既有工程 `sim/afdx_elinx/afdx_es.epr`（不要新建）。
2. **点击 Run Synthesis 前核对两点**（与 GUI_RUN_LOG 成功态逐条一致）：
   - 源列表**无任何 `D:/elinx/...` 条目**（应为 30 条仓库 RTL，引用原路径）；
   - IP 管理器/源树**有 `Tri_Mode_Ethernet_Mac_0`**（OOC 生成件）。
3. `Flow → Run Synthesis`（**只跑综合，不跑 Implementation，不改任何设置**）。产出：
   - `afdx_es.runs/synth_1/afdx_es.vqm`（结构网表，stratix 原语，含 2 例
     Tri_Mode_Ethernet_Mac_0）；
   - `afdx_es.map.rpt` / `afdx_es_utilization_synth.rpt`（综合报告与资源利用率）。
4. 产出归档：vqm / map.rpt / utilization 拷回本目录 `out/`（RPT 引用路径）。
5. ~~`.vo` 门级仿真网表：`Flow → EDA Netlist Writer`~~——**该口径作废**（docfix
   第 4 项）：后仿实际走 `.vqm + 三原语库`，无需 .vo / EDA Netlist Writer 环节。

## 四、综合后仿真（ModelSim SE-64 2020.4）

- 脚本：`scripts/run_psynth_ms.sh`（默认 TB_V=tb_delivery_e2e；探针集
  `TB_V=tb_delivery_short`）——vlib → vlog 三原语库（`D:\elinx\eLinx3.0\Simulation\
  verilog\{220model.v, elinx_mf.v, elinx_atoms.v}`）→ vlog `.vqm` → vsim -c
  `run -all`。工作目录 `out/psynth_work_ms/`（与 Questa 25.1 的 psynth_work 隔离）。
- **判据（check_output c 案，调度裁决 2026-11-03）**：TB 内建逐字节 PASS 行
  （`[DELIV-TB] PASS` / `[SHORT-TB] PASS`，payload byte-exact）+ 里程碑日志
  （cfg-done / injected / delivered 带 $time）+ transcript 存档；check_output.py
  本体不适用于门级交付流（两 delivery TB 均不落帧）。
- **阶段 1.5 预验证结论（RPT-FREEZE-002）**：装载 107s、Errors=0、零节流、
  cfg-done 801.272µs 已越；门级速率实测 ≈30µs 仿真时间/分钟（较 Starter 探针快
  ≈33×），全量 e2e 预期 3~5h（分离进程长跑）。
- 联调口径：IP 仅承担 TX 方向（tx_axis→GMII，含前导/SFD/FCS/IFG）；RX 方向由
  T7 RTL mac_rx 接收 IP 的 GMII 输出（片内回环）。IP 的 rx_axis 在
  rx_mac_rst_n=0 下抑制（纯仿真无外接 PHY，rx_mac_aclk 域不可控）。

## 五、已知集成约束（RPT-T8 边界说明素材）

1. `Tri_Mode_Ethernet_Mac` 的 speed/link 由 MDIO 读外接 PHY 决定；纯仿真挂
   `mdio_phy_sim` 虚拟 PHY（Clause 22 读应答 BMCR=0x0140/BMSR=0x0024，使
   speed=2'b10/link=1）。真实硬件时移除虚拟 PHY、mdio/mdc 引出管脚。
2. IP 的 rx_axis 数据/时钟域（rx_mac_aclk）源于 PHY 读到的速率，纯仿真下不可控且
   无验收口径；本设计 RX 链保持 T7 RTL mac_rx（已全量验收），IP 承担 TX 与 GMII
   物理口。此为集成口径，非 IP 功能裁剪。
3. IP 例化名 `Tri_Mode_Ethernet_Mac_0`（37 口，与 r3.0 实际接口一致；13 个 PG171
   多入口按厂商配置板承载，零损失——GUI_RUN_LOG §二.4）。

## 六、复现基线（docfix 第 9 项归档——第 11 轮，2026-11-03 04:27:32 综合 0 errors）

- **工程输入 = 以下 30 条仓库 RTL（qsf VERILOG_FILE 仓库路径条目，引用不拷贝）+
  IP Catalog `Tri_Mode_Ethernet_Mac_0` 生成件**（`afdx_es.srcs/sources_1/ip/
  Tri_Mode_Ethernet_Mac_0/`：.eci/.eeo/.v/.vo/.vqm/.mif）。17 条 D:/elinx 加密条目
  不入工程（GUI 会话成功态，第 11 轮 0 errors 佐证）。
- 30 条清单：rtl/AFDX_top.v；rtl/core/{afdx_es_core,cfg_bus,ram_init,timebase,
  host_arb,stats}.v；rtl/tx/{tx_vl_table,tx_regulator,tx_scheduler,tx_sn_gen,
  tx_frame_builder}.v；rtl/mac/{mac_ip_top,mac_rx,mac_tx,mac_port_if,mdio_phy_sim}.v；
  rtl/rx/{rx_path,rx_parser,rx_vl_lookup,rx_integrity,rx_redundancy,rx_reassembly}.v；
  rtl/lib/{buf_mgr,sdp_ram,sync_fifo,rr_arbiter,tick_gen,crc32_d8,csum16}.v。
  （其中 mac_tx/mac_port_if/mdio_phy_sim 为未例化保留条目，不进网表。）
- 第 11 轮产物：afdx_es.vqm 68,315,907B（LC 51,444，较第 10 轮 +4 = frag_end
  17 位比较器），map.rpt 0 errors / 59 warnings。

## 七、RTL 回归激励生成说明（docfix 第 10 项，2026-11-03）

- `scripts/regress.sh` 的 txframe 段消费 `sim/tb_out/tx1/`（含 vl_cfg.txt）——
  历史上该目录仅能由手工 `python3 -I scripts/gen_stim.py tx1 sim/tb_out/tx1`
  生成，新克隆必挂（`captured A=0 B=0` 空转自败）。
- **已修复**：regress.sh 在 txframe 段前新增 `tx1_gen` 步（gen_stim.py tx1），
  计入 PASS/FAIL 判据；fresh clone 预演实证留检查点 C。
- 同族缺口备忘：e2e 段默认 smoke 案例无 vls 配置（gen_stim.py:24）不生成
  vl_cfg.txt，tb_afdx_tx 必读该文件——e2e_sim 在默认口径下结构性 FAIL
  （2026-11-03 全量回归实录，21 过/2 挂），修法 a/b/c 已报调度待裁。
