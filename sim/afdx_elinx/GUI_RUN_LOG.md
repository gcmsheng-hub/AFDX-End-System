# eLinx GUI 综合运行日志（T8 · afdx_es）

> 用途：记录 GUI 综合全过程的所有改动/发现/回退项，防上下文丢失；与 RUNBOOK.md 并排，
> RPT-T8 报告引用。状态截至 2026-11-01 晚（第 4 轮综合调试中）。

## 一、工程基本信息

- 工程：`D:\Agent_Project\Zcode\AFDX_ES\sim\afdx_elinx\afdx_es.epr`
- 器件 EQ6HL130 - CSG484_H（eHiChip6），Top=AFDX_top，Verilog，VERILOG_2001
- 源文件：30 个仓库 RTL（引用原路径，未拷贝）+ IP Catalog 生成的 `Tri_Mode_Ethernet_Mac_0`
- **17 个加密 .v 已从工程移除**（原始文件在 `D:\elinx\eLinx3.0\bin\ip\RTL\Tri_Mode_Ethernet_Mac_RTL\`，未动）
- 注意：GLM 操作单写"19 个加密文件"，实际 17 个（另有 ModuleList.dat 非源文件）

## 二、已完成的关键操作（按序）

1. **宏开关**：eLinx 3.0 无 VERILOG_MACRO 设置 UI、无 single compilation unit 选项
   → 硬编码进 RTL（见"三、改动清单"）。
2. **加密 IP 直读失败**：ehiway_map 把 17 个密文当明文读，Error 10170（第 1 轮）。
   RUNBOOK §二.2"GUI 原生解密"口径作废；CLI yosys 同样不可读（GLM afdx_synth.ys
   注释已自证："解析得空设计"）。
3. **改走 IP Catalog**：
   - `Tri_Mode_Ethernet_Mac` 未出现在 IP Catalog → 根因：安装文件
     `D:\elinx\eLinx3.0\bin\ip\GUI\ip_repository.xml` 第 844 行
     `<FlowType>999</FlowType>`（厂商藏 IP；有效流程号须含 0）。
     **已改**为 `0;1;2;6`（对照 TripleSpeedEthernet 条目），备份在同目录
     `ip_repository.xml.bak`。
   - Customize IP：Component Name 用 **`Tri_Mode_Ethernet_Mac_0`**（不能与内部
     模块同名，弹窗确认）；GMII；**MDIO "PHY Device RD/WR Port" 必须勾**（虚拟
     PHY 依赖）；features 三项（HALF Duplex/Frame Filter/RX Flow Control）不勾。
   - Generate（Out of context per IP）产物：
     `afdx_es.srcs/sources_1/ip/Tri_Mode_Ethernet_Mac_0/`{.eci,.eeo,.v,.vo,.vqm}
     ——解密发生在生成环节。
4. **IP 实际接口与 PG171 指南端口表不符**：v3.0 实际 37 口，无以下 13 口：
   `col, crs, mii_tx_clk, user_duplex_mode, user_port_mode, mac_filter_sw/en/num/addr,
   pause_req, pause_info, sour_mac_addr, rx_axis_filter_tuser`
   → `mac_ip_top.v` 例化已裁剪至 37 口（37/37 与 .v 核对一致）；被删口原本全是
   0 值拉死，语义由 Customize 配置承载（GMII/全双工/无过滤/无流控），零损失。

## 三、改动清单与回退策略

### A. 临时改动（归档后 git 回退，恢复 iverilog 回归口径）
| 文件 | 改动 | 回退 |
|---|---|---|
| `rtl/core/afdx_es_core.v` | 顶部加 `` `define AFDX_MAC_IP 1`` / `` `define AFDX_MAC_LOOPBACK 1`` | `git checkout -- rtl/core/afdx_es_core.v` |
| `rtl/mac/mac_ip_top.v` | ①顶部加 `` `define AFDX_MAC_LOOPBACK 1``；②例化名改 `Tri_Mode_Ethernet_Mac_0`；③删 13 个 IP 侧不存在的口连接 | `git checkout -- rtl/mac/mac_ip_top.v` |

> 回退原因：iverilog 回归链接行为替身 `rtl/mac/mac_ip_behav.v`（模块名
> `Tri_Mode_Ethernet_Mac`，带指南全端口），例化名与端口表必须复原。

### B. 真缺陷修复（**保留，不回退**；GLM 需重跑 T6/T7 回归确认）
| 文件 | 缺陷 | 修法 |
|---|---|---|
| `rtl/lib/buf_mgr.v` | `smax_r` 被两个 always 块复位（同值 16'hFFFF 双驱动；iverilog 同值双驱动仿真侥幸，综合非法） | 删"主时序"块内复位副本（原 292 行），留注释 |
| `rtl/rx/rx_reassembly.v` | `busy[]/done[]` 被 W 引擎块与 R 读出块双驱动 | R 块两处释放（R_RUN 末字节同拍 r_ack / R_WAIT 补确认）迁入 W 块 endcase 后统一驱动；条件逐拍等价（rstate/r_valid/r_last/rctx_q/r_ack 均前值可读），同拍 W 侧与"读出中 done=1"互斥，无覆盖冲突，时序不变 |

### C. 工具/环境改动
| 位置 | 改动 | 处置 |
|---|---|---|
| `D:\elinx\eLinx3.0\bin\ip\GUI\ip_repository.xml` | Tri_Mode 条目 FlowType 999→0;1;2;6 | 建议**保留**（解锁厂商自带 IP）；还原则拷回 .bak |
| `sim/afdx_elinx/afdx_es.qsf` | **被 eLinx 综合时覆盖**为其生成的工程文件（20:20） | GLM 原稿在 git 中，`git checkout -- sim/afdx_elinx/afdx_es.qsf` 恢复 |
| `sim/afdx_elinx/afdx_ip_en.v` | 本轮新建（define 兜底尝试），现仅存磁盘、已移出工程、未跟踪 | 归档后可删；RUNBOOK 引用它的说法需更新 |

## 四、当前状态与下一步

- 状态：第 4 轮综合调试。已过：IP 解密/集成、buf_mgr 多驱动、rx_reassembly 多驱动、
  IP 端口裁剪（37/37 对齐）。
- **待办：Refresh Hierarchy → F11 重跑综合**。
- 预期：ehiway_map 分批报错，可能还有其他文件的 iverilog 习性双驱动/严格性错误，
  循环"发我 → 修 → 重跑"直至 `Synthesis Complete ✓`。
- 成功判据：Hierarchy 无红叉、`u_macA/u_macB → u_ip : Tri_Mode_Ethernet_Mac_0` 正常
  例化、LUT 数显著高于 T6/T7 基线（2×MAC IP + FIFO/CRC）。
- 失败处置：Messages 勾 Error 筛选，按报错逐个修。

## 五、综合成功后的收尾（未做）

1. Design Runs → 右键 `synth_1` → `Open Run Directory` → 收
   `*.vqm` + 综合报告（≈map.rpt）+ utilization 报告 → 拷入 `sim/afdx_elinx/out/`
2. 顶层 `.vo`：UG102 v2024.9 无 EDA Netlist Writer 菜单；**注意 IP 生成环节能产
   .vo/.vqm（OOC），顶层 .vo 是否可得待查**（Flow 菜单 / Settings→Simulation+
   3rd Party Simulators / Tcl `-help` 三处排查；无则如实报 GLM）
3. 执行"A. 临时改动回退"两条 git 命令（**不要**碰 B 类文件）
4. 向 GLM 交接口径（见"六"），回"GUI 产出已归档到 out/"

## 六、给 GLM 的口径更新要点

1. RUNBOOK §二.2 作废：GUI/CLI 均不可直读加密 .v → 正确路径 **IP Catalog +
   Out of context per IP**（PG171 第 4 章）。
2. ip_repository.xml FlowType=999 = 厂商隐藏 IP 手法；改含 0 即解锁。
3. IP 实际端口（37）≠ PG171 指南端口表：缺 13 口（清单见"二.4"），全双工/GMII/
   无过滤/无流控由定制配置固化，适配层已裁剪。
4. `mac_ip_behav.v` 替身口径不变（iverilog 回归），综合口径走 `Tri_Mode_Ethernet_Mac_0`。
5. buf_mgr.v / rx_reassembly.v 为**真 RTL 缺陷修复**（多驱动），需重跑回归；
   其余 define/例化名/端口裁剪为 GUI 综合口径，回归前 git 回退。
6. GLM 操作单"19 个加密文件"实为 17 个。
7. eLinx 综合时会覆盖工程目录同名 .qsf/.qpf（Quartus 同族引擎驱动文件）。

## 七、经验速查（下次直接用）

- eLinx 综合 = 生成 qsf/qpf → ehiway_map（Quartus 同族，`--read_settings_files=on`）
- 加密 IP 只能经 IP Catalog 生成（解密在 Generate 环节，产 .vo/.vqm OOC 网表）
- ehiway_map 拒绝多过程驱动同一 reg（iverilog 侥幸）→ 修法：指定唯一属主块，
  其他块的写迁移进属主块（条件可读则逐拍等价；否则加 1 拍握手需评估时序）
- 报错分批（凑 20 个即中止），逐轮修
- Sources 文件右键菜单无排序；Add Sources 列表仅本会话新增文件
- 判宏生效：Hierarchy 出现 mac_ip_top 且无红叉 + LUT 暴涨
