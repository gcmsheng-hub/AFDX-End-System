# README_BYPASS — 门级 e2e 旁路变体（官方答疑 2026-10-31~11-02 授权）

> **口径声明**：本目录是**官方答疑授权的后仿规避方案**配套件。AFDX 业务逻辑
> （组帧/调度/BAG/SN/分片/接收链/回环）**未变**，仅把「加密 MAC IP 边界」换成
> 语义等价的简化模型；**提交物仍为正常版本**（RTL + eLinx 工程 + 文档），
> 本变体只作诊断/规避证据，不入正常交付路径、不改 main。

## 1. 问题与结论

- 第 11 轮网表（`afdx_es.vqm`）+ `tb_delivery_e2e` 门级全量后仿：
  `[DELIV-TB] FAIL: watchdog timeout (delivered=0/6)`（2026-11-03 18:18:48，
  `Time: 60 ms`，`Errors: 0`）。
- 同 TB、同 RTL 设计（`mac_ip_behav` 行为替身）RTL 级 ：6/6 PASS，
  `$finish at 3892620000`（3.89ms）。
- 静态核查（`reports/PROBE-DIAG-001.md`）：回环连线在网表中完整
  （`vqm:512191` 的 `gmii_rx_a_d[0]~I .dataa = ...lb_d[0]`），虚拟 PHY 在网表，
  `rx_mac_aclk/tx_mac_aclk` 0 命中 → 失败是**动态**的，MAC 边界为唯一嫌疑段。
- 第 11 轮 vqm 为**单模块扁平网表**（`^module`=1），ModelSim 无 IP 实例可 bind
  → 实例级绑定路线不可行（30 分钟规则内已停）。

## 2. 变体是什么

`mac_ip_top_variant.v` = `rtl/mac/mac_ip_top.v` 的**接口完全一致**的替换件：

- 保留：TX 适配（IF_ID 按网替换）、skid FIFO、复位链、lb 回环 1 拍寄存、
  `gmii_*`/`frame_start/end`/`done`（IFG 12 拍）口径、`ev_*` 恒 0；
- 替换：删除 `Tri_Mode_Ethernet_Mac_0` 例化，改为**内联简化 MAC**——
  TX 流式（IDLE→7×0x55+SFD 0xD5→帧体直通→FCS32 低字节先发→IDLE），
  语义取自 `rtl/mac/mac_ip_behav.v`；不实现 RX（本设计 RX 走 `rtl/mac/mac_rx`）；
- 删除：MDIO 25MHz 分频与虚拟 PHY 依赖（变体不读 MDIO，link 恒有效）。

## 3. 已验证（RTL 级，2026-11-03 19:4x）

```
iverilog -g2001 -Wall -I tb <34 源，mac_ip_top 用本变体> -o tb_deliv_variant.vvp
vvp tb_deliv_variant.vvp
# [DELIV-TB] cfg done (3VL + globals + tx_en), t=801272000
# [DELIV-TB] injected 6 envelopes (3VL x2: 100B/2000B/1B), t=835080000
# [DELIV-TB] PASS (delivered 6/6: VL10 x2 VL11 x2 VL12 x2, payload byte-exact)
tb/tb_delivery_e2e.v:243: $finish called at 3892620000 (1ps)
```

**与正常 RTL 路径的 `$finish` 时间逐 ps 相同（3,892,620,000ps）**，说明变体在
本用例上与原设计行为等价。

## 4. 综合落点（给落执行 / 调度）

1. 在 eLinx 工程源列表里，用 `sim/psynth_bypass/mac_ip_top_variant.v`
   **替换** `rtl/mac/mac_ip_top.v`（模块名仍为 `mac_ip_top`，参数/端口一致，
   其余 29 个源文件与设置不动）；
2. IP Catalog 的 `Tri_Mode_Ethernet_Mac_0.vqm` 可保留（无例化 → 优化掉）
   或移除（更干净，二者均不影响其它源）；
3. Run Synthesis → 得到变体网表（建议另存，勿覆盖第 11 轮 `afdx_es.vqm`）；
4. 门级跑 `run_psynth_ms.sh`（TB 不变），判读标尺见 §5。

## 5. 变体门级判读标尺（与调度令一致）

- 注入时点约 **0.82ms**；RTL 全交付 **3.89ms**；
- 推进过 **5ms 仍零 delivered** → 判「同墙」，停跑回报；
- 任一 `delivered` 出现 → 立即回报并停手等裁；
- 短集（`tb_delivery_short`）用同一标尺；若时间允许再起 `tb_delivery_e2e`，
  逐封计数入档（`reports/PROBE-DIAG-001.md`）。

## 6. 边界

- 本变体只证明「综合后 AFDX 逻辑 + 简化 MAC 可交付」，**不能**证明加密 IP 本身
  在门级可工作；正常交付版本仍以 RTL 证据链（6/6）为准；
- 变体未入 `rtl/`、未改 main、未打 tag；仅在本探路分支。
