# simfix3 定案（2026-11-04）：全量网表门级 3/3 PASS

## 前置
- simfix2 全量短集 11:29 起跑，13:49 按令提前停跑（探针已给出零交付判据、根因已定），
  停前 [SHORT-TB] 仅 4 行里程碑（cfg-done + 三注入），零交付；日志原样保留于
  `out/psynth_fullsimfix2_ms/logs/vsim_run.log`。
- 对照探针（simfix2 全量 + tb_probe_micro）：poll#1–#6（396→1646 µs）
  `tx_frames=0 rx_frames=0 rx_msgs=0`，SUMMARY delivered=0，Elapsed 1:00:00。

## 静态预检（Pass 4 的 8 单元，simfix2 件原文）
| 单元 | lut_mask（现值=预期前置） | .ena 现值 |
|---|---|---|
| u_macA/u_macB `link~I` | AA00 / AA00 | 逻辑网 `…|link~2` |
| u_macA/u_macB `speed[1]~I` | F000 / F000 | `…|link~2` |
| u_macA/u_macB `speed[0]~I` | CC00 / CC00 | `…|link~2` |
| u_macA/u_macB `mac_duplex_mode~I` | F000 / F000 | `…|link~2` |
全部相符，未触发停报。

## simfix3 增量（vqm_simfix3.py，输入 simfix2 件）
- Pass 3（2 处 pause_tx_mac ROM）：`init_file_layout "Port_A"→"port_a"`（模型只在严格
  小写时装载 mem_init），补 `mem_init0 = 256'h…`（MIF：addr1–72=1，其余 0，值=2^73−2；
  解码复核 64 hex digits / 72 ones ✅）。
- Pass 4（8 单元）：`.ena(link~2)→.ena(vcc)`；`lut_mask` AA00→AAAA、F000→FF00、
  CC00→0000、F000→FF00。
- 自检：`+mdio_fastlink: 8 (ena 8 / mask 8)`、`+rom_init: 2 (layout 2 / mem_init0 2)`。
- 差异范围：与 simfix2 件相异仅 20 行、全部落在这 10 个实例内（8×ena、8×mask、
  2×layout、2×mem_init0）。

## 探针（全量 simfix3 + tb_probe_micro，13:51:57 起）
```
P1 STATUS=0x00000000 @0.824us
P2 INIT-DONE init_done=1 @144.992us (poll_cyc=18000)
P3 TX W2@0x01052=0xA / W0@0x01050=0x80000001 / idx20 W0@0x010A0=0
P4 g_msgs=1 @146.400us
P5 poll#1 tx_frames=1 rx_frames=1 rx_msgs=1 @396.464us
DELIVERED rx_msgs=1 tx_frames=1 rx_frames=1 @396.464us
SUMMARY mismatches=0 delivered=1
```
Elapsed 0:15:54，Errors 0。

## 短集（同库续跑 tb_delivery_short，14:09 起）
```
[SHORT-TB] MILESTONE cfg-done 3VL+globals+tx_en t=801272000
[SHORT-TB] MILESTONE injected smoke(1B,VL12) t=801312000
[SHORT-TB] MILESTONE injected 100B(VL10) t=802144000
[SHORT-TB] delivered #1 vlid=c len=1 bad=0 t=803100000
[SHORT-TB] delivered #2 vlid=a len=100 bad=0 t=807364000
[SHORT-TB] MILESTONE injected 2000B(VL11,fragmented) t=818176000
[SHORT-TB] delivered #3 vlid=b len=2000 bad=0 t=1889276000
[SHORT-TB] PASS (delivered 3/3: smoke(1B) x1, 100B x1, 2000B(fragmented) x1, payload byte-exact) t=1890276000
```
Elapsed 0:46:33，Errors 0。

## 产物（新增，原网表与 simfix1/2 件均未动）
- `sim/psynth_bypass/vqm_simfix3.py`
- `sim/psynth_bypass/run_psynth_fullsim3_mp.sh`
- `sim/psynth_bypass/run_simfix3_short_samework.sh`
- `sim/afdx_elinx/out/psynth_fullsimfix_ms/afdx_es_full.simfix3.vqm`
- 工作目录：`out/psynth_fullsim3_mp`（探针 + 短集同库）、`out/psynth_fullsim2_mp`（对照）
