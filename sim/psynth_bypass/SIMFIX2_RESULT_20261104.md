# simfix2 定案（2026-11-04）

## 结论
综合后门级仿真（ModelSim SE-64 + eLinx 原语模型）**首次 3/3 PASS（payload byte-exact）**。
根因链两段：v1 补 `.ena(vcc)`/`.inverta(gnd)` 解决“寄存器不装载”，v2 补
`cin_used="true"` 解决“arithmetic 进位链被模型忽略”。

## v2 = v1 + cin_used
- 生成器：`sim/psynth_bypass/vqm_simfix2.py`（第一遍扫 arithmetic 且接 `.cin` 的实例，
  第二遍在实例块后追加 `defparam <inst> .cin_used = "true";`）
- 变体 `afdx_es_variant.simfix2.vqm` 自检：lcell=48,266 / add_ena=30,284 /
  add_ena_clocked=1,145 / add_inv=48,266 / **add_cin_used=6,194** / lut_dep_unconn=0
- 全量 `afdx_es_full.simfix2.vqm` 自检：lcell=51,444 / add_ena=32,726 /
  add_ena_clocked=2,245 / add_inv=51,444 / **add_cin_used=6,834** / lut_dep_unconn=0

## 探针（tb_probe_micro，10:53:40 起跑，11:01:23 止，Elapsed 0:07:43，Errors 0）
门级读数与 RTL 基准逐项、逐 ns 一致：P2 init_done=1 @144.992 µs；P5 DELIVERED
@396.464 µs；mismatches=0。

## 短集（tb_delivery_short，11:04:23 起跑，Elapsed 0:12:22，Errors 0）
```
[SHORT-TB] MILESTONE cfg-done 3VL+globals+tx_en t=801272000
[SHORT-TB] MILESTONE injected smoke(1B,VL12) t=801312000
[SHORT-TB] MILESTONE injected 100B(VL10) t=802144000
[SHORT-TB] delivered #1 vlid=c len=1 bad=0 t=802588000
[SHORT-TB] delivered #2 vlid=a len=100 bad=0 t=805676000
[SHORT-TB] MILESTONE injected 2000B(VL11,fragmented) t=818176000
[SHORT-TB] delivered #3 vlid=b len=2000 bad=0 t=1872612000
[SHORT-TB] PASS (delivered 3/3: smoke(1B) x1, 100B x1, 2000B(fragmented) x1, payload byte-exact) t=1873612000
```
对照：同一 TB 在 v1 网表下跑 232 分钟 0 交付；v2 网表 12 分 22 秒内 3/3 交付通过。

## 产物（均为新增，原 .vqm / v1 件未动）
- `sim/psynth_bypass/vqm_simfix2.py`
- `sim/psynth_bypass/run_psynth_variant_simfix2.sh`（探针）
- `sim/psynth_bypass/run_psynth_variant_simfix2_short.sh`（短集）
- `sim/afdx_elinx_variant/out/psynth_variant_ms/afdx_es_variant.simfix2.vqm`
- `sim/afdx_elinx/out/psynth_fullsimfix_ms/afdx_es_full.simfix2.vqm`
- 证据归档：`_archive_mp_v2_20261104/`（v2 探针日志/transcript/work）
