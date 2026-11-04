vlib work
vlog -sv -work work tb/tb_delivery_e2e.v rtl/AFDX_top.v rtl/core/afdx_es_core.v rtl/core/cfg_bus.v rtl/core/ram_init.v rtl/core/timebase.v rtl/core/host_arb.v rtl/core/stats.v rtl/lib/buf_mgr.v rtl/lib/sdp_ram.v rtl/lib/sync_fifo.v rtl/lib/rr_arbiter.v rtl/lib/tick_gen.v rtl/lib/crc32_d8.v rtl/lib/csum16.v rtl/tx/tx_vl_table.v rtl/tx/tx_regulator.v rtl/tx/tx_scheduler.v rtl/tx/tx_sn_gen.v rtl/tx/tx_frame_builder.v rtl/mac/mac_ip_top.v rtl/mac/mac_ip_behav.v rtl/mac/mdio_phy_sim.v rtl/mac/mac_rx.v rtl/rx/rx_path.v rtl/rx/rx_parser.v rtl/rx/rx_vl_lookup.v rtl/rx/rx_integrity.v rtl/rx/rx_redundancy.v rtl/rx/rx_reassembly.v
vsim work.tb_delivery_e2e
add wave /tb_delivery_e2e/clk
add wave /tb_delivery_e2e/reset
add wave -radix hex /tb_delivery_e2e/tx_data
add wave /tb_delivery_e2e/tx_valid
add wave -radix hex /tb_delivery_e2e/rx_data
add wave /tb_delivery_e2e/rx_valid
run -all