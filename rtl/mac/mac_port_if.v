//=============================================================================
// mac_port_if.v — 端口契约适配（v2 §4.2 mac；D1~D3 差异只在此体现）
// T6（纯仿真）：A/B 两网 GMII 观测直通——帧起始/结束选通输出供 tb 抓帧。
// T8（eLinx 集成）：本模块改为对接 Tri_Mode_Ethernet_Mac IP 的客户端/GMII
// 契约（.vo 网表联调），afdx_es_core 内部其余接口不动。
//=============================================================================

module mac_port_if #(
    parameter [1:0] NET_ID = 2'b01        // 01=A 网 10=B 网（观测标注用）
)(
    input  wire       clk,
    input  wire       reset,

    // 来自 mac_tx 的 GMII 发送侧
    input  wire [7:0] tx_d_in,
    input  wire       tx_en_in,

    // 对外（T6：tb 观测；T8：MAC IP 客户端口）
    output wire [7:0] gmii_d,
    output wire       gmii_en,
    output wire       gmii_er,            // 恒 0（无错误注入）
    output wire       frame_start,        // 单拍：tx_en 上升沿（tb 抓帧定位）
    output wire       frame_end           // 单拍：tx_en 下降沿
);

    reg tx_en_d;
    always @(posedge clk) begin
        if (reset) tx_en_d <= 1'b0;
        else       tx_en_d <= tx_en_in;
    end

    assign gmii_d       = tx_d_in;
    assign gmii_en      = tx_en_in;
    assign gmii_er      = 1'b0;
    assign frame_start  = tx_en_in && !tx_en_d;
    assign frame_end    = !tx_en_in && tx_en_d;

endmodule
