//=============================================================================
// rx_parser.v — 接收头部解析与检查（v2 §4.2 rx_parser / §5.5；doc/07 §2.3-2）
//
// 输入：帧体前 42 字节顺序流（hp_start 后 42 拍 hp_valid，帧偏移 0..41）。
// 检查项（任何一项失败由调用方丢弃并计数，v2 §5.5 口径）：
//   hp_ok_mac   目的 MAC 前 4B == 03:00:00:00（组播常量，doc/01 §1）
//   hp_ok_etype EtherType == 0x0800（IPv4）
//   hp_ok_ip    ver/ihl == 0x45 且 proto == 17(UDP) 且 IP 头校验和对
//               （校验和域清零后经 csum16 复算，与 afdx_ref.ip_csum 同口径）
//   hp_ok_udp   单帧：UDP Length ≥ 8 且 IP 总长 == 20+UDP Length；
//               首片：UDP Length ≥ 9（UDP Length 填整条消息总长，ISS-06，
//               与单片 IP 总长不一致是设计内行为）；中/尾片：调用方忽略
// 提取字段：vlid（dst MAC 低 16b）、sip、ip_id、mf/off（片标志/偏移）、
//           sport/dport/ulen（UDP 头）。
// 时序：字节流 1B/拍；hp_done 在末字节（偏移 41）处理后 1 拍单拍输出，
//       届时全部 hp_ok_* / 字段寄存稳定。总耗时 43 拍/帧头。
//=============================================================================

module rx_parser (
    input  wire        clk,
    input  wire        reset,

    input  wire        hp_start,          // 单拍：启动（随后 42 拍顺序字节）
    input  wire        hp_valid,
    input  wire [7:0]  hp_data,

    output reg         hp_done,           // 单拍：头部处理完，以下输出有效
    output reg         hp_ok_mac,
    output reg         hp_ok_etype,
    output reg         hp_ok_ip,
    output reg         hp_ok_udp,
    output reg  [15:0] hp_vlid,
    output reg  [31:0] hp_sip,
    output reg  [15:0] hp_ipid,
    output reg  [15:0] hp_iptot,          // IP 总长度字段
    output reg         hp_mf,
    output reg  [12:0] hp_off,            // 片偏移（8B 单位）
    output reg  [15:0] hp_sport,
    output reg  [15:0] hp_dport,
    output reg  [15:0] hp_ulen
);

    reg [5:0] cnt;                        // 帧偏移 0..41

    // IP 头校验和累加（偏移 14..33 共 20B；校验和域清零喂入）
    wire       in_ip   = (cnt >= 6'd14) && (cnt <= 6'd33);
    wire       in_csum = (cnt == 6'd24) || (cnt == 6'd25);
    wire [15:0] csum_w;
    csum16 u_cs (
        .clk(clk), .reset(reset),
        .clr(hp_start),
        .valid(hp_valid && in_ip),
        .data(in_csum ? 8'h00 : hp_data),
        .csum(csum_w));

    // 头部暂存（校验域比对用；ip_total/ulen 直接入输出寄存器）
    reg [15:0] csum_q;
    reg        ver_ok, proto_ok;

    always @(posedge clk) begin
        if (reset) begin
            cnt <= 6'd0;
            hp_done <= 1'b0;
            hp_ok_mac <= 1'b0; hp_ok_etype <= 1'b0;
            hp_ok_ip <= 1'b0;  hp_ok_udp <= 1'b0;
            hp_vlid <= 16'h0;  hp_sip <= 32'h0;
            hp_ipid <= 16'h0;  hp_mf <= 1'b0; hp_off <= 13'h0;
            hp_sport <= 16'h0; hp_dport <= 16'h0; hp_ulen <= 16'h0;
            hp_iptot <= 16'h0;
            csum_q <= 16'h0;
            ver_ok <= 1'b0; proto_ok <= 1'b0;
        end else begin
            hp_done <= 1'b0;

            if (hp_start)
                cnt <= 6'd0;

            if (hp_valid) begin
                case (cnt)
                    // 目的 MAC：前 4B 常量 03:00:00:00（doc/01 §1 组播映射）
                    6'd0: hp_ok_mac <= (hp_data == 8'h03);
                    6'd1: hp_ok_mac <= hp_ok_mac && (hp_data == 8'h00);
                    6'd2: hp_ok_mac <= hp_ok_mac && (hp_data == 8'h00);
                    6'd3: hp_ok_mac <= hp_ok_mac && (hp_data == 8'h00);
                    // VLID（dst MAC 低 16b，大端）
                    6'd4: hp_vlid[15:8] <= hp_data;
                    6'd5: hp_vlid[7:0]  <= hp_data;
                    // EtherType
                    6'd12: hp_ok_etype <= (hp_data == 8'h08);
                    6'd13: hp_ok_etype <= hp_ok_etype && (hp_data == 8'h00);
                    // IPv4 头（14..33）
                    6'd14: ver_ok <= (hp_data == 8'h45);       // ver=4, IHL=5
                    6'd16: hp_iptot[15:8] <= hp_data;
                    6'd17: hp_iptot[7:0]  <= hp_data;
                    6'd18: hp_ipid[15:8] <= hp_data;
                    6'd19: hp_ipid[7:0]  <= hp_data;
                    6'd20: begin                               // 标志/片偏移高字节
                        hp_mf        <= hp_data[5];            // MF = 16b 域 bit13（0x2000）
                        hp_off[12:8] <= hp_data[4:0];          // 偏移[12:8]（8B 单位）
                    end
                    6'd21: hp_off[7:0] <= hp_data;             // 偏移[7:0]
                    6'd23: proto_ok <= (hp_data == 8'h11);     // UDP
                    6'd24: csum_q[15:8] <= hp_data;
                    6'd25: csum_q[7:0]  <= hp_data;
                    6'd26: hp_sip[31:24] <= hp_data;
                    6'd27: hp_sip[23:16] <= hp_data;
                    6'd28: hp_sip[15:8]  <= hp_data;
                    6'd29: hp_sip[7:0]   <= hp_data;
                    // UDP 头（34..41；中/尾片此区为数据，调用方按 off 忽略）
                    6'd34: hp_sport[15:8] <= hp_data;
                    6'd35: hp_sport[7:0]  <= hp_data;
                    6'd36: hp_dport[15:8] <= hp_data;
                    6'd37: hp_dport[7:0]  <= hp_data;
                    6'd38: hp_ulen[15:8]  <= hp_data;
                    6'd39: hp_ulen[7:0]   <= hp_data;
                    default: ;
                endcase

                // 末字节处理完 → 下一拍 hp_done（csum16 末拍已在本沿推进）
                // ok_udp 口径（v2 §5.5 一致性 + ISS-06 首片 UDP Length=整条消息长）：
                //   单帧（off==0,mf=0）：ulen ≥ 8 且 iptot == 20+ulen
                //   首片（off==0,mf=1）：ulen ≥ 9（重组总载荷 = ulen-8 ≥ 1）
                //   中/尾片（off≠0）：调用方忽略（34..41 为数据）
                if (cnt == 6'd41) begin
                    hp_done   <= 1'b1;
                    hp_ok_ip  <= ver_ok && proto_ok && (csum_w == csum_q);
                    hp_ok_udp <= hp_mf ? (hp_ulen >= 16'd9)
                                       : ((hp_ulen >= 16'd8)
                                          && (hp_iptot == 16'd20 + hp_ulen));
                end else if (cnt != 6'd42) begin
                    cnt <= cnt + 6'd1;                         // 饱和于 42
                end
            end
        end
    end

endmodule
