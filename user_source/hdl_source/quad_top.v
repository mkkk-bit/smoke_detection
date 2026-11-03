module quad_top (
    input              I_clk,
    input              I_rst_n,
    input              I_key,          // KEY1 (低有效, 直连 I_button[1])
    input              I_view_enable,  // 阈值调节期间屏蔽短按视图切换
    input              I_sw2,          // SW2 抓拍使能 (拨下=1, wrapper已反相): 显示右下角相机图标
    input      [7:0]   I_snap_count,   // completed snapshot count for success feedback
    input      [3:0]   I_sw_raw,       // 原始物理拨片电平: SW1..SW4 -> I_sw[0]..I_sw[3]
    input      [10:0]  I_roi_left,
    input      [10:0]  I_roi_right,
    // 原图流 (hdmi_mixer 输出, 含候选计数OSD, 相对判定流早 5 拍)
    input              I_vid_de,
    input              I_vid_hsync,
    input              I_vid_vsync,
    input  [23:0]      I_vid_data,
    // 判定流 (osd_box + ALARM + 状态面板 + 可信度面板, = 原图+5拍)
    input              I_det_de,
    input              I_det_hsync,
    input              I_det_vsync,
    input  [23:0]      I_det_data,
    output reg         O_de,
    output reg         O_hsync,
    output reg         O_vsync,
    output reg [23:0]  O_data,
    output reg [2:0]   O_view_mode,    // 0=QUAD 1=PROCESS QUAD 2=ORIG 3=EDGE 4=MASK 5=TARGET
    output reg         O_quad_mode,    // 1=任一四分屏
    output reg         O_process_quad  // 1=图像处理对比四分屏
);

// ============================================================
// 四分屏/全屏视图切换 + KEY1 六态循环 + 视图OSD (时钟 S_hdmi_pixel_clk)
// 输入两路同源显示流 (同一 VTC 时序, 判定流=原图流+5拍):
//   I_vid_* = 原图流 (hdmi_mixer 输出, 含候选计数OSD)
//   I_det_* = 判定流 (框选/ALARM/状态/可信度 OSD 合成后的画面)
// 内部重建: 边缘流 (+2拍->补3拍), 掩码流 (+1拍->补4拍)
// 四路对齐到 判定流相位(+5拍) 后 4:1 mux 寄存 1 拍, 再 1 拍输出寄存器 (总 +7 拍)
// view_mode: 0=检测四分屏 1=处理四分屏 2=ORIG 3=EDGE 4=MASK 5=TARGET
// QUAD 模式: 原图流即 DDR 拼接帧 (vo 读拼接缓冲), 判定流叠加红框/ALARM
//   (坐标已由 wrapper 按 (x>>1)+640,(y>>1)+360 变换到右下象限);
//   叠加白分隔线(2px) + 四象限标签 (2倍字模, 白字+深色底条)
// 全屏模式: 右上角模式名 (ORIGINAL/EDGE/MASK/DETECT)
// 覆盖层像素计数器锚定 mux 后 hsync/vsync (与 osd_alarm 同款自校准,
//   与 mux 数据同拍, 输出寄存后与 O_hsync/O_vsync 精确同相位)
// 切换瞬间无复位无清缓存, 纯 mux 换源 (画面突变 1 帧, 零丢帧)
// ============================================================

    // ---- KEY1 消抖 + 六态循环 (两个四分屏 + 四个诊断单屏) ----
    wire S_key_trig;
    key_remove_shakes u_key_shakes(
        .I_clk          (I_clk),
        .I_rst_n        (I_rst_n),
        .I_key_in       (~I_key),
        .O_key_trig_out (S_key_trig)
    );
    reg [2:0] view_mode;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            view_mode <= 3'd0;
        else if(S_key_trig && I_view_enable)
            view_mode <= (view_mode == 3'd5) ? 3'd0 : view_mode + 3'd1;
    end

    // ---- 亮度 Y=(77R+150G+29B)>>8 (sobel 输入) ----
    wire [17:0] Y_full = I_vid_data[23:16]*8'd77 + I_vid_data[15:8]*8'd150 + I_vid_data[7:0]*8'd29;
    wire [7:0]  Y = Y_full[15:8];

    // ---- 边缘流 (+2拍) ----
    wire [7:0]  S_edge_dout;
    wire        S_edge_de, S_edge_hs, S_edge_vs;
    sobel_disp u_sobel(
        .I_clk       (I_clk),
        .I_rst_n     (I_rst_n),
        .I_din       (Y),
        .I_de        (I_vid_de),
        .I_hsync     (I_vid_hsync),
        .I_vsync     (I_vid_vsync),
        .O_dout      (S_edge_dout),
        .O_de_out    (S_edge_de),
        .O_hsync_out (S_edge_hs),
        .O_vsync_out (S_edge_vs)
    );

    // ---- 掩码流 (+1拍) ----
    wire [23:0] S_mask_dout;
    wire        S_mask_de, S_mask_hs, S_mask_vs;
    mask_disp u_mask(
        .I_clk       (I_clk),
        .I_rst_n     (I_rst_n),
        .I_de        (I_vid_de),
        .I_hsync     (I_vid_hsync),
        .I_vsync     (I_vid_vsync),
        .I_din       (I_vid_data),
        .O_dout      (S_mask_dout),
        .O_de_out    (S_mask_de),
        .O_hsync_out (S_mask_hs),
        .O_vsync_out (S_mask_vs)
    );

    // ---- 延迟对齐 (统一到 原图+5拍 = 判定流相位) ----
    // 原图 +0 -> 补5; 边缘 +2 -> 补3; 掩码 +1 -> 补4; 判定 +5 -> 补0
    reg [23:0] S_orig_d1, S_orig_d2, S_orig_d3, S_orig_d4, S_orig_d5;
    reg        S_orig_de1, S_orig_hs1, S_orig_vs1;
    reg        S_orig_de2, S_orig_hs2, S_orig_vs2;
    reg        S_orig_de3, S_orig_hs3, S_orig_vs3;
    reg        S_orig_de4, S_orig_hs4, S_orig_vs4;
    reg        S_orig_de5, S_orig_hs5, S_orig_vs5;
    reg [23:0] S_edge_d2, S_edge_d3, S_edge_d4;
    reg        S_edge_de2, S_edge_hs2, S_edge_vs2;
    reg        S_edge_de3, S_edge_hs3, S_edge_vs3;
    reg        S_edge_de4, S_edge_hs4, S_edge_vs4;
    reg [23:0] S_mask_d2, S_mask_d3, S_mask_d4, S_mask_d5;
    reg        S_mask_de2, S_mask_hs2, S_mask_vs2;
    reg        S_mask_de3, S_mask_hs3, S_mask_vs3;
    reg        S_mask_de4, S_mask_hs4, S_mask_vs4;
    reg        S_mask_de5, S_mask_hs5, S_mask_vs5;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_orig_d1 <= 24'd0; S_orig_de1 <= 1'b0; S_orig_hs1 <= 1'b0; S_orig_vs1 <= 1'b0;
            S_orig_d2 <= 24'd0; S_orig_de2 <= 1'b0; S_orig_hs2 <= 1'b0; S_orig_vs2 <= 1'b0;
            S_orig_d3 <= 24'd0; S_orig_de3 <= 1'b0; S_orig_hs3 <= 1'b0; S_orig_vs3 <= 1'b0;
            S_orig_d4 <= 24'd0; S_orig_de4 <= 1'b0; S_orig_hs4 <= 1'b0; S_orig_vs4 <= 1'b0;
            S_orig_d5 <= 24'd0; S_orig_de5 <= 1'b0; S_orig_hs5 <= 1'b0; S_orig_vs5 <= 1'b0;
            S_edge_d2 <= 24'd0; S_edge_de2 <= 1'b0; S_edge_hs2 <= 1'b0; S_edge_vs2 <= 1'b0;
            S_edge_d3 <= 24'd0; S_edge_de3 <= 1'b0; S_edge_hs3 <= 1'b0; S_edge_vs3 <= 1'b0;
            S_edge_d4 <= 24'd0; S_edge_de4 <= 1'b0; S_edge_hs4 <= 1'b0; S_edge_vs4 <= 1'b0;
            S_mask_d2 <= 24'd0; S_mask_de2 <= 1'b0; S_mask_hs2 <= 1'b0; S_mask_vs2 <= 1'b0;
            S_mask_d3 <= 24'd0; S_mask_de3 <= 1'b0; S_mask_hs3 <= 1'b0; S_mask_vs3 <= 1'b0;
            S_mask_d4 <= 24'd0; S_mask_de4 <= 1'b0; S_mask_hs4 <= 1'b0; S_mask_vs4 <= 1'b0;
            S_mask_d5 <= 24'd0; S_mask_de5 <= 1'b0; S_mask_hs5 <= 1'b0; S_mask_vs5 <= 1'b0;
        end else begin
            S_orig_d1 <= I_vid_data;  S_orig_de1 <= I_vid_de;  S_orig_hs1 <= I_vid_hsync; S_orig_vs1 <= I_vid_vsync;
            S_orig_d2 <= S_orig_d1;   S_orig_de2 <= S_orig_de1; S_orig_hs2 <= S_orig_hs1; S_orig_vs2 <= S_orig_vs1;
            S_orig_d3 <= S_orig_d2;   S_orig_de3 <= S_orig_de2; S_orig_hs3 <= S_orig_hs2; S_orig_vs3 <= S_orig_vs2;
            S_orig_d4 <= S_orig_d3;   S_orig_de4 <= S_orig_de3; S_orig_hs4 <= S_orig_hs3; S_orig_vs4 <= S_orig_vs3;
            S_orig_d5 <= S_orig_d4;   S_orig_de5 <= S_orig_de4; S_orig_hs5 <= S_orig_hs4; S_orig_vs5 <= S_orig_vs4;
            S_edge_d2 <= {S_edge_dout, S_edge_dout, S_edge_dout};   // 灰度白/黑 -> RGB
            S_edge_de2 <= S_edge_de; S_edge_hs2 <= S_edge_hs; S_edge_vs2 <= S_edge_vs;
            S_edge_d3 <= S_edge_d2; S_edge_de3 <= S_edge_de2; S_edge_hs3 <= S_edge_hs2; S_edge_vs3 <= S_edge_vs2;
            S_edge_d4 <= S_edge_d3; S_edge_de4 <= S_edge_de3; S_edge_hs4 <= S_edge_hs3; S_edge_vs4 <= S_edge_vs3;
            S_mask_d2 <= S_mask_dout; S_mask_de2 <= S_mask_de; S_mask_hs2 <= S_mask_hs; S_mask_vs2 <= S_mask_vs;
            S_mask_d3 <= S_mask_d2;   S_mask_de3 <= S_mask_de2; S_mask_hs3 <= S_mask_hs2; S_mask_vs3 <= S_mask_vs2;
            S_mask_d4 <= S_mask_d3;   S_mask_de4 <= S_mask_de3; S_mask_hs4 <= S_mask_hs3; S_mask_vs4 <= S_mask_vs3;
            S_mask_d5 <= S_mask_d4;   S_mask_de5 <= S_mask_de4; S_mask_hs5 <= S_mask_hs4; S_mask_vs5 <= S_mask_vs4;
        end
    end

    // ---- 4:1 mux (对齐点: 判定流+5拍, 四路 hsync/vsync 同相位) ----
    reg [23:0] S_mux_data;
    reg        S_mux_de, S_mux_hs, S_mux_vs;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_mux_data <= 24'd0;
            S_mux_de <= 1'b0; S_mux_hs <= 1'b0; S_mux_vs <= 1'b0;
        end else begin
            case(view_mode)
            3'd2: begin   // ORIG
                S_mux_data <= S_orig_d5;
                S_mux_de <= S_orig_de5; S_mux_hs <= S_orig_hs5; S_mux_vs <= S_orig_vs5;
            end
            3'd3: begin   // EDGE
                S_mux_data <= S_edge_d4;
                S_mux_de <= S_edge_de4; S_mux_hs <= S_edge_hs4; S_mux_vs <= S_edge_vs4;
            end
            3'd4: begin   // MASK
                S_mux_data <= S_mask_d5;
                S_mux_de <= S_mask_de5; S_mux_hs <= S_mask_hs5; S_mux_vs <= S_mask_vs5;
            end
            default: begin  // QUAD pages and TARGET: 判定流 (原图+红框+ALARM文字)
                S_mux_data <= I_det_data;
                S_mux_de <= I_det_de; S_mux_hs <= I_det_hsync; S_mux_vs <= I_det_vsync;
            end
            endcase
        end
    end

    // ---- 像素计数器 (锚定 mux 后同步信号, 与 S_mux_data 同拍; 同 osd_alarm 方案) ----
    reg S_mux_hs_d1, S_mux_vs_d1;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_mux_hs_d1 <= 1'b0;
            S_mux_vs_d1 <= 1'b0;
        end else begin
            S_mux_hs_d1 <= S_mux_hs;
            S_mux_vs_d1 <= S_mux_vs;
        end
    end
    wire hs_rise = S_mux_hs && !S_mux_hs_d1;
    wire vs_rise = S_mux_vs && !S_mux_vs_d1;

    localparam [10:0] H_FRAME      = 11'd1650; // 行总拍数
    localparam [10:0] H_ACTIVE     = 11'd1280; // 行有效像素
    localparam [10:0] H_SYNC_START = 11'd1390; // hsync 起始相位(锚点)
    localparam [9:0]  V_FRAME      = 10'd750;  // 场总行数
    localparam [9:0]  V_ACTIVE     = 10'd720;  // 场有效行
    localparam [9:0]  V_SYNC_START = 10'd725;  // vsync 起始相位(锚点)
    reg [10:0] x_cnt;
    reg [9:0]  y_cnt;
    reg        locked;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            x_cnt <= 11'd0;
        else if(hs_rise)
            x_cnt <= H_SYNC_START + 11'd1;
        else if(x_cnt == H_FRAME - 11'd1)
            x_cnt <= 11'd0;
        else
            x_cnt <= x_cnt + 11'd1;
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            y_cnt  <= 10'd0;
            locked <= 1'b0;
        end else begin
            if(vs_rise) begin
                y_cnt  <= V_SYNC_START;
                locked <= 1'b1;
            end else if(x_cnt == H_FRAME - 11'd1) begin
                if(y_cnt == V_FRAME - 10'd1)
                    y_cnt <= 10'd0;
                else
                    y_cnt <= y_cnt + 10'd1;
            end
        end
    end
    wire de_i = locked && (x_cnt < H_ACTIVE) && (y_cnt < V_ACTIVE);

    // ---- 覆盖上下文选择 (当前像素所属标签区) ----
    // QUAD: 四象限标签; 全屏: 右上角模式名
    reg [10:0] cur_x0;
    reg [9:0]  cur_y0;
    reg [1:0]  cur_s;
    reg        cur_en;
    always @* begin
        cur_en = 1'b0; cur_x0 = 11'd0; cur_y0 = 10'd0; cur_s = 2'd0;
        if((view_mode == 3'd0) || (view_mode == 3'd1)) begin
            // 四象限标签 (区域互不重叠)
            if     ((x_cnt >= 11'd8)   && (x_cnt < 11'd90)  && (y_cnt >= 10'd4)   && (y_cnt < 10'd20))  begin cur_en = 1'b1; cur_x0 = 11'd8;   cur_y0 = 10'd4;   cur_s = 2'd0; end
            else if((x_cnt >= 11'd648) && (x_cnt < 11'd730) && (y_cnt >= 10'd4)   && (y_cnt < 10'd20))  begin cur_en = 1'b1; cur_x0 = 11'd648; cur_y0 = 10'd4;   cur_s = 2'd1; end
            else if((x_cnt >= 11'd8)   && (x_cnt < 11'd90)  && (y_cnt >= 10'd364) && (y_cnt < 10'd380)) begin cur_en = 1'b1; cur_x0 = 11'd8;   cur_y0 = 10'd364; cur_s = 2'd2; end
            else if((x_cnt >= 11'd648) && (x_cnt < 11'd730) && (y_cnt >= 10'd364) && (y_cnt < 10'd380)) begin cur_en = 1'b1; cur_x0 = 11'd648; cur_y0 = 10'd364; cur_s = 2'd3; end
        end else begin
            case(view_mode)
            3'd2: begin cur_en = 1'b1; cur_x0 = 11'd1194; cur_y0 = 10'd0; cur_s = 2'd0; end   // ORIGINAL
            3'd3: begin cur_en = 1'b1; cur_x0 = 11'd1194; cur_y0 = 10'd0; cur_s = 2'd1; end   // EDGE MAP
            3'd4: begin cur_en = 1'b1; cur_x0 = 11'd1194; cur_y0 = 10'd0; cur_s = 2'd2; end   // RED MASK
            3'd5: begin cur_en = 1'b1; cur_x0 = 11'd1194; cur_y0 = 10'd0; cur_s = 2'd3; end   // TARGET
            default: ;
            endcase
        end
    end

    // ---- 标签字符串表: s 的第 i 字符 ----
    function [4:0] lbl_char;
        input [1:0] s;
        input [2:0] i;
        input process_page;
        begin
            if(process_page) begin
                case({s,i})
                    5'd0:lbl_char=15; 5'd1:lbl_char=2; 5'd2:lbl_char=4; 5'd3:lbl_char=5; 5'd4:lbl_char=1; 5'd5:lbl_char=17;
                    5'd8:lbl_char=3; 5'd9:lbl_char=5; 5'd10:lbl_char=9; 5'd11:lbl_char=9; 5'd12:lbl_char=5;
                    5'd16:lbl_char=10; 5'd17:lbl_char=5; 5'd18:lbl_char=13;
                    5'd24:lbl_char=16; 5'd25:lbl_char=7; 5'd26:lbl_char=5; 5'd27:lbl_char=13; 5'd28:lbl_char=31; 5'd29:lbl_char=9; 5'd30:lbl_char=5; 5'd31:lbl_char=14;
                    default:lbl_char=31;
                endcase
            end else begin
                case({s,i})
                    5'd0:lbl_char=0; 5'd1:lbl_char=1; 5'd2:lbl_char=2; 5'd3:lbl_char=3; 5'd4:lbl_char=2; 5'd5:lbl_char=4; 5'd6:lbl_char=5; 5'd7:lbl_char=6;
                    5'd8:lbl_char=7; 5'd9:lbl_char=8; 5'd10:lbl_char=3; 5'd11:lbl_char=7; 5'd12:lbl_char=31; 5'd13:lbl_char=9; 5'd14:lbl_char=5; 5'd15:lbl_char=14;
                    5'd16:lbl_char=1; 5'd17:lbl_char=7; 5'd18:lbl_char=8; 5'd19:lbl_char=31; 5'd20:lbl_char=9; 5'd21:lbl_char=5; 5'd22:lbl_char=10; 5'd23:lbl_char=11;
                    5'd24:lbl_char=13; 5'd25:lbl_char=5; 5'd26:lbl_char=1; 5'd27:lbl_char=3; 5'd28:lbl_char=5; 5'd29:lbl_char=13;
                    default:lbl_char=31;
                endcase
            end
        end
    endfunction

    // ---- 8x14 标签字模: 字体风格与 osd_alarm 的 ALARM 字模一致 ----
    // 每个5x7点阵行重复两次, 保持小字号, 形成清楚、粗实的14像素字高。
    function [4:0] label_row5;
        input [4:0] ch;
        input [2:0] row;
        begin
            case (ch)
            4'd0: case(row) // O
                0:label_row5=5'b01110; 1:label_row5=5'b10001; 2:label_row5=5'b10001;
                3:label_row5=5'b10001; 4:label_row5=5'b10001; 5:label_row5=5'b10001;
                6:label_row5=5'b01110; default:label_row5=5'b00000;
            endcase
            4'd1: case(row) // R
                0:label_row5=5'b11110; 1:label_row5=5'b10001; 2:label_row5=5'b10001;
                3:label_row5=5'b11110; 4:label_row5=5'b10100; 5:label_row5=5'b10010;
                6:label_row5=5'b10001; default:label_row5=5'b00000;
            endcase
            4'd2: case(row) // I
                0:label_row5=5'b11111; 1:label_row5=5'b00100; 2:label_row5=5'b00100;
                3:label_row5=5'b00100; 4:label_row5=5'b00100; 5:label_row5=5'b00100;
                6:label_row5=5'b11111; default:label_row5=5'b00000;
            endcase
            4'd3: case(row) // G
                0:label_row5=5'b01110; 1:label_row5=5'b10001; 2:label_row5=5'b10000;
                3:label_row5=5'b10111; 4:label_row5=5'b10001; 5:label_row5=5'b10001;
                6:label_row5=5'b01110; default:label_row5=5'b00000;
            endcase
            4'd4: case(row) // N
                0:label_row5=5'b10001; 1:label_row5=5'b11001; 2:label_row5=5'b10101;
                3:label_row5=5'b10011; 4:label_row5=5'b10001; 5:label_row5=5'b10001;
                6:label_row5=5'b10001; default:label_row5=5'b00000;
            endcase
            4'd5: case(row) // A
                0:label_row5=5'b01110; 1:label_row5=5'b10001; 2:label_row5=5'b10001;
                3:label_row5=5'b11111; 4:label_row5=5'b10001; 5:label_row5=5'b10001;
                6:label_row5=5'b10001; default:label_row5=5'b00000;
            endcase
            4'd6: case(row) // L
                0:label_row5=5'b10000; 1:label_row5=5'b10000; 2:label_row5=5'b10000;
                3:label_row5=5'b10000; 4:label_row5=5'b10000; 5:label_row5=5'b10000;
                6:label_row5=5'b11111; default:label_row5=5'b00000;
            endcase
            4'd7: case(row) // E
                0:label_row5=5'b11111; 1:label_row5=5'b10000; 2:label_row5=5'b10000;
                3:label_row5=5'b11110; 4:label_row5=5'b10000; 5:label_row5=5'b10000;
                6:label_row5=5'b11111; default:label_row5=5'b00000;
            endcase
            4'd8: case(row) // D
                0:label_row5=5'b11110; 1:label_row5=5'b10001; 2:label_row5=5'b10001;
                3:label_row5=5'b10001; 4:label_row5=5'b10001; 5:label_row5=5'b10001;
                6:label_row5=5'b11110; default:label_row5=5'b00000;
            endcase
            4'd9: case(row) // M
                0:label_row5=5'b10001; 1:label_row5=5'b11011; 2:label_row5=5'b10101;
                3:label_row5=5'b10101; 4:label_row5=5'b10001; 5:label_row5=5'b10001;
                6:label_row5=5'b10001; default:label_row5=5'b00000;
            endcase
            4'd10: case(row) // S
                0:label_row5=5'b01111; 1:label_row5=5'b10000; 2:label_row5=5'b10000;
                3:label_row5=5'b01110; 4:label_row5=5'b00001; 5:label_row5=5'b00001;
                6:label_row5=5'b11110; default:label_row5=5'b00000;
            endcase
            4'd11: case(row) // K
                0:label_row5=5'b10001; 1:label_row5=5'b10010; 2:label_row5=5'b10100;
                3:label_row5=5'b11000; 4:label_row5=5'b10100; 5:label_row5=5'b10010;
                6:label_row5=5'b10001; default:label_row5=5'b00000;
            endcase
            4'd13: case(row) // T
                0:label_row5=5'b11111; 1:label_row5=5'b00100; 2:label_row5=5'b00100;
                3:label_row5=5'b00100; 4:label_row5=5'b00100; 5:label_row5=5'b00100;
                6:label_row5=5'b00100; default:label_row5=5'b00000;
            endcase
            4'd14: case(row) // P
                0:label_row5=5'b11110; 1:label_row5=5'b10001; 2:label_row5=5'b10001;
                3:label_row5=5'b11110; 4:label_row5=5'b10000; 5:label_row5=5'b10000;
                6:label_row5=5'b10000; default:label_row5=5'b00000;
            endcase
            5'd15: case(row) // B
                0:label_row5=5'b11110; 1:label_row5=5'b10001; 2:label_row5=5'b10001;
                3:label_row5=5'b11110; 4:label_row5=5'b10001; 5:label_row5=5'b10001;
                6:label_row5=5'b11110; default:label_row5=5'b00000;
            endcase
            5'd16: case(row) // H
                0:label_row5=5'b10001; 1:label_row5=5'b10001; 2:label_row5=5'b10001;
                3:label_row5=5'b11111; 4:label_row5=5'b10001; 5:label_row5=5'b10001;
                6:label_row5=5'b10001; default:label_row5=5'b00000;
            endcase
            5'd17: case(row) // Y
                0:label_row5=5'b10001; 1:label_row5=5'b10001; 2:label_row5=5'b01010;
                3:label_row5=5'b00100; 4:label_row5=5'b00100; 5:label_row5=5'b00100;
                6:label_row5=5'b00100; default:label_row5=5'b00000;
            endcase
            default: label_row5=5'b00000;
            endcase
        end
    endfunction

    function [7:0] label_font;
        input [4:0] ch;
        input [3:0] row;
        reg [4:0] row_bits;
        begin
            row_bits = label_row5(ch, row[3:1]);
            label_font = {1'b0, row_bits, 2'b00};
        end
    endfunction

    // ---- 标签字模解码: 四分屏和全屏均使用原生小字号 ----
    wire [10:0] label_x = x_cnt - cur_x0;
    wire [9:0]  label_y = y_cnt - cur_y0;
    reg [4:0]  w_ch;
    reg [3:0]  w_row;
    reg [2:0]  w_cx;
    reg        w_v;
    reg [10:0] xl;
    always @* begin
        w_ch = 5'd0; w_row = 4'd0; w_cx = 3'd0; w_v = 1'b0;
        if(cur_en && (y_cnt >= cur_y0) && (y_cnt < cur_y0 + 10'd16)) begin
            w_row = label_y[3:0];
            xl = label_x;
            if      (xl < 11'd10) w_ch = lbl_char(cur_s, 3'd0, (view_mode == 3'd1));
            else if (xl < 11'd20) begin w_ch = lbl_char(cur_s, 3'd1, (view_mode == 3'd1)); xl = xl - 11'd10; end
            else if (xl < 11'd30) begin w_ch = lbl_char(cur_s, 3'd2, (view_mode == 3'd1)); xl = xl - 11'd20; end
            else if (xl < 11'd40) begin w_ch = lbl_char(cur_s, 3'd3, (view_mode == 3'd1)); xl = xl - 11'd30; end
            else if (xl < 11'd50) begin w_ch = lbl_char(cur_s, 3'd4, (view_mode == 3'd1)); xl = xl - 11'd40; end
            else if (xl < 11'd60) begin w_ch = lbl_char(cur_s, 3'd5, (view_mode == 3'd1)); xl = xl - 11'd50; end
            else if (xl < 11'd70) begin w_ch = lbl_char(cur_s, 3'd6, (view_mode == 3'd1)); xl = xl - 11'd60; end
            else if (xl < 11'd80) begin w_ch = lbl_char(cur_s, 3'd7, (view_mode == 3'd1)); xl = xl - 11'd70; end
            w_cx = xl[2:0];
            w_v = (w_ch != 5'd31) && (xl < 11'd8);
        end
    end
    wire [7:0] w_font = label_font(w_ch, w_row);   // 函数结果先落 wire 再位选
    wire w_glyph = w_v && w_font[7 - w_cx];

    // 小尺寸深色底条与 ALARM 字号协调, 保留足够对比度。
    wire quad_label_bg = ((view_mode == 3'd0) || (view_mode == 3'd1)) &&
        (((y_cnt >= 10'd2) && (y_cnt < 10'd22)) ||
         ((y_cnt >= 10'd362) && (y_cnt < 10'd382))) &&
        (((x_cnt >= 11'd4) && (x_cnt < 11'd90)) ||
         ((x_cnt >= 11'd644) && (x_cnt < 11'd730)));
    wire fullscreen_label_bg = (view_mode >= 3'd2) &&
        (x_cnt >= 11'd1190) && (y_cnt < 10'd20);

    // ---- 分隔线 (QUAD: 2px 白色十字) ----
    wire sep_hit = (x_cnt == 11'd639) || (x_cnt == 11'd640) ||
                   (y_cnt == 10'd359) || (y_cnt == 10'd360);

    wire [9:0] roi_left_disp  = I_roi_left[10:1];
    wire [9:0] roi_right_disp = I_roi_right[10:1];
    wire roi_quad_mask = (view_mode == 3'd0) && (y_cnt >= 10'd361) &&
                         (x_cnt < 11'd639);
    wire roi_quad_target = (view_mode == 3'd0) && (y_cnt >= 10'd361) &&
                           (x_cnt >= 11'd641);
    wire roi_left_line = (roi_quad_mask && (x_cnt == {1'b0, roi_left_disp})) ||
                         (roi_quad_target && (x_cnt == (11'd641 + {1'b0, roi_left_disp}))) ||
                         (((view_mode == 3'd4) || (view_mode == 3'd5)) && (x_cnt == {1'b0, I_roi_left}));
    wire roi_right_line = (roi_quad_mask && (x_cnt == {1'b0, roi_right_disp})) ||
                          (roi_quad_target && (x_cnt == (11'd641 + {1'b0, roi_right_disp}))) ||
                          (((view_mode == 3'd4) || (view_mode == 3'd5)) && (x_cnt == {1'b0, I_roi_right}));

    wire roi_status_quad_mask = (view_mode == 3'd0) &&
        (y_cnt >= 10'd366) && (y_cnt < 10'd374) &&
        (x_cnt >= 11'd92) && (x_cnt < 11'd132);
    wire roi_status_quad_target = (view_mode == 3'd0) &&
        (y_cnt >= 10'd366) && (y_cnt < 10'd374) &&
        (x_cnt >= 11'd732) && (x_cnt < 11'd772);
    wire roi_status_full = ((view_mode == 3'd4) || (view_mode == 3'd5)) &&
        (y_cnt >= 10'd24) && (y_cnt < 10'd32) &&
        (x_cnt >= 11'd1192) && (x_cnt < 11'd1232);
    wire roi_status_region = roi_status_quad_mask || roi_status_quad_target || roi_status_full;
    wire [10:0] roi_status_x = roi_status_quad_target ? (x_cnt - 11'd732) :
                               roi_status_quad_mask ? (x_cnt - 11'd92) :
                               (x_cnt - 11'd1192);
    wire roi_status_hit = roi_status_region &&
        ((roi_status_x < 11'd8) ||
         ((roi_status_x >= 11'd10) && (roi_status_x < 11'd18)) ||
         ((roi_status_x >= 11'd20) && (roi_status_x < 11'd28)) ||
         ((roi_status_x >= 11'd30) && (roi_status_x < 11'd38)));
    wire [1:0] roi_status_slot = (roi_status_x < 11'd10) ? 2'd0 :
                                (roi_status_x < 11'd20) ? 2'd1 :
                                (roi_status_x < 11'd30) ? 2'd2 : 2'd3;
    wire roi_status_on = (roi_status_slot == 2'd0) ? I_sw_raw[3] :
                         (roi_status_slot == 2'd1) ? I_sw_raw[2] :
                         (roi_status_slot == 2'd2) ? I_sw_raw[1] : I_sw_raw[0];
    wire [23:0] roi_status_color = roi_status_on ? 24'hFFFF00 : 24'h404040;

    // ---- SW2 抓拍使能图标: 32x32 红色相机 @ 右下角(1248,688), SW2拨下(往SW2一侧)=显示 ----
    // 每像素2bit编码: 00透明 / 01深色(镜头内芯) / 10红色 / 11高光
    // 位图由 render_snap_icons.py 从已批准渲染图(icon_a_camera_red.png)逐像素生成(4x4子像素阈值)
    function [63:0] icon_row;
        input [4:0] r;
        begin
            case(r)
            5'd00: icon_row = 64'h0000000000000000;
            5'd01: icon_row = 64'h0000000000000000;
            5'd02: icon_row = 64'h0000000000000000;
            5'd03: icon_row = 64'h0000002AA8000000;
            5'd04: icon_row = 64'h000000AAAA000000;
            5'd05: icon_row = 64'h000000AAAA000000;
            5'd06: icon_row = 64'h000000AAAA000000;
            5'd07: icon_row = 64'h000000AAAA000000;
            5'd08: icon_row = 64'h000000AAAA000000;
            5'd09: icon_row = 64'h002AAAAAAAAAA800;
            5'd10: icon_row = 64'h00AAAAAAAAAAAA00;
            5'd11: icon_row = 64'h02AAAAAAAAAAAA80;
            5'd12: icon_row = 64'h02AAAAAAAAAAAA80;
            5'd13: icon_row = 64'h02AAAAA55AAAAA80;
            5'd14: icon_row = 64'h02AAAA5555AAAA80;
            5'd15: icon_row = 64'h02AAA9557F6AAA80;
            5'd16: icon_row = 64'h02AAA5557F6AAA80;
            5'd17: icon_row = 64'h02AAA5557F5AAA80;
            5'd18: icon_row = 64'h02AAA5555D5AAA80;
            5'd19: icon_row = 64'h02AAA555555AAA80;
            5'd20: icon_row = 64'h02AAA555555AAA80;
            5'd21: icon_row = 64'h02AAA555556AAA80;
            5'd22: icon_row = 64'h02AAA955556AAA80;
            5'd23: icon_row = 64'h02AAAA5555AAAA80;
            5'd24: icon_row = 64'h02AAAA9556AAAA80;
            5'd25: icon_row = 64'h02AAAAAAAAAAAA80;
            5'd26: icon_row = 64'h02AAAAAAAAAAAA80;
            5'd27: icon_row = 64'h00AAAAAAAAAAAA00;
            5'd28: icon_row = 64'h002AAAAAAAAAA800;
            5'd29: icon_row = 64'h0000000000000000;
            5'd30: icon_row = 64'h0000000000000000;
            5'd31: icon_row = 64'h0000000000000000;
            default: icon_row = 64'h0000000000000000;
            endcase
        end
    endfunction

    wire        in_icon_x = (x_cnt >= 11'd1248) && (x_cnt < 11'd1280);
    wire        in_icon_y = (y_cnt >= 10'd688)  && (y_cnt < 10'd720);
    wire [4:0]  icon_x    = x_cnt[4:0];             // 1248 % 32 == 0, 低5位即图标列
    wire [4:0]  icon_y    = y_cnt[4:0] - 5'd16;     // 688 % 32 == 16 (5bit模减自动回绕0..31)
    wire [63:0] icon_bits = icon_row(icon_y);       // 函数结果先落 wire 再位选 (V2001)
    wire [1:0]  icon_px   = icon_bits[{icon_x, 1'b0} +: 2];

    reg [7:0] snap_count_d;
    reg [26:0] snap_success_timer;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            snap_count_d <= 8'd0;
            snap_success_timer <= 27'd0;
        end else begin
            snap_count_d <= I_snap_count;
            if(I_snap_count != snap_count_d)
                snap_success_timer <= 27'd37_125_000; // 约0.5秒成功提示
            else if(snap_success_timer != 27'd0)
                snap_success_timer <= snap_success_timer - 27'd1;
        end
    end
    wire snap_success = (snap_success_timer != 27'd0);

    // ---- 安路 logo 水印 (最底层, 65% 不透明度) ----
    // addr = y*160 + x
    wire [14:0] S_logo_addr = {y_cnt[7:0], 7'd0} + {y_cnt[7:0], 5'd0} + {7'd0, x_cnt[7:0]};
    reg  [14:0] S_logo_addr_d1;
    wire [24:0] S_logo_pixel_rom;
    anlogic_logo_rom u_anlogic_logo_rom(
        .I_addr  ( S_logo_addr_d1 ),
        .O_pixel ( S_logo_pixel_rom )
    );
    reg  [24:0] S_logo_pixel;
    reg         S_logo_xy_guard_d1;
    reg         S_logo_xy_guard_d2;
    reg  [23:0] S_mux_data_d1;
    reg  [23:0] S_mux_data_d2;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_logo_addr_d1     <= 15'd0;
            S_logo_pixel       <= 25'd0;
            S_logo_xy_guard_d1 <= 1'b0;
            S_logo_xy_guard_d2 <= 1'b0;
            S_mux_data_d1      <= 24'd0;
            S_mux_data_d2      <= 24'd0;
        end else begin
            S_logo_addr_d1     <= S_logo_addr;
            S_logo_pixel       <= S_logo_pixel_rom;
            S_logo_xy_guard_d1 <= (x_cnt < 12'd160) && (y_cnt < 12'd160);
            S_logo_xy_guard_d2 <= S_logo_xy_guard_d1;
            S_mux_data_d1      <= S_mux_data;
            S_mux_data_d2      <= S_mux_data_d1;
        end
    end
    // 混合: (视频*11 + logo*21 + 16) >> 5
    wire [16:0] S_logo_blend_r = ({S_mux_data_d2[23:16], 3'd0}) +        // v*8
                                 ({S_mux_data_d2[23:16], 1'd0}) +        // v*2
                                 S_mux_data_d2[23:16] +                  // v*1
                                 ({S_logo_pixel[23:16], 4'd0}) +        // l*16
                                 ({S_logo_pixel[23:16], 2'd0}) +        // l*4
                                 S_logo_pixel[23:16] + 13'd16;          // l*1 + 16
    wire [16:0] S_logo_blend_g = ({S_mux_data_d2[15:8], 3'd0}) +
                                 ({S_mux_data_d2[15:8], 1'd0}) +
                                 S_mux_data_d2[15:8] +
                                 ({S_logo_pixel[15:8], 4'd0}) +
                                 ({S_logo_pixel[15:8], 2'd0}) +
                                 S_logo_pixel[15:8] + 13'd16;
    wire [16:0] S_logo_blend_b = ({S_mux_data_d2[7:0], 3'd0}) +
                                 ({S_mux_data_d2[7:0], 1'd0}) +
                                 S_mux_data_d2[7:0] +
                                 ({S_logo_pixel[7:0], 4'd0}) +
                                 ({S_logo_pixel[7:0], 2'd0}) +
                                 S_logo_pixel[7:0] + 13'd16;
    wire [23:0] S_logo_blend_rgb = {S_logo_blend_r[12:5], S_logo_blend_g[12:5], S_logo_blend_b[12:5]};
    wire logo_hit = S_logo_xy_guard_d2 && S_logo_pixel[24];

    // ---- 输出寄存器 (mux 结果 + 覆盖层; 与同步信号同拍) ----
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_de <= 1'b0; O_hsync <= 1'b0; O_vsync <= 1'b0; O_data <= 24'd0;
        end else begin
            O_de    <= S_mux_de;
            O_hsync <= S_mux_hs;
            O_vsync <= S_mux_vs;
            if(de_i && roi_left_line)
                O_data <= 24'hFFFF00;             // 左边界: 黄色
            else if(de_i && roi_right_line)
                O_data <= 24'h00FFFF;             // 右边界: 青色
            else if(de_i && roi_status_hit)
                O_data <= roi_status_color;        // SW1..SW4 状态, 高电平=黄
            else if(de_i && w_glyph)
                O_data <= 24'hFFFFFF;             // 白字
            else if(de_i && (quad_label_bg || fullscreen_label_bg))
                O_data <= 24'h101820;             // 高对比深色标签底条
            else if(de_i && ((view_mode == 3'd0) || (view_mode == 3'd1)) && sep_hit)
                O_data <= 24'hFFFFFF;             // 分隔线
            else if(de_i && (view_mode != 3'd1) && (I_sw2 || snap_success) && in_icon_x && in_icon_y && (icon_px == 2'd2))
                O_data <= snap_success ? 24'h00E040 : 24'hEB3C3C; // 保存成功: 绿色
            else if(de_i && (view_mode != 3'd1) && (I_sw2 || snap_success) && in_icon_x && in_icon_y && (icon_px == 2'd3))
                O_data <= snap_success ? 24'hD8FFE4 : 24'hF5F5F5; // 保存成功高亮
            else if(de_i && (view_mode != 3'd1) && (I_sw2 || snap_success) && in_icon_x && in_icon_y && (icon_px == 2'd1))
                O_data <= 24'h1C222C;             // SW2相机图标: 镜头深色内芯
            else if(de_i && logo_hit)
                O_data <= S_logo_blend_rgb;       // 安路 logo 65% 真色水印 (最底层)
            else
                O_data <= S_mux_data;
        end
    end

    // ---- 模式输出 (wrapper: vo基址/OSD坐标切换) ----
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_view_mode <= 3'd0;
            O_quad_mode <= 1'b1;
            O_process_quad <= 1'b0;
        end else begin
            O_view_mode <= view_mode;
            O_quad_mode <= (view_mode == 3'd0) || (view_mode == 3'd1);
            O_process_quad <= (view_mode == 3'd1);
        end
    end

endmodule
