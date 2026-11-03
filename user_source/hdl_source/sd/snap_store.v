module snap_store #(
    parameter [9:0]  THUMB_W      = 10'd256,   // 缩略图宽 (1280/5)
    parameter [9:0]  THUMB_H      = 10'd120,   // 缩略图高 (720/6)
    parameter [31:0] LOG_BASE     = 32'd8,     // 日志扇区起始 (避开低区)
    parameter [31:0] RING_SECTORS = 32'd1936   // 环形区扇区数 (16次告警 x 121)
)(
    input  wire         I_clk,          // S_csi_rx_clk
    input  wire         I_rst_n,
    // ---- 视频流 (ISP 输出, 4像素RGB888/96bit; 像素0=[95:72] 与 isp 一致) ----
    input  wire         I_tuser,        // 帧开始
    input  wire         I_tvalid,       // 像素有效 (每 beat 4 像素)
    input  wire [95:0]  I_tdata,        // 4像素 RGB888
    // ---- 告警触发 (HDMI域脉冲, 内部两级同步) ----
    input  wire         I_snap_trig,       // 自动告警触发
    input  wire         I_manual_snap_trig,// K2长按触发 (HDMI域展宽脉冲)
    input  wire [127:0] I_manual_fifo_data,// 显示画面采样RGB565, 每拍8像素
    input  wire         I_manual_fifo_valid,
    input  wire         I_manual_fifo_empty,
    output wire         O_manual_fifo_rd,
    output wire         O_busy,
    // ---- 检测状态 (日志内容, csi域帧级信号) ----
    input  wire         I_fire_valid,
    input  wire         I_cig_valid,
    input  wire [19:0]  I_fire_area,
    input  wire [19:0]  I_cig_count,
    input  wire [10:0]  I_fxmin, I_fxmax,
    input  wire [9:0]   I_fymin, I_fymax,
    input  wire [10:0]  I_cxmin, I_cxmax,
    input  wire [9:0]   I_cymin, I_cymax,
    // ---- SD 接口 (到 sd_card_top) ----
    output wire         O_sec_write,
    output wire [31:0]  O_sec_addr,
    output reg  [7:0]   O_sec_data,
    output reg  [7:0]   O_snap_count,   // 快照完成计数 (调试: 每次 S_DONE +1)
    input  wire         I_data_req,
    input  wire         I_write_end
);

    // ---- 派生参数 ----
    localparam [16:0] THUMB_PIXELS      = THUMB_W * THUMB_H;      // 256*120 = 30720
    localparam [16:0] THUMB_BYTES       = THUMB_PIXELS * 2;       // RGB565 = 2B/px -> 61440
    localparam [7:0]  THUMB_SECTORS     = THUMB_BYTES >> 9;       // 61440/512 = 120 (整除)
    localparam [7:0]  SECTORS_PER_ALARM = THUMB_SECTORS + 1;      // 1日志 + 120缩略图 = 121
    localparam [8:0]  BEAT_MAX          = 9'd319;                 // 每行拍数-1 (1280/4-1)

    // ================= 告警触发 CDC =================
    reg snap_s1, snap_s2, snap_s2d;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            snap_s1 <= 1'b0; snap_s2 <= 1'b0; snap_s2d <= 1'b0;
        end else begin
            snap_s1  <= I_snap_trig;
            snap_s2  <= snap_s1;
            snap_s2d <= snap_s2;
        end
    end
    wire snap_rise = snap_s2 && !snap_s2d;

    // 手动显示抓拍触发跨到CSI/SD时钟域。
    reg manual_s1, manual_s2, manual_s2d;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            manual_s1 <= 1'b0; manual_s2 <= 1'b0; manual_s2d <= 1'b0;
        end else begin
            manual_s1  <= I_manual_snap_trig;
            manual_s2  <= manual_s1;
            manual_s2d <= manual_s2;
        end
    end
    wire manual_snap_rise = manual_s2 && !manual_s2d;

    // ================= csi 域帧计数 (日志时间戳) =================
    reg [31:0] frame_cnt;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) frame_cnt <= 32'd0;
        else if(I_tuser) frame_cnt <= frame_cnt + 32'd1;
    end

    // ================= 触发信息锁存 (日志) =================
    reg [31:0] log_frame;
    reg [19:0] log_fire_area, log_cig_count;
    reg [10:0] log_fxmin, log_fxmax, log_cxmin, log_cxmax;
    reg [9:0]  log_fymin, log_fymax, log_cymin, log_cymax;
    reg        log_fire_v, log_cig_v;

    // ================= 主状态机 =================
    localparam S_IDLE       = 4'd0;
    localparam S_WAIT_FRAME = 4'd1;
    localparam S_CAPTURE    = 4'd2;
    localparam S_WR_LOG     = 4'd3;
    localparam S_WR_GAP     = 4'd4;
    localparam S_WR_THUMB   = 4'd5;
    localparam S_DONE       = 4'd6;
    localparam S_CAPTURE_MANUAL = 4'd7;

    reg [3:0]  state;
    reg [8:0]  beat_cnt;      // 行内拍计数 0..319
    reg [9:0]  row_cnt;       // 行计数 0..719
    reg [2:0]  beat_phase;    // 列采样相位 0..4 (÷5, <4 时采该拍的第 beat_phase 个像素)
    reg [2:0]  row_phase;     // 行采样相位 0..5 (÷6, ==0 时采该行)
    reg [15:0] thumb_cnt;     // 已采像素数 0..30719
    reg [14:0] thumb_waddr;   // 采集写地址 0..30719
    reg [6:0]  sec_idx;       // 缩略图扇区索引 0..119
    reg [9:0]  byte_idx;      // 扇区写字节索引 0..511
    reg [31:0] ring_ptr;
    reg        busy;
    reg [127:0] manual_word;
    reg [2:0]   manual_lane;
    reg         manual_word_pending;
    reg         manual_rd_pending;
    wire [15:0] manual_lane_pixel = manual_pixel(manual_word, manual_lane);

    assign O_busy = busy;
    assign O_manual_fifo_rd = (state == S_CAPTURE_MANUAL) &&
                              !manual_word_pending && !manual_rd_pending &&
                              !I_manual_fifo_empty;

    // ---- 采样判定 ----
    wire samp         = (beat_phase < 3'd4) && (row_phase == 3'd0);
    wire capture_done = (thumb_cnt >= THUMB_PIXELS);

    // ---- 像素选取 (按列采样相位) + RGB565 打包 ----
    wire [23:0] pixel0 = I_tdata[95:72];
    wire [23:0] pixel1 = I_tdata[71:48];
    wire [23:0] pixel2 = I_tdata[47:24];
    wire [23:0] pixel3 = I_tdata[23:0];
    wire [23:0] sel_pixel = (beat_phase == 3'd0) ? pixel0 :
                            (beat_phase == 3'd1) ? pixel1 :
                            (beat_phase == 3'd2) ? pixel2 : pixel3;
    wire [15:0] rgb565 = {sel_pixel[23:19], sel_pixel[15:10], sel_pixel[7:3]};  // R5 G6 B5

    // ================= 缩略图块 RAM (32768x16, 简单双口) =================
    reg [15:0] thumb_ram [0:32767];

    // 写端口 (采集)
    always @(posedge I_clk) begin
        if(state == S_CAPTURE && I_tvalid && samp && !capture_done)
            thumb_ram[thumb_waddr] <= rgb565;
        else if(state == S_CAPTURE_MANUAL && manual_word_pending)
            thumb_ram[thumb_waddr] <= manual_lane_pixel;
    end

    // 读端口 (SD 写侧, 同步读 1 拍前推; 每 word 拆 2 字节, 低字节在前)
    wire [14:0] ram_rd_addr = {sec_idx, byte_idx[8:1]};   // sec_idx*256 + 字索引
    reg  [15:0] thumb_word;
    always @(posedge I_clk)
        thumb_word <= thumb_ram[ram_rd_addr];
    wire [7:0]  thumb_byte = byte_idx[0] ? thumb_word[15:8] : thumb_word[7:0];

    // ---- SD 写控制信号 (组合) ----
    assign O_sec_write = (state == S_WR_LOG) || (state == S_WR_THUMB);
    assign O_sec_addr  = (state == S_WR_LOG)   ? ring_ptr :
                         (state == S_WR_THUMB) ? (ring_ptr + 32'd1 + {25'd0, sec_idx}) : 32'd0;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            state        <= S_IDLE;
            beat_cnt     <= 9'd0;
            row_cnt      <= 10'd0;
            beat_phase   <= 3'd0;
            row_phase    <= 3'd0;
            thumb_cnt    <= 16'd0;
            thumb_waddr  <= 15'd0;
            sec_idx      <= 7'd0;
            byte_idx     <= 10'd0;
            ring_ptr     <= LOG_BASE;
            O_sec_data   <= 8'hFF;
            O_snap_count <= 8'd0;
            busy         <= 1'b0;
            log_frame    <= 32'd0;
            log_fire_area <= 20'd0; log_cig_count <= 20'd0;
            log_fxmin <= 11'd0; log_fxmax <= 11'd0;
            log_fymin <= 10'd0; log_fymax <= 10'd0;
            log_cxmin <= 11'd0; log_cxmax <= 11'd0;
            log_cymin <= 10'd0; log_cymax <= 10'd0;
            log_fire_v <= 1'b0; log_cig_v <= 1'b0;
            manual_word <= 128'd0;
            manual_lane <= 3'd0;
            manual_word_pending <= 1'b0;
            manual_rd_pending <= 1'b0;
        end else begin
            if(state != S_CAPTURE_MANUAL) begin
                manual_rd_pending <= 1'b0;
            end else begin
                if(O_manual_fifo_rd)
                    manual_rd_pending <= 1'b1;
                if(I_manual_fifo_valid)
                    manual_rd_pending <= 1'b0;
            end
            case(state)
                S_IDLE: begin
                    if(manual_snap_rise) begin
                        log_frame      <= frame_cnt;
                        log_fire_area  <= I_fire_area;
                        log_cig_count  <= I_cig_count;

                        log_fxmin <= I_fxmin; log_fxmax <= I_fxmax;
                        log_fymin <= I_fymin; log_fymax <= I_fymax;
                        log_cxmin <= I_cxmin; log_cxmax <= I_cxmax;
                        log_cymin <= I_cymin; log_cymax <= I_cymax;
                        log_fire_v <= I_fire_valid;
                        log_cig_v  <= I_cig_valid;
                        thumb_cnt <= 16'd0;
                        thumb_waddr <= 15'd0;
                        manual_lane <= 3'd0;
                        manual_word_pending <= 1'b0;
                        busy <= 1'b1;
                        state <= S_CAPTURE_MANUAL;
                    end else if(snap_rise) begin
                        log_frame      <= frame_cnt;
                        log_fire_area  <= I_fire_area;
                        log_cig_count  <= I_cig_count;
                        log_fxmin <= I_fxmin; log_fxmax <= I_fxmax;
                        log_fymin <= I_fymin; log_fymax <= I_fymax;
                        log_cxmin <= I_cxmin; log_cxmax <= I_cxmax;
                        log_cymin <= I_cymin; log_cymax <= I_cymax;
                        log_fire_v <= I_fire_valid;
                        log_cig_v  <= I_cig_valid;
                        busy      <= 1'b1;
                        state     <= S_WAIT_FRAME;
                    end
                end
                S_WAIT_FRAME: begin
                    if(I_tuser) begin
                        beat_cnt    <= 9'd0;
                        row_cnt     <= 10'd0;
                        beat_phase  <= 3'd0;
                        row_phase   <= 3'd0;
                        thumb_cnt   <= 16'd0;
                        thumb_waddr <= 15'd0;
                        state       <= S_CAPTURE;
                    end
                end
                S_CAPTURE_MANUAL: begin
                    if(I_manual_fifo_valid && !manual_word_pending) begin
                        manual_word <= I_manual_fifo_data;
                        manual_lane <= 3'd0;
                        manual_word_pending <= 1'b1;
                    end else if(manual_word_pending) begin
                        thumb_waddr <= thumb_waddr + 15'd1;
                        thumb_cnt   <= thumb_cnt + 16'd1;
                        if(manual_lane == 3'd7) begin
                            manual_word_pending <= 1'b0;
                            manual_lane <= 3'd0;
                            if(thumb_cnt == THUMB_PIXELS - 1'b1) begin
                                sec_idx  <= 7'd0;
                                byte_idx <= 10'd0;
                                state    <= S_WR_LOG;
                            end
                        end else begin
                            manual_lane <= manual_lane + 3'd1;
                        end
                    end
                end
                S_CAPTURE: begin
                    if(I_tvalid) begin
                        if(I_tuser) begin
                            beat_cnt   <= 9'd0;
                            row_cnt    <= 10'd0;
                            beat_phase <= 3'd0;
                            row_phase  <= 3'd0;
                        end else begin
                            if(beat_cnt == BEAT_MAX) begin
                                beat_cnt  <= 9'd0;
                                row_cnt   <= (row_cnt == 10'd719) ? 10'd0 : row_cnt + 10'd1;
                                row_phase <= (row_phase == 3'd5) ? 3'd0 : row_phase + 3'd1;
                            end else begin
                                beat_cnt <= beat_cnt + 9'd1;
                            end
                            beat_phase <= (beat_phase == 3'd4) ? 3'd0 : beat_phase + 3'd1;
                        end
                        if(samp && !capture_done) begin
                            thumb_waddr <= thumb_waddr + 15'd1;
                            thumb_cnt   <= thumb_cnt + 16'd1;
                        end
                        if(capture_done) begin
                            sec_idx  <= 7'd0;
                            byte_idx <= 10'd0;
                            state    <= S_WR_LOG;
                        end
                    end
                end
                S_WR_LOG: begin
                    if(I_data_req) begin
                        O_sec_data <= log_byte(byte_idx, log_frame, log_fire_area,
                                               log_cig_count, log_fxmin, log_fxmax,
                                               log_fymin, log_fymax, log_cxmin, log_cxmax,
                                               log_cymin, log_cymax, log_fire_v, log_cig_v);
                        if(byte_idx == 10'd511)
                            byte_idx <= 10'd0;
                        else
                            byte_idx <= byte_idx + 10'd1;
                    end
                    if(I_write_end) begin
                        sec_idx  <= 7'd0;
                        byte_idx <= 10'd0;
                        state    <= S_WR_GAP;
                    end
                end
                S_WR_GAP: begin
                    state <= S_WR_THUMB;
                end
                S_WR_THUMB: begin
                    if(I_data_req) begin
                        O_sec_data <= thumb_byte;
                        if(byte_idx == 10'd511)
                            byte_idx <= 10'd0;
                        else
                            byte_idx <= byte_idx + 10'd1;
                    end
                    if(I_write_end) begin
                        if(sec_idx == THUMB_SECTORS - 1) begin
                            if(ring_ptr + SECTORS_PER_ALARM >= LOG_BASE + RING_SECTORS)
                                ring_ptr <= LOG_BASE;
                            else
                                ring_ptr <= ring_ptr + SECTORS_PER_ALARM;
                            busy  <= 1'b0;
                            state <= S_DONE;
                        end else begin
                            sec_idx  <= sec_idx + 7'd1;
                            byte_idx <= 10'd0;
                            state    <= S_WR_GAP;
                        end
                    end
                end
                S_DONE: begin
                    O_snap_count <= O_snap_count + 8'd1;  // 快照完成计数
                    state <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end

    // ================= 日志扇区字节组合输出 =================
    function [15:0] manual_pixel;
        input [127:0] word_data;
        input [2:0] lane;
        begin
            case(lane)
                3'd0: manual_pixel = word_data[15:0];
                3'd1: manual_pixel = word_data[31:16];
                3'd2: manual_pixel = word_data[47:32];
                3'd3: manual_pixel = word_data[63:48];
                3'd4: manual_pixel = word_data[79:64];
                3'd5: manual_pixel = word_data[95:80];
                3'd6: manual_pixel = word_data[111:96];
                default: manual_pixel = word_data[127:112];
            endcase
        end
    endfunction

    function [7:0] log_byte;
        input [9:0]  idx;
        input [31:0] frm;
        input [19:0] fa, ca;
        input [10:0] fx0, fx1, cx0, cx1;
        input [9:0]  fy0, fy1, cy0, cy1;
        input fv, cv;
        begin
            case(idx)
                10'd0: log_byte = 8'hA5;
                10'd1: log_byte = 8'hA5;
                10'd2: log_byte = 8'hA5;
                10'd3: log_byte = 8'hA5;
                10'd4: log_byte = frm[7:0];
                10'd5: log_byte = frm[15:8];
                10'd6: log_byte = frm[23:16];
                10'd7: log_byte = frm[31:24];
                10'd8:  log_byte = fa[7:0];
                10'd9:  log_byte = fa[15:8];
                10'd10: log_byte = fa[19:16];
                10'd11: log_byte = 8'd0;
                10'd12: log_byte = ca[7:0];
                10'd13: log_byte = ca[15:8];
                10'd14: log_byte = ca[19:16];
                10'd15: log_byte = 8'd0;
                10'd16: log_byte = fx0[7:0];
                10'd17: log_byte = {5'd0, fx0[10:8]};
                10'd18: log_byte = fx1[7:0];
                10'd19: log_byte = {5'd0, fx1[10:8]};
                10'd20: log_byte = fy0[7:0];
                10'd21: log_byte = {6'd0, fy0[9:8]};
                10'd22: log_byte = fy1[7:0];
                10'd23: log_byte = {6'd0, fy1[9:8]};
                10'd24: log_byte = cx0[7:0];
                10'd25: log_byte = {5'd0, cx0[10:8]};
                10'd26: log_byte = cx1[7:0];
                10'd27: log_byte = {5'd0, cx1[10:8]};
                10'd28: log_byte = cy0[7:0];
                10'd29: log_byte = {6'd0, cy0[9:8]};
                10'd30: log_byte = cy1[7:0];
                10'd31: log_byte = {6'd0, cy1[9:8]};
                10'd32: log_byte = {6'd0, cv, fv};   // bit0=fire, bit1=cig
                10'd33: log_byte = 8'd0;
                10'd34: log_byte = frm[7:0];
                10'd35: log_byte = frm[15:8];
                10'd36: log_byte = frm[23:16];
                10'd37: log_byte = frm[31:24];
                default: log_byte = 8'hFF;
            endcase
        end
    endfunction

endmodule
