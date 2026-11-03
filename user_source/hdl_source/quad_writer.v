module quad_writer #(
    parameter [24:0] QUAD_BASE_0 = 25'd28000000,   // 十进制!
    parameter [24:0] QUAD_BASE_1 = 25'd30000000
)(
    input wire         I_rst_n,
    input wire         I_quad_clk,           // 相机域 (74.25M, 与 video_in 的 I_camera_clk 同源)
    input wire         I_quad_valid,
    input wire[127:0]  I_quad_data,
    input wire         I_quad_frame_start,   // 相机帧头 1 拍脉冲 (与 video_in 同源)
    input wire         I_ddr_clk,
    input wire         I_vi_wr_busy,
    input wire         I_vo_rd_busy,
    input wire         I_display_pause,
    input wire         I_ddr_user_ready,
    output reg         O_ddr_user_wr_en,
    output reg[24:0]   O_ddr_user_addr,
    output wire[127:0] O_ddr_user_wr_data,
    output wire        O_wr_busy,
    output reg         O_quad_rp             // 显示侧拼接帧基址选择 (0/1)
);

    localparam [9:0]   WORDS_PER_ROW  = 10'd240;
    localparam [9:0]   ROWS_PER_FRAME = 10'd720;
    localparam [20:0]  ROW_STRIDE     = 21'd1920;   // 240字 x 8地址单位/字

    // ---- 帧头脉冲展宽 + 跨域同步 (照抄 video_in) ----
    reg       S_frame_start_extend;
    reg [3:0] S_extend_cnt;
    reg       S_frame_start_extend_1d, S_frame_start_extend_2d, S_frame_start_extend_3d;
    wire      S_frame_start;
    wire      S_fifo_rst;

    always @(posedge I_quad_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            S_frame_start_extend <= 1'b0;
        else if(I_quad_frame_start)
            S_frame_start_extend <= 1'b1;
        else if(S_extend_cnt == 'd7)
            S_frame_start_extend <= 1'b0;
        else
            S_frame_start_extend <= S_frame_start_extend;
    end
    always @(posedge I_quad_clk) begin
        if(S_frame_start_extend)
            S_extend_cnt <= S_extend_cnt + 'd1;
        else
            S_extend_cnt <= 'd0;
    end
    always @(posedge I_ddr_clk) begin
        S_frame_start_extend_1d <= S_frame_start_extend;
        S_frame_start_extend_2d <= S_frame_start_extend_1d;
        S_frame_start_extend_3d <= S_frame_start_extend_2d;
    end
    assign S_fifo_rst     = S_frame_start_extend_3d;
    assign S_frame_start  = ~S_frame_start_extend_3d & S_frame_start_extend_2d;

    // ---- 写侧 FIFO ----
    wire [8:0] S_fifo_rd_num;
    wire       S_fifo_rd_en;
    w128_d512_fifo U_quad_fifo(
        .rst        ( S_fifo_rst         ),
        .clkw       ( I_quad_clk         ),
        .we         ( I_quad_valid       ),
        .di         ( I_quad_data        ),
        .afull      (),
        .full_flag  (),
        .wrusedw    (),
        .clkr       ( I_ddr_clk          ),
        .re         ( S_fifo_rd_en       ),
        .dout       ( O_ddr_user_wr_data ),
        .rdusedw    ( S_fifo_rd_num      ),
        .valid      (),
        .empty_flag (),
        .aempty     ()
    );

    // ---- burst 状态机 (与 video_in 同构) ----
    reg       S_ddr_wr_valid;
    reg [9:0] S_ddr_wr_cnt;
    always @(posedge I_ddr_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            S_ddr_wr_valid <= 1'b0;
        else if(S_ddr_wr_valid && I_ddr_user_ready && S_ddr_wr_cnt == 'd239)
            S_ddr_wr_valid <= 1'b0;
        else if(S_fifo_rd_num >= 'd240 && (!I_vi_wr_busy) && (!I_vo_rd_busy) &&
                (!S_fifo_rst) && I_ddr_user_ready)
            S_ddr_wr_valid <= 1'b1;
        else
            S_ddr_wr_valid <= S_ddr_wr_valid;
    end
    always @(posedge I_ddr_clk) begin
        if(S_ddr_wr_valid) begin
            if(I_ddr_user_ready)
                S_ddr_wr_cnt <= S_ddr_wr_cnt + 'd1;
            else
                S_ddr_wr_cnt <= S_ddr_wr_cnt;
        end else
            S_ddr_wr_cnt <= 'd0;
    end
    assign S_fifo_rd_en = S_ddr_wr_valid & I_ddr_user_ready;
    assign O_wr_busy    = S_ddr_wr_valid;

    always @(posedge I_ddr_clk)
        O_ddr_user_wr_en <= S_fifo_rd_en;

    // ---- 偶帧头复位行计数 + 相机帧计数 (拼接帧 = 2 相机帧) ----
    // qframe_cnt 与 quad_isp 的 frame_par 同为"相机帧序 mod 2"(各自在
    // 帧头/帧头tuser翻转, 一一对应), 偶帧头 = 拼接帧边界
    reg       S_qframe_cnt;
    reg [9:0] S_row_cnt;
    wire      S_burst_end = S_ddr_wr_valid && I_ddr_user_ready && (S_ddr_wr_cnt == 'd239);
    always @(posedge I_ddr_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_qframe_cnt <= 1'b0;
            S_row_cnt    <= 10'd0;
        end else begin
            if(S_frame_start) begin
                S_qframe_cnt <= ~S_qframe_cnt;
                // 偶帧头(拼接帧边界)复位行计数: 偶数源帧(frame_par寄存器=0,
                // quad_isp frame_eff=0)产出拼接行0..359(Q0/Q1), 须与行0对齐;
                // S_qframe_cnt 与 frame_par 同源同相位翻转, 复位判据取翻转前==1
                // (即本帧翻转后==0 的偶数源帧; 上电首帧错位在下个偶帧头自愈)
                if(S_qframe_cnt == 1'b1)
                    S_row_cnt <= 10'd0;
            end else if(S_burst_end) begin
                S_row_cnt <= (S_row_cnt == (ROWS_PER_FRAME - 10'd1)) ? 10'd0
                                                                     : S_row_cnt + 10'd1;
            end
        end
    end

    // ---- 地址: 触发拍锁存行首址, burst 内按写使能 +8 ----
    wire S_wr_trig = (S_fifo_rd_num >= 'd240) && (!I_vi_wr_busy) &&
                     (!I_vo_rd_busy) && (!S_ddr_wr_valid) && (!S_fifo_rst) &&
                     I_ddr_user_ready;
    always @(posedge I_ddr_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            O_ddr_user_addr <= 25'd0;
        else if(S_wr_trig)
            O_ddr_user_addr <= (S_wp ? QUAD_BASE_1 : QUAD_BASE_0) + S_row_cnt * ROW_STRIDE;
        else if(O_ddr_user_wr_en)
            O_ddr_user_addr <= O_ddr_user_addr + 25'd8;
        else
            O_ddr_user_addr <= O_ddr_user_addr;
    end

    // ---- 帧完成 -> 乒乓翻转 + 读指针发布 (暂停时冻结) ----
    reg       S_wp;
    wire      S_frame_done = S_burst_end && (S_row_cnt == (ROWS_PER_FRAME - 10'd1));
    always @(posedge I_ddr_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_wp       <= 1'b0;
            O_quad_rp  <= 1'b0;
        end else if(S_frame_done && !I_display_pause) begin
            S_wp       <= ~S_wp;
            O_quad_rp  <= S_wp;   // 发布刚完成的缓冲
        end
    end

endmodule
