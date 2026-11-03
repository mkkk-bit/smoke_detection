

module design_top_wrapper (
    input wire        I_sys_clk,
    input wire        I_rst_n,
      
    output wire       O_cam_scl,
    inout  wire       IO_cam_sda,
    output wire       O_cam_24m,
    output wire       O_cam_rst,
      
    inout wire [3:0]  I_button,
    input wire [3:0]  I_sw,       // 乒乓开关 SW1~SW4 (拨上=1); SW2=抓拍使能(拨下/往SW2一侧=使能)

    output wire       O_screen_pwm,
    output wire       O_led_fire,    // 阶段1: 明火告警LED (D10, 高亮=告警)
    output wire       O_led_cig,     // 阶段1: 烟头告警LED (E10, 高亮=告警)
    output wire       O_led_sys,     // 阶段2: 总告警LED (C6, sys_alarm)
    output wire       O_buzzer,      // 蜂鸣器 (C12, 低电平有效, 国标火警双音交替: 1000/800Hz 0.5s换音)
    output wire       O_tmds_ch0_p,
    output wire       O_tmds_ch1_p,
    output wire       O_tmds_ch2_p,
    output wire       O_tmds_clk_p,

    inout wire        IO_rx_clk_pad_n, 
    inout wire        IO_rx_clk_pad_p, 
    inout wire[3:0]   IO_rx_data_pad_n,
    inout wire[3:0]   IO_rx_data_pad_p,

    output wire[12:0] ddr_addr,
    output wire[ 1:0] ddr_ba,
    output wire[ 0:0] ddr_cke,
    output wire[ 0:0] ddr_odt,
    output wire[ 0:0] ddr_cs_n,
    output wire       ddr_ras_n,
    output wire       ddr_cas_n,
    output wire       ddr_we_n,
    output wire[ 0:0] ddr_ck_p,
    output wire[ 0:0] ddr_ck_n,
    inout wire[1:0]   ddr_dm,
    inout wire[15:0]  ddr_dq,
    inout wire[1:0]   ddr_dqs_p,
    inout wire[1:0]   ddr_dqs_n,

    // TF卡 SD (SPI模式): SD_nCS=SD_DAT3(B12), SD_DCLK=SD_CLK(A7), SD_MOSI=SD_CMD(A8), SD_MISO=SD_DAT0(E8)
    output wire       SD_nCS,
    output wire       SD_DCLK,
    output wire       SD_MOSI,
    input  wire       SD_MISO,

    // UART 调试输出 (TX=H1)
    output wire       O_uart_tx
);


    wire        	S_100m_clk;
    wire        	S_24m_clk;
    wire        	S_aux_50m_clk;
	wire        	S_10m_clk;
		
	wire        	S_pll_lock;
    wire        	S_rst_n;
	reg  [17:0] 	S_cam_power_cnt;
	reg         	S_cam_pwdnb;
	reg         	S_cam_i2c_ready;
	(* keep = "true" *) wire [15:0] S_cam_sensor_id;
	(* keep = "true" *) wire        S_cam_sensor_id_ok;
	(* keep = "true" *) wire        S_cam_i2c_error;
	localparam integer CAM_PWDNB_LOW_CYCLES = 23810;  // >= 1 ms at 23.809524 MHz
	localparam integer CAM_I2C_START_CYCLES = 142857; // release + >= 5 ms
	
	wire [15:0] 	S_ae;
    wire [15:0] 	S_ag;
    wire 			S_cam_cfg_done;
    wire 			S_ae_cfg_done;
    wire 			S_ae_req; 
	
	wire        	S_csi_rx_clk;  
    wire        	S_hs_rx_valid;  
    wire[15:0]  	S_hs_rx_data;   
	wire[1:0]   	S_lane_error;
	
	wire        	S_csi_frame_start;       
	wire        	S_csi_frame_end;         
	wire        	S_csi_valid;             
	wire[31:0]  	S_csi_data;  
		
	wire        	S_raw10_frame_start; 
	wire        	S_raw10_frame_end;   
	wire        	S_raw10_valid;       
	wire [39:0] 	S_raw10_data;
		
	wire 			S_axis_tlast;  
	wire 			S_axis_tuser;  
	wire [39:0]		S_axis_tdata;  
	wire 			S_axis_tvalid; 
	
	wire [39:0]		S_raw_tdata ;
	wire			S_raw_tlast ;
	wire 			S_raw_tvalid;
	wire 			S_raw_tuser ;
	
	wire			S_ISP_O_tready;
	wire [127:0]	S_ISP_O_tdata ;
    wire [19:0] S_candidate_count_cam;
    wire [21:0] S_bbox_xmin_cam, S_bbox_xmax_cam;
    wire [39:0] S_area_cam;
    wire [19:0] S_area_th_cam;      // 自适应面积阈值 (isp_top.O_min_area, 仪表盘观察)
    wire [19:0] S_bbox_ymin_cam, S_bbox_ymax_cam;
    wire [1:0]  S_bbox_valid_cam;
    // 烟头暗光亮点检测 (isp_top 输出, 相机域)
    wire [7:0]  S_lum_th_cam;
    wire [10:0] S_cig_xmin_cam, S_cig_xmax_cam;
    wire [9:0]  S_cig_ymin_cam, S_cig_ymax_cam;
    wire        S_cig_valid_cam;
    wire [19:0] S_cig_count_cam;
    wire [7:0]  S_cig_conf_cam;
    wire        S_fire_valid_cam;   // 融合火情判定 (isp_top.O_fire_valid, csi域帧级)
    wire [7:0]  S_confidence_cam;   // 检测置信度 (csi域帧级)
    wire [95:0] S_snap_tdata;       // AWB 流 (TF卡告警快照源, isp_top.O_snap_tdata)
    wire        S_snap_tuser, S_snap_tvalid;
    wire [127:0] S_manual_snap_fifo_dout;
    wire         S_manual_snap_fifo_valid, S_manual_snap_fifo_full;
    wire         S_manual_snap_fifo_empty, S_manual_snap_fifo_we;
    wire         S_manual_snap_fifo_re;
    wire [8:0]   S_manual_snap_fifo_wrusedw, S_manual_snap_fifo_rdusedw;
    wire [127:0] S_manual_snap_fifo_din;
    wire         S_snap_store_busy;
    reg          S_snap_busy_sync1_hdmi, S_snap_busy_sync2_hdmi;
    wire         S_snap_busy_hdmi = S_snap_busy_sync2_hdmi;
    wire         S_key2_short, S_key2_long;
    reg          S_manual_snap_trig_stretch;
    reg  [3:0]   S_manual_snap_stretch_cnt;
	reg  [19:0] S_min_area_config_hdmi;
	reg         S_min_area_toggle_hdmi;
	reg  [19:0] S_min_area_sync1_cam, S_min_area_sync2_cam, S_min_area_cam;
	reg         S_min_area_toggle_sync1_cam, S_min_area_toggle_sync2_cam;
	reg         S_min_area_toggle_sync2_d_cam;
    reg  [10:0] S_roi_left_config_hdmi, S_roi_right_config_hdmi;
    reg         S_roi_toggle_hdmi;
    reg  [10:0] S_roi_left_sync1_cam, S_roi_left_sync2_cam, S_roi_left_cam;
    reg  [10:0] S_roi_right_sync1_cam, S_roi_right_sync2_cam, S_roi_right_cam;
    reg         S_roi_toggle_sync1_cam, S_roi_toggle_sync2_cam, S_roi_toggle_sync2_d_cam;
	wire 			S_ISP_O_tlast ;
	wire 			S_ISP_O_tuser ;
	wire 			S_ISP_O_tvalid;

    wire [127:0]   S_quad_tdata_cam;   // 四分屏拼接帧 (quad_isp 输出, 相机域)
    wire           S_quad_tvalid_cam;
    wire           S_process_quad_hdmi;
    reg            S_process_quad_cam_sync1, S_process_quad_cam_sync2;
    wire           S_qw_ddr_wr_en;     // 拼接帧 DDR 写用户 (quad_writer)
    wire [24:0]    S_qw_ddr_wr_addr;
    wire [127:0]   S_qw_ddr_wr_data;
    wire           S_qw_ddr_wr_ready;
    wire           S_quad_wr_busy;
    wire           S_quad_rp;          // 拼接帧读基址选择 (0=QUAD_BASE_0 / 1=QUAD_BASE_1)
	

    wire        S_ddr_clk;
    wire        S_hdmi_pixel_clk;
    wire        S_hdmi_serial_clk;
    wire        S_hdmi_rst_n;
    wire        S_video_out_rst_n;


    wire        S_vi_128b_frame_start;
    wire        S_vi_128b_valid;      
    wire[127:0] S_vi_128b_data;    
    
    wire        S_video_out_rd_busy;
    wire        S_video_in_wr_busy; 
    wire[1:0]   S_video_out_rp;     

    wire        S_ddr_user_wr_en;       
    wire        S_ddr_user_rd_en;       
    wire[24:0]  S_ddr_user_addr;        
    wire[127:0] S_ddr_user_wr_data;     
    wire        S_ddr_user_ready;       
    wire        S_ddr_user_rd_valid;    
    wire[127:0] S_ddr_user_rd_data;     

    wire        S_vi_ddr_wr_en;   
    wire[24:0]  S_vi_ddr_wr_addr; 
    wire[127:0] S_vi_ddr_wr_data;
    wire        S_vi_ddr_wr_ready;

    wire        S_vo_ddr_rd_en;   
    wire[24:0]  S_vo_ddr_rd_addr; 
    wire        S_vo_ddr_rd_ready;
    wire        S_vo_dbg_fifo_rst;
    wire        S_vo_dbg_ddr_rd_valid;
    wire        S_vo_dbg_fifo_rd_en;
    wire        S_vo_dbg_fifo_empty;

    wire        S_init_calib_complete; 
    wire[24:0]  S_mc_app_addr;          
    wire[2:0]   S_mc_app_cmd;           
    wire        S_mc_app_en;            
    wire[127:0] S_mc_app_wdf_data;      
    wire        S_mc_app_wdf_end;       
    wire[15:0]  S_mc_app_wdf_mask;      
    wire        S_mc_app_wdf_wren;      
    wire[127:0] S_mc_app_rd_data;       
    wire        S_mc_app_rd_data_end;   
    wire        S_mc_app_rd_data_valid; 
    wire        S_mc_app_rdy;           
    wire        S_mc_app_wdf_rdy;       
    wire        S_mc_dbg_fifo_afull;
    wire        S_mc_dbg_app_rdy;
    wire        S_mc_dbg_wdf_rdy;
    wire        S_mc_dbg_cmd_pop;

    wire        S_hdmi_vsync;
    wire        S_hdmi_hsync;
    wire        S_hdmi_de;
    wire        S_hdmi_user;
    wire        S_hdmi_last;
    wire        S_video_out_vsync;
    wire        S_hdmi_window_rd_en;
    wire        S_hdmi_out_vsync;
    wire        S_hdmi_out_hsync;
    wire        S_hdmi_out_de;
    wire[23:0]  S_hdmi_out_data;

    wire[23:0]  S_video_rd_data;
    reg         S_dbg_raw_seen;
    reg         S_dbg_isp_seen;
    reg         S_dbg_ddr_wr_seen;
    reg         S_dbg_ddr_rd_seen;
    reg         S_dbg_cam_cfg_seen;
    reg         S_dbg_hs_seen;
    reg         S_dbg_csi_seen;
    reg         S_dbg_raw10_seen;
    reg         S_dbg_isp_fs_seen;
    reg         S_dbg_isp_valid_seen;
    reg         S_dbg_ddr_req_seen;
    reg         S_dbg_ddr_ready_seen;
    reg [1:0]   S_dbg_raw_seen_sync;
    reg [1:0]   S_dbg_isp_seen_sync;
    reg [1:0]   S_dbg_ddr_wr_seen_sync;
    reg [1:0]   S_dbg_ddr_rd_seen_sync;
    reg [1:0]   S_dbg_cam_cfg_seen_sync;
    reg [1:0]   S_dbg_hs_seen_sync;
    reg [1:0]   S_dbg_csi_seen_sync;
    reg [1:0]   S_dbg_raw10_seen_sync;
    reg [1:0]   S_dbg_isp_fs_seen_sync;
    reg [1:0]   S_dbg_isp_valid_seen_sync;
    reg [1:0]   S_dbg_ddr_req_seen_sync;
    reg [1:0]   S_dbg_ddr_ready_seen_sync;
    reg         S_dbg_video_nonzero_seen;
    reg         S_dbg_rd_req_seen;
    reg         S_dbg_rd_valid_seen;
    reg         S_dbg_rd_nonzero_seen;
    reg [1:0]   S_dbg_rd_req_seen_sync;
    reg [1:0]   S_dbg_rd_valid_seen_sync;
    reg [1:0]   S_dbg_rd_nonzero_seen_sync;
    reg         S_dbg_mc_init_seen;
    reg         S_dbg_mc_app_rdy_seen;
    reg         S_dbg_mc_wdf_rdy_seen;
    reg         S_dbg_mc_cmd_pop_seen;
    reg [1:0]   S_dbg_mc_init_seen_sync;
    reg [1:0]   S_dbg_mc_app_rdy_seen_sync;
    reg [1:0]   S_dbg_mc_wdf_rdy_seen_sync;
    reg [1:0]   S_dbg_mc_cmd_pop_seen_sync;
    wire[3:0]   S_hdmi_debug_status;
    wire        S_lane_error_any;


    assign O_screen_pwm = 1'b1;
    assign S_video_out_vsync = ~S_hdmi_vsync;
	
	
    assign S_rst_n 		= S_pll_lock;
    assign S_hdmi_rst_n = S_pll_lock;

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_raw_seen <= 1'b0;
        else if(S_raw10_frame_start | S_axis_tvalid)
            S_dbg_raw_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_seen <= 1'b0;
        else if(S_ISP_O_tuser | S_ISP_O_tvalid)
            S_dbg_isp_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_wr_seen <= 1'b0;
        else if(S_vi_ddr_wr_en)
            S_dbg_ddr_wr_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_rd_seen <= 1'b0;
        else if(S_ddr_user_rd_valid)
            S_dbg_ddr_rd_seen <= 1'b1;
    end

    always @(posedge S_24m_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_cam_cfg_seen <= 1'b0;
        else if(S_cam_cfg_done)
            S_dbg_cam_cfg_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_hs_seen <= 1'b0;
        else if(S_hs_rx_valid)
            S_dbg_hs_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_csi_seen <= 1'b0;
        else if(S_csi_frame_start | S_csi_valid)
            S_dbg_csi_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_raw10_seen <= 1'b0;
        else if(S_raw10_frame_start | S_raw10_valid)
            S_dbg_raw10_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_fs_seen <= 1'b0;
        else if(S_ISP_O_tuser)
            S_dbg_isp_fs_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_valid_seen <= 1'b0;
        else if(S_ISP_O_tvalid)
            S_dbg_isp_valid_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_req_seen <= 1'b0;
        else if(S_vi_ddr_wr_en)
            S_dbg_ddr_req_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_ready_seen <= 1'b0;
        else if(S_ddr_user_ready)
            S_dbg_ddr_ready_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_req_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_req_seen <= 1'b0;
        else if(S_vo_ddr_rd_en)
            S_dbg_rd_req_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_valid_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_valid_seen <= 1'b0;
        else if(S_ddr_user_rd_valid)
            S_dbg_rd_valid_seen <= 1'b1;
    end

    always @(posedge S_hdmi_pixel_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_nonzero_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_nonzero_seen <= 1'b0;
        else if(S_hdmi_window_rd_en && (S_video_rd_data != 24'd0))
            S_dbg_rd_nonzero_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_init_seen <= 1'b0;
        else if(S_init_calib_complete)
            S_dbg_mc_init_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_app_rdy_seen <= 1'b0;
        else if(S_mc_dbg_app_rdy)
            S_dbg_mc_app_rdy_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_wdf_rdy_seen <= 1'b0;
        else if(S_mc_dbg_wdf_rdy)
            S_dbg_mc_wdf_rdy_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_cmd_pop_seen <= 1'b0;
        else if(S_mc_dbg_cmd_pop)
            S_dbg_mc_cmd_pop_seen <= 1'b1;
    end

    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_dbg_cam_cfg_seen_sync <= 2'b00;
            S_dbg_hs_seen_sync      <= 2'b00;
            S_dbg_csi_seen_sync     <= 2'b00;
            S_dbg_raw10_seen_sync   <= 2'b00;
            S_dbg_raw_seen_sync    <= 2'b00;
            S_dbg_isp_seen_sync    <= 2'b00;
            S_dbg_ddr_wr_seen_sync <= 2'b00;
            S_dbg_ddr_rd_seen_sync <= 2'b00;
            S_dbg_isp_fs_seen_sync  <= 2'b00;
            S_dbg_isp_valid_seen_sync <= 2'b00;
            S_dbg_ddr_req_seen_sync <= 2'b00;
            S_dbg_ddr_ready_seen_sync <= 2'b00;
            S_dbg_video_nonzero_seen <= 1'b0;
            S_dbg_rd_req_seen_sync <= 2'b00;
            S_dbg_rd_valid_seen_sync <= 2'b00;
            S_dbg_rd_nonzero_seen_sync <= 2'b00;
            S_dbg_mc_init_seen_sync <= 2'b00;
            S_dbg_mc_app_rdy_seen_sync <= 2'b00;
            S_dbg_mc_wdf_rdy_seen_sync <= 2'b00;
            S_dbg_mc_cmd_pop_seen_sync <= 2'b00;
        end
        else begin
            S_dbg_cam_cfg_seen_sync <= {S_dbg_cam_cfg_seen_sync[0], S_dbg_cam_cfg_seen};
            S_dbg_hs_seen_sync      <= {S_dbg_hs_seen_sync[0],      S_dbg_hs_seen};
            S_dbg_csi_seen_sync     <= {S_dbg_csi_seen_sync[0],     S_dbg_csi_seen};
            S_dbg_raw10_seen_sync   <= {S_dbg_raw10_seen_sync[0],   S_dbg_raw10_seen};
            S_dbg_raw_seen_sync    <= {S_dbg_raw_seen_sync[0],    S_dbg_raw_seen};
            S_dbg_isp_seen_sync    <= {S_dbg_isp_seen_sync[0],    S_dbg_isp_seen};
            S_dbg_ddr_wr_seen_sync <= {S_dbg_ddr_wr_seen_sync[0], S_dbg_ddr_wr_seen};
            S_dbg_ddr_rd_seen_sync <= {S_dbg_ddr_rd_seen_sync[0], S_dbg_ddr_rd_seen};
            S_dbg_isp_fs_seen_sync  <= {S_dbg_isp_fs_seen_sync[0],  S_dbg_isp_fs_seen};
            S_dbg_isp_valid_seen_sync <= {S_dbg_isp_valid_seen_sync[0], S_dbg_isp_valid_seen};
            S_dbg_ddr_req_seen_sync <= {S_dbg_ddr_req_seen_sync[0], S_dbg_ddr_req_seen};
            S_dbg_ddr_ready_seen_sync <= {S_dbg_ddr_ready_seen_sync[0], S_dbg_ddr_ready_seen};
            S_dbg_rd_req_seen_sync <= {S_dbg_rd_req_seen_sync[0], S_dbg_rd_req_seen};
            S_dbg_rd_valid_seen_sync <= {S_dbg_rd_valid_seen_sync[0], S_dbg_rd_valid_seen};
            S_dbg_rd_nonzero_seen_sync <= {S_dbg_rd_nonzero_seen_sync[0], S_dbg_rd_nonzero_seen};
            S_dbg_mc_init_seen_sync <= {S_dbg_mc_init_seen_sync[0], S_dbg_mc_init_seen};
            S_dbg_mc_app_rdy_seen_sync <= {S_dbg_mc_app_rdy_seen_sync[0], S_dbg_mc_app_rdy_seen};
            S_dbg_mc_wdf_rdy_seen_sync <= {S_dbg_mc_wdf_rdy_seen_sync[0], S_dbg_mc_wdf_rdy_seen};
            S_dbg_mc_cmd_pop_seen_sync <= {S_dbg_mc_cmd_pop_seen_sync[0], S_dbg_mc_cmd_pop_seen};
            if(S_hdmi_window_rd_en && (S_video_rd_data != 24'd0))
                S_dbg_video_nonzero_seen <= 1'b1;
        end
    end

    assign S_lane_error_any = |S_lane_error;

    assign S_vo_dbg_fifo_rst = 1'b0;
    assign S_vo_dbg_ddr_rd_valid = 1'b0;
    assign S_vo_dbg_fifo_rd_en = 1'b0;
    assign S_vo_dbg_fifo_empty = 1'b0;
    assign S_mc_dbg_fifo_afull = 1'b0;
    assign S_mc_dbg_app_rdy = 1'b0;
    assign S_mc_dbg_wdf_rdy = 1'b0;
    assign S_mc_dbg_cmd_pop = 1'b0;

    assign S_hdmi_debug_status = {
        (|S_ddr_user_rd_data),
        S_ddr_user_rd_valid,
        S_vo_ddr_rd_en,
        S_vi_ddr_wr_en
    };
	
	// SC520CS上电后先保持PWDNB/XSHUTDN为低，再释放并等待内部上电完成，
	// 避免PLL刚锁定就开始I2C初始化而造成寄存器偶发写入失败。
	always @(posedge S_24m_clk or negedge S_pll_lock) begin
		if (!S_pll_lock) begin
			S_cam_power_cnt <= 18'd0;
			S_cam_pwdnb <= 1'b0;
			S_cam_i2c_ready <= 1'b0;
		end else begin
			if (S_cam_power_cnt < CAM_I2C_START_CYCLES)
				S_cam_power_cnt <= S_cam_power_cnt + 1'b1;
			if (S_cam_power_cnt >= CAM_PWDNB_LOW_CYCLES)
				S_cam_pwdnb <= 1'b1;
			if (S_cam_power_cnt >= CAM_I2C_START_CYCLES - 1)
				S_cam_i2c_ready <= 1'b1;
		end
	end

	assign O_cam_rst 	= S_cam_pwdnb;
    assign O_cam_24m 	= S_24m_clk;
	
	
	
	
    PLL u_PLL(
        .refclk      ( I_sys_clk         ),
        .reset    	 ( ~I_rst_n          ),
        
        .clk0_out    ( S_100m_clk        ),
        .clk1_out    ( S_24m_clk         ),

        .clk4_out    ( S_hdmi_pixel_clk  ),
        .clk5_out    ( S_hdmi_serial_clk ),

        .lock        ( S_pll_lock        )

    );


	

  
  ae_set u_ae_set (
      .I_clk(S_24m_clk),
      .I_rst(~S_rst_n),
      .I_btn(4'b1111),   // KEY2已让给画面暂停, 手动曝光加减键停用
      .I_cam_cfg_done(S_cam_cfg_done),
      .I_ae_cfg_done(S_ae_cfg_done),
      .O_ae_req(S_ae_req),
      .O_ae(S_ae),
      .O_ag(S_ag)
  );
  
    // SC520CS原厂真实HD模式：1280x720@60、RAW10、2-Lane、828 Mbps/Lane。
    // 当前寄存器表已关闭Slave/FSIN等待，Sensor以free-run方式连续输出。
    uicfgcs520 #(
      .CLK_DIV(23809524 / 100000 - 1)
  ) u_uicfgcs520 (
      .I_clk(S_24m_clk),  //系统时钟输入
      .I_rst_n(S_cam_i2c_ready), // SC520CS power-up delay completed
      .I_ae_req(S_ae_req),
      .I_ae(S_ae),
      .I_ag(S_ag),
      .O_cam_scl(O_cam_scl),  //I2C总线，SCL时钟
      .IO_cam_sda(IO_cam_sda),  //I2C总线，SDA数据
      .O_cfg_done(S_cam_cfg_done),  //摄像头寄存器初始化完成
      .O_ae_cfg_done(S_ae_cfg_done),  //AE配置完成
      .O_sensor_id(S_cam_sensor_id),
      .O_sensor_id_ok(S_cam_sensor_id_ok),
      .O_iic_error(S_cam_i2c_error)
  );
	
    mipi_dphy_rx_ph1p_mipiio_wrapper#(
        .DPHY_RX_LOCATION      ( "DPHY0" ),
        .HS_EQUALIZER          ( "3dB"   ),// 828 Mbps/Lane接收均衡
        .HS_VGA_GAIN           ( "8dB" ),//"-3dB","-1.5dB","0dB","1.5dB","3dB","4.5dB","6dB","7.5dB"
        .LANE_NUM              ( 2       ),
        .BYTE_NUM              ( 1       )
    )u_mipi_dphy_rx_ph1p_mipiio_wrapper(
        .I_lp_clk              ( S_100m_clk       ),
        .I_rst                 ( ~S_rst_n         ),

        .I_clk_lane_in_delay   ( 6'd0             ),
        .I_data_lane0_in_delay ( 6'd0             ),
        .I_data_lane1_in_delay ( 6'd0             ),
        .I_data_lane2_in_delay ( 6'd0             ),
        .I_data_lane3_in_delay ( 6'd0             ),

        .I_lane_invert         ( 4'b0000          ),
     
        .O_hs_rx_clk           ( S_csi_rx_clk     ),
        .O_hs_rx_valid         ( S_hs_rx_valid    ),
        .O_hs_rx_data          ( S_hs_rx_data     ),
      
        .O_lp_rx_lane0_p       (  ),
        .O_lp_rx_lane0_n       (  ),
      
        .I_lp_tx_en            ( 1'b0             ),
        .I_lp_tx_lane0_p       ( 1'b1             ),
        .I_lp_tx_lane0_n       ( 1'b1             ),
      
        .O_lane_match_error    (                  ),
        .O_lane_error          ( S_lane_error     ),
      
        .IO_rx_clk_pad_n       ( IO_rx_clk_pad_n  ),
        .IO_rx_clk_pad_p       ( IO_rx_clk_pad_p  ),
        .IO_rx_data_pad_n      ( IO_rx_data_pad_n ),
        .IO_rx_data_pad_p      ( IO_rx_data_pad_p )
    );
	
	
 //  cwc cwc_inst
 //(
 //    .probe0(S_hs_rx_valid),
 //    .probe1(S_hs_rx_data),
 //    .probe2(S_lane_error),
 //    .clk(S_csi_rx_clk)
 //);

 //csi 解码为RAW数据
csi_unpacket_2lane u_csi_unpacket(
.I_clk                 ( S_csi_rx_clk       ),
.I_rst_n               ( S_rst_n        	),
.I_hs_valid            ( S_hs_rx_valid     ),
.I_hs_data             ( S_hs_rx_data      ),

.O_csi_frame_start     ( S_csi_frame_start ),
.O_csi_frame_end       ( S_csi_frame_end   ),
.O_csi_valid           ( S_csi_valid       ),
.O_csi_data            ( S_csi_data        )
);


//解码为RAW8
raw10_unpacket_2lane u_raw10_unpacket (
.I_clk  (S_csi_rx_clk),
.I_rst_n(S_rst_n),

.I_csi_frame_start(S_csi_frame_start),
.I_csi_frame_end  (S_csi_frame_end),
.I_csi_valid      (S_csi_valid),
.I_csi_data       (S_csi_data),

.O_raw10_frame_start(S_raw10_frame_start),
.O_raw10_frame_end  (S_raw10_frame_end),
.O_raw10_valid      (S_raw10_valid),
.O_raw10_data       (S_raw10_data)
  );


//将数据转为stream流
uial2axis #(
.IMG_WIDTH(1280),
.IMG_HEIGHT(720),
.INPUT_DATA_WIDTH(40)
) 
u_uial2axis (
.I_native_clk(S_csi_rx_clk),
.I_rst_n     (S_rst_n),
.I_data      (S_raw10_data       ),
.I_data_valid(S_raw10_valid      ),
.I_data_start(S_raw10_frame_start),
.I_data_end  (S_raw10_frame_end  ),
.axis_tvalid (S_axis_tvalid),
.axis_tdata  (S_axis_tdata ),
.axis_tuser  (S_axis_tuser ),
.axis_tlast  (S_axis_tlast )
);
	

// 原厂HD模式直接输出1280x720 RAW10，不再进行FPGA固定窗口裁剪。
assign S_raw_tdata  = S_axis_tdata;
assign S_raw_tlast  = S_axis_tlast;
assign S_raw_tvalid = S_axis_tvalid;
assign S_raw_tuser  = S_axis_tuser;


//cwc1 cwc1_Inst
//  (
//      .probe0(S_axis_tuser),
//      .probe1(S_axis_tvalid),
//      .probe2(S_axis_tlast),
//      .probe3(S_axis_tdata),
//      .probe4(S_ISP_O_tuser),
//      .probe5(S_ISP_O_tvalid),
//      .probe6(S_ISP_O_tlast),
//      .probe7(S_ISP_O_tdata),
//      .probe8 (S_raw_tdata ),
//      .probe9 (S_raw_tlast ),
//      .probe10(S_raw_tvalid),
//      .probe11(S_raw_tuser ),
//      .clk(S_csi_rx_clk)
//  );


// ISP 下游恒可就绪: video_in(摄像头侧, 与 isp_top 同 S_csi_rx_clk 域)无 ready 输出,
// 内部 512 深 FIFO + DDR 写带宽远大于摄像头速率, 持续可吸收数据。
// 原 S_ISP_O_tready 悬空, 综合器拉 0 (SYN-5013) 会停摆整个 ISP 流水线。
assign S_ISP_O_tready = 1'b1;

//ISP算法顶层模块
isp_top u_isp_top (
.axi4s_video_aclk(S_csi_rx_clk),
.I_rst_n         (S_rst_n),
.I_tlast         (S_raw_tlast	),
.I_tuser         (S_raw_tuser	),
.I_tdata         (S_raw_tdata	),
.I_tvalid        (S_raw_tvalid	),
.I_tdest         (),
.O_tready        (S_ISP_O_tready),
.I_min_area      (S_min_area_cam),
.I_lum_th        (S_lum_th_cam    ),  // 烟头亮点阈值 (threshold_calib, csi域)
.I_roi_left      (S_roi_left_cam),
.I_roi_right     (S_roi_right_cam),
.I_quad_page     (S_process_quad_cam_sync2),
.I_edge_key      (1'b1),           // KEY1 已上移至显示域 quad_top(六态循环), ISP 恒出原图
.O_tdata         (S_ISP_O_tdata ),
.O_candidate_count(S_candidate_count_cam),
.O_quad_tdata     (S_quad_tdata_cam ),
.O_quad_tvalid    (S_quad_tvalid_cam),
.O_bbox_xmin     (S_bbox_xmin_cam),
.O_bbox_xmax     (S_bbox_xmax_cam),
.O_bbox_ymin     (S_bbox_ymin_cam),
.O_bbox_ymax     (S_bbox_ymax_cam),
.O_bbox_valid    (S_bbox_valid_cam),
.O_area          (S_area_cam       ),
.O_min_area      (S_area_th_cam    ),
.O_fire_valid    (S_fire_valid_cam ),
.O_confidence    (S_confidence_cam ),
.O_cig_xmin      (S_cig_xmin_cam   ),
.O_cig_xmax      (S_cig_xmax_cam   ),
.O_cig_ymin      (S_cig_ymin_cam   ),
.O_cig_ymax      (S_cig_ymax_cam   ),
.O_cig_valid     (S_cig_valid_cam  ),
.O_cig_count     (S_cig_count_cam  ),
.O_cig_confidence(S_cig_conf_cam   ),
.O_snap_tdata    (S_snap_tdata     ),
.O_snap_tuser    (S_snap_tuser     ),
.O_snap_tvalid   (S_snap_tvalid    ),
.O_tlast         (S_ISP_O_tlast ),
.O_tuser         (S_ISP_O_tuser ),
.O_tvalid        (S_ISP_O_tvalid),
.I_tready        ()
  );

always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
    if(!S_rst_n) begin
        S_process_quad_cam_sync1 <= 1'b0;
        S_process_quad_cam_sync2 <= 1'b0;
    end else begin
        S_process_quad_cam_sync1 <= S_process_quad_hdmi;
        S_process_quad_cam_sync2 <= S_process_quad_cam_sync1;
    end
end

// ---- 烟头亮点阈值标定: 长按 KEY2(板载, I_button[0]) 循环上调 LUM_TH (csi域) ----
threshold_calib u_threshold_calib(
    .I_clk    (S_csi_rx_clk),
    .I_rst_n  (S_rst_n),
    .I_key2   (1'b1),          // K2改为短按暂停/长按手动抓拍, 阈值保持默认150
    .O_lum_th (S_lum_th_cam)
);

always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
    if(!S_rst_n) begin
        S_min_area_sync1_cam <= 20'd64;
        S_min_area_sync2_cam <= 20'd64;
        S_min_area_cam <= 20'd64;
        S_min_area_toggle_sync1_cam <= 1'b0;
        S_min_area_toggle_sync2_cam <= 1'b0;
        S_min_area_toggle_sync2_d_cam <= 1'b0;
    end else begin
        S_min_area_sync1_cam <= S_min_area_config_hdmi;
        S_min_area_sync2_cam <= S_min_area_sync1_cam;
        S_min_area_toggle_sync1_cam <= S_min_area_toggle_hdmi;
        S_min_area_toggle_sync2_cam <= S_min_area_toggle_sync1_cam;
        S_min_area_toggle_sync2_d_cam <= S_min_area_toggle_sync2_cam;
        if(S_min_area_toggle_sync2_cam != S_min_area_toggle_sync2_d_cam)
            S_min_area_cam <= S_min_area_sync2_cam;
    end
end

// ROI 坐标以稳定总线+toggle方式原子同步到相机处理域，避免边界更新时左右坐标撕裂。
always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
    if(!S_rst_n) begin
        S_roi_left_sync1_cam <= 11'd320;
        S_roi_left_sync2_cam <= 11'd320;
        S_roi_left_cam <= 11'd320;
        S_roi_right_sync1_cam <= 11'd960;
        S_roi_right_sync2_cam <= 11'd960;
        S_roi_right_cam <= 11'd960;
        S_roi_toggle_sync1_cam <= 1'b0;
        S_roi_toggle_sync2_cam <= 1'b0;
        S_roi_toggle_sync2_d_cam <= 1'b0;
    end else begin
        S_roi_left_sync1_cam <= S_roi_left_config_hdmi;
        S_roi_left_sync2_cam <= S_roi_left_sync1_cam;
        S_roi_right_sync1_cam <= S_roi_right_config_hdmi;
        S_roi_right_sync2_cam <= S_roi_right_sync1_cam;
        S_roi_toggle_sync1_cam <= S_roi_toggle_hdmi;
        S_roi_toggle_sync2_cam <= S_roi_toggle_sync1_cam;
        S_roi_toggle_sync2_d_cam <= S_roi_toggle_sync2_cam;
        if(S_roi_toggle_sync2_cam != S_roi_toggle_sync2_d_cam) begin
            S_roi_left_cam <= S_roi_left_sync2_cam;
            S_roi_right_cam <= S_roi_right_sync2_cam;
        end
    end
end


reg          S_fire_valid_sync;  // 同步到 hdmi 域
reg  [7:0]   S_confidence_sync;  // 置信度同步到 hdmi 域
wire         S_sec_write;        // snap_store <-> sd_card_top 扇区写握手
wire [31:0]  S_sec_addr;
wire [7:0]   S_sec_data;
wire         S_sec_data_req, S_sec_write_end;
wire [7:0]   S_snap_count;       // 快照完成计数 (调试 -> UART)
reg [7:0] S_snap_count_sync1_hdmi, S_snap_count_sync2_hdmi;
always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
    if(!S_hdmi_rst_n) begin
        S_snap_count_sync1_hdmi <= 8'd0;
        S_snap_count_sync2_hdmi <= 8'd0;
    end else begin
        S_snap_count_sync1_hdmi <= S_snap_count;
        S_snap_count_sync2_hdmi <= S_snap_count_sync1_hdmi;
    end
end
(* keep = "true" *) reg [15:0] S_latency_cam;  // ISP流水线时延 (csi域拍数)
reg [15:0]  S_lat_cnt;
reg S_candidate_toggle_cam;
reg S_candidate_toggle_sync1;
reg S_candidate_toggle_sync2;
reg S_candidate_toggle_sync2_d;
reg [19:0] S_candidate_count_sync;
reg [19:0] S_area_th_sync;
reg [21:0] S_bbox_xmin_sync, S_bbox_xmax_sync;
reg [19:0] S_bbox_ymin_sync, S_bbox_ymax_sync;
reg [1:0]  S_bbox_valid_sync;
reg [21:0] S_bbox_xmin_pause, S_bbox_xmax_pause;
reg [19:0] S_bbox_ymin_pause, S_bbox_ymax_pause;
reg [1:0] S_bbox_valid_pause;
reg S_fire_alarm_pause;
reg [39:0] S_area_sync;
reg [15:0] S_latency_sync;   // ISP段时延 (csi域拍数, 经toggle握手同步到hdmi域)
reg        S_cig_valid_sync;  // 烟头判定 (csi域 -> hdmi域)
reg [10:0] S_cig_xmin_sync, S_cig_xmax_sync;
reg [9:0]  S_cig_ymin_sync, S_cig_ymax_sync;
reg [19:0] S_cig_count_sync;
reg [7:0] S_cig_conf_sync;   // synchronized smoke confidence
wire [7:0] S_conf_overall = (S_confidence_sync > S_cig_conf_sync) ? S_confidence_sync : S_cig_conf_sync;
wire [1:0] S_conf_level = (S_conf_overall >= 8'd80) ? 2'd2 :
                         (S_conf_overall >= 8'd40) ? 2'd1 : 2'd0;
// 烟头阈值 CDC (慢信号, 按键长按时才变, 2级同步足够)
reg [7:0]  S_lum_th_sync1, S_lum_th_sync;
always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
    if(!S_hdmi_rst_n) begin
        S_lum_th_sync1 <= 8'd150;
        S_lum_th_sync  <= 8'd150;
    end else begin
        S_lum_th_sync1 <= S_lum_th_cam;
        S_lum_th_sync  <= S_lum_th_sync1;
    end
end
always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
    if(!S_rst_n)
        S_candidate_toggle_cam <= 1'b0;
    else if(S_ISP_O_tuser)
        S_candidate_toggle_cam <= ~S_candidate_toggle_cam;
end
always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
    if(!S_hdmi_rst_n) begin
        S_candidate_toggle_sync1 <= 1'b0;
        S_candidate_toggle_sync2 <= 1'b0;
        S_candidate_toggle_sync2_d <= 1'b0;
        S_candidate_count_sync <= 20'd0;
        S_bbox_xmin_sync <= 22'd0; S_bbox_xmax_sync <= 22'd0;
        S_bbox_ymin_sync <= 20'd0; S_bbox_ymax_sync <= 20'd0;
        S_bbox_valid_sync <= 2'b0;
        S_area_sync <= 40'd0;
        S_area_th_sync <= 20'd64;
        S_fire_valid_sync <= 1'b0;
        S_confidence_sync <= 8'd0;
        S_latency_sync <= 16'd0;
        S_cig_valid_sync <= 1'b0;
        S_cig_xmin_sync <= 11'd0; S_cig_xmax_sync <= 11'd0;
        S_cig_ymin_sync <= 10'd0; S_cig_ymax_sync <= 10'd0;
        S_cig_count_sync <= 20'd0;
        S_cig_conf_sync <= 8'd0;
    end else begin
        S_candidate_toggle_sync1 <= S_candidate_toggle_cam;
        S_candidate_toggle_sync2 <= S_candidate_toggle_sync1;
        S_candidate_toggle_sync2_d <= S_candidate_toggle_sync2;
        if(S_candidate_toggle_sync2 != S_candidate_toggle_sync2_d) begin
            S_candidate_count_sync <= S_candidate_count_cam;
            S_bbox_xmin_sync <= S_bbox_xmin_cam;
            S_bbox_xmax_sync <= S_bbox_xmax_cam;
            S_bbox_ymin_sync <= S_bbox_ymin_cam;
            S_bbox_ymax_sync <= S_bbox_ymax_cam;
            S_bbox_valid_sync <= S_bbox_valid_cam;
            S_area_sync <= S_area_cam;
            S_area_th_sync <= S_area_th_cam;
            S_fire_valid_sync <= S_fire_valid_cam;
            S_confidence_sync <= S_confidence_cam;
            S_latency_sync <= S_latency_cam;
            S_cig_valid_sync <= S_cig_valid_cam;
            S_cig_xmin_sync <= S_cig_xmin_cam;
            S_cig_xmax_sync <= S_cig_xmax_cam;
            S_cig_ymin_sync <= S_cig_ymin_cam;
            S_cig_ymax_sync <= S_cig_ymax_cam;
            S_cig_count_sync <= S_cig_count_cam;
            S_cig_conf_sync <= S_cig_conf_cam;
        end
    end
end
// 每帧一个脉冲, 作为判定模块的帧结算信号 (与计数锁存同一拍)
wire S_frame_tick = (S_candidate_toggle_sync2 != S_candidate_toggle_sync2_d);

// ---- 帧率量化: 1 秒窗口内计数帧 tick -> 整数 fps (S_hdmi_pixel_clk = 74.25MHz) ----
reg [26:0] S_fps_win_cnt;   // 1秒窗口计数器 (74.25M 拍, 27bit)
reg [6:0]  S_fps_cnt;       // 窗口内帧计数
reg [6:0]  S_fps_reg;       // 锁存的帧率 (fps, 供 UART/OSD 输出)
always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
    if(!S_hdmi_rst_n) begin
        S_fps_win_cnt <= 27'd0;
        S_fps_cnt     <= 7'd0;
        S_fps_reg     <= 7'd0;
    end else begin
        if(S_frame_tick)
            S_fps_cnt <= S_fps_cnt + 7'd1;
        if(S_fps_win_cnt == 27'd74250000 - 27'd1) begin
            S_fps_win_cnt <= 27'd0;
            S_fps_reg     <= S_fps_cnt;
            S_fps_cnt     <= 7'd0;
        end else begin
            S_fps_win_cnt <= S_fps_win_cnt + 27'd1;
        end
    end
end

// ---- 帧级目标有效标志 (连通域面积判定, 阈值默认64并支持按键调节) ----
// fire: 至少一个红色连通域面积 >= MIN_AREA (bbox_valid两位分别对应面积排序的两个目标),
//       有框必有警; (连通域检测取代原"候选像素计数>=2"判定: 绿色像素/散点噪声
//       不形成 >=MIN_AREA 的连通域, 无法触发框选与告警)
// cig: 小簇带判定废弃 (LED E10 恒灭, L2预留)
wire S_fire_valid = S_fire_valid_sync;   // 融合判定 (feature_fuse, 经CDC同步)
wire S_cig_valid  = S_cig_valid_sync;    // 烟头判定 (luma_spot+smoke_bbox, 经CDC同步)

// ---- 快照触发: SW2(拨下/往SW2一侧=使能, 即I_sw[1]=0) + 明火框(bbox) 或 烟头 短防抖(约4帧) + 30帧保持去重 ----
// 触发源 = |S_bbox_valid_sync(红框) || S_cig_valid(烟头), 用户可见即触发。
// SW2 拨上=禁止抓拍: cand恒0 -> 防抖清零、armed 30帧后释放, 在途抓拍自然完成。
reg [1:0]  S_snap_deb_cnt;
reg [4:0]  S_snap_hold_cnt;
reg        S_snap_armed;
reg        S_snap_trig;
wire       S_snap_en   = ~I_sw[1];                                // SW2: 拨下(往SW2一侧)=抓拍使能, 拨上=禁止
wire       S_snap_cand = S_snap_en && (|S_bbox_valid_sync || S_cig_valid);
always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
    if(!S_hdmi_rst_n) begin
        S_snap_deb_cnt  <= 2'd0;
        S_snap_hold_cnt <= 5'd0;
        S_snap_armed    <= 1'b0;
        S_snap_trig     <= 1'b0;
    end else begin
        S_snap_trig <= 1'b0;  // 默认脉冲清零
        // 防抖计数 (候选连续帧)
        if(S_snap_cand)
            S_snap_deb_cnt <= (S_snap_deb_cnt == 2'd3) ? 2'd3 : S_snap_deb_cnt + 2'd1;
        else
            S_snap_deb_cnt <= 2'd0;
        // 保持: 候选消失 30 帧后解除 armed (去重, 防连拍)
        if(!S_snap_cand) begin
            if(S_snap_hold_cnt == 5'd30) S_snap_armed <= 1'b0;
            else S_snap_hold_cnt <= S_snap_hold_cnt + 5'd1;
        end else begin
            S_snap_hold_cnt <= 5'd0;
        end
        // 触发: 候选连续约 4 帧 + 未 armed
        if(S_snap_cand && (S_snap_deb_cnt == 2'd3) && !S_snap_armed) begin
            S_snap_trig <= 1'b1;
            S_snap_armed <= 1'b1;
        end
    end
end
alarm_module #(
    .DEBOUNCE (16'd30),
    .HOLD     (16'd30)
) u_alarm_module(
    .I_clk        (S_hdmi_pixel_clk),
    .I_rst_n      (S_hdmi_rst_n    ),
    .I_vsync      (S_hdmi_vsync    ),
    .I_hsync      (S_hdmi_hsync    ),
    .I_fire_valid (S_fire_valid    ),
    .I_cig_valid  (S_cig_valid     ),
    .O_fire_alarm (O_led_fire      ),
    .O_cig_alarm  (O_led_cig       ),
    .O_sys_alarm  (O_led_sys       ),
    .O_snap_trig  (                )   // 快照触发改用独立短防抖, 不再用 alarm 30帧防抖
);

// ---- detect_judge保留为蜂鸣音型发生器 (国标火警双音交替) ----
// 触发源 = S_fire_valid 目标检测有效 (bbox, 与OSD ALARM同源);
// 内部积分锁定0.5s防抖, LED改由alarm_module驱动
detect_judge u_detect_judge(
    .I_clk         (S_hdmi_pixel_clk      ),
    .I_rst_n       (S_hdmi_rst_n          ),
    .I_frame_tick  (S_frame_tick          ),
    .I_candidate   (S_candidate_count_sync),
    .I_fire_valid  (S_fire_valid          ),
    .I_cig_valid  (S_cig_valid     ),
    .O_alarm_fire  (                      ),
    .O_alarm_cig   (                      ),
    .O_buzzer      (O_buzzer              )
);

// ---- 模块: TF卡告警快照 (snap_store 缩略图+日志 -> sd_card_top 环形裸扇区) ----
// 全旁路: 不插入视频数据通路, SD异常不影响显示/检测/告警主功能。
reg  S_snap_trig_stretch;
reg  [3:0] S_snap_stretch_cnt;
always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
    if(!S_hdmi_rst_n) begin
        S_snap_trig_stretch <= 1'b0;
        S_snap_stretch_cnt  <= 4'd0;
    end else if(S_snap_trig) begin
        S_snap_trig_stretch <= 1'b1;
        S_snap_stretch_cnt  <= 4'd0;
    end else if(S_snap_trig_stretch) begin
        if(S_snap_stretch_cnt == 4'd15) S_snap_trig_stretch <= 1'b0;
        else S_snap_stretch_cnt <= S_snap_stretch_cnt + 1'b1;
    end
end

snap_store u_snap_store(
    .I_clk        (S_csi_rx_clk         ),
    .I_rst_n      (S_rst_n              ),
    .I_tuser      (S_snap_tuser         ),
    .I_tvalid     (S_snap_tvalid        ),
    .I_tdata      (S_snap_tdata         ),
    .I_snap_trig  (S_snap_trig_stretch  ),
    .I_manual_snap_trig (S_manual_snap_trig_stretch),
    .I_manual_fifo_data (S_manual_snap_fifo_dout),
    .I_manual_fifo_valid(S_manual_snap_fifo_valid),
    .I_manual_fifo_empty(S_manual_snap_fifo_empty),
    .O_manual_fifo_rd   (S_manual_snap_fifo_re),
    .O_busy             (S_snap_store_busy),
    .I_fire_valid (|S_bbox_valid_cam    ),
    .I_cig_valid  (S_cig_valid_cam      ),
    .I_fire_area  (S_area_cam[19:0]     ),
    .I_cig_count  (S_cig_count_cam      ),
    .I_fxmin      (S_bbox_xmin_cam[10:0]),
    .I_fxmax      (S_bbox_xmax_cam[10:0]),
    .I_fymin      (S_bbox_ymin_cam[9:0] ),
    .I_fymax      (S_bbox_ymax_cam[9:0] ),
    .I_cxmin      (S_cig_xmin_cam       ),
    .I_cxmax      (S_cig_xmax_cam       ),
    .I_cymin      (S_cig_ymin_cam       ),
    .I_cymax      (S_cig_ymax_cam       ),
    .O_sec_write  (S_sec_write          ),
    .O_sec_addr   (S_sec_addr           ),
    .O_sec_data   (S_sec_data           ),
    .O_snap_count (S_snap_count         ),
    .I_data_req   (S_sec_data_req       ),
    .I_write_end  (S_sec_write_end      )
);
sd_card_top #(
    .SPI_LOW_SPEED_DIV  (248),
    .SPI_HIGH_SPEED_DIV (1)
) u_sd_card_top(
    .clk                  (S_csi_rx_clk    ),
    .rst                  (~S_rst_n        ),
    .SD_nCS               (SD_nCS          ),
    .SD_DCLK              (SD_DCLK         ),
    .SD_MOSI              (SD_MOSI         ),
    .SD_MISO              (SD_MISO         ),
    .sd_init_done         (),
    .sd_sec_read          (1'b0            ),
    .sd_sec_read_addr     (32'd0           ),
    .sd_sec_read_data     (),
    .sd_sec_read_data_valid(),
    .sd_sec_read_end      (),
    .sd_sec_write         (S_sec_write     ),
    .sd_sec_write_addr    (S_sec_addr      ),
    .sd_sec_write_data    (S_sec_data      ),
    .sd_sec_write_data_req(S_sec_data_req  ),
    .sd_sec_write_end     (S_sec_write_end )
);

// ---- 模块: UART 调试输出 (检测状态每秒一行) ----
uart_debug u_uart_debug(
    .I_clk        (S_hdmi_pixel_clk),
    .I_rst_n      (S_hdmi_rst_n    ),
    .I_frame_tick (S_frame_tick    ),
    .I_fire_valid (S_fire_valid    ),
    .I_cig_valid  (S_cig_valid     ),
    .I_fire_conf  (S_confidence_sync),
    .I_cig_conf   (S_cig_conf_sync ),
    .I_level      (S_conf_level    ),
    .I_lum_th     (S_lum_th_sync   ),
    .I_fps        (S_fps_reg       ),
    .I_latency    (S_latency_sync  ),
    .I_snap_count (S_snap_count    ),
    .O_tx_pin     (O_uart_tx       )
);

// ---- 时延量化: ISP 输入帧头 -> ISP 输出帧头 的拍数 (csi时钟域) ----
always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
    if(!S_rst_n) begin
        S_lat_cnt     <= 16'd0;
        S_latency_cam <= 16'd0;
    end else if(S_raw_tuser) begin
        S_lat_cnt <= 16'd0;
    end else if(S_ISP_O_tuser) begin
        S_latency_cam <= S_lat_cnt;
        S_lat_cnt     <= 16'd0;
    end else begin
        S_lat_cnt <= S_lat_cnt + 16'd1;
    end
end
// ---- K3/K4调节面积门槛；K1切换视图；K2短按暂停、长按手动抓拍 ----
    reg S_key2_sync1, S_key2_sync2;
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_key2_sync1 <= 1'b0;
            S_key2_sync2 <= 1'b0;
        end else begin
            S_key2_sync1 <= ~I_button[0];
            S_key2_sync2 <= S_key2_sync1;
        end
    end
    wire S_key2_press_level = S_key2_sync2;
    key_calib #(
        .LONG_CNT     (28'd22275000), // 300ms @ 74.25MHz
        .DEBOUNCE_CNT (22'd100000)    // 约1.35ms消抖
    ) u_key2_manual_capture(
        .I_clk        (S_hdmi_pixel_clk),
        .I_rst_n      (S_hdmi_rst_n),
        .I_key1       (1'b0),
        .I_key2       (S_key2_press_level),
        .O_key1_short (),
        .O_key1_long  (),
        .O_key2_short(S_key2_short),
        .O_key2_long (S_key2_long)
    );
    reg S_area_down_trig;
    reg S_area_up_trig;
    reg S_area_key_dn_sync1, S_area_key_dn_sync2;
    reg S_area_key_up_sync1, S_area_key_up_sync2;
    reg S_area_key_dn_pressed, S_area_key_up_pressed;
    reg [20:0] S_area_key_dn_db_cnt, S_area_key_up_db_cnt;
    reg [25:0] S_area_key_dn_hold_cnt, S_area_key_up_hold_cnt;
    reg S_area_key_dn_repeat, S_area_key_up_repeat;
    localparam [20:0] AREA_KEY_DEBOUNCE_CYCLES = 21'd1113750; // 15ms @ 74.25MHz
    localparam [25:0] AREA_KEY_REPEAT_DELAY_CYCLES = 26'd33412500; // 450ms
    localparam [23:0] AREA_KEY_REPEAT_INTERVAL_CYCLES = 24'd11137500; // 150ms

    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_area_key_dn_sync1 <= 1'b0;
            S_area_key_dn_sync2 <= 1'b0;
            S_area_key_up_sync1 <= 1'b0;
            S_area_key_up_sync2 <= 1'b0;
            S_area_key_dn_pressed <= 1'b0;
            S_area_key_up_pressed <= 1'b0;
            S_area_key_dn_db_cnt <= 21'd0;
            S_area_key_up_db_cnt <= 21'd0;
            S_area_key_dn_hold_cnt <= 26'd0;
            S_area_key_up_hold_cnt <= 26'd0;
            S_area_key_dn_repeat <= 1'b0;
            S_area_key_up_repeat <= 1'b0;
            S_area_down_trig <= 1'b0;
            S_area_up_trig <= 1'b0;
        end else begin
            S_area_key_dn_sync1 <= ~I_button[2];
            S_area_key_dn_sync2 <= S_area_key_dn_sync1;
            S_area_key_up_sync1 <= ~I_button[3];
            S_area_key_up_sync2 <= S_area_key_up_sync1;
            S_area_down_trig <= 1'b0;
            S_area_up_trig <= 1'b0;

            if(S_area_key_dn_sync2 == S_area_key_dn_pressed) begin
                S_area_key_dn_db_cnt <= 21'd0;
                if(S_area_key_dn_pressed) begin
                    if(!S_area_key_dn_repeat) begin
                        if(S_area_key_dn_hold_cnt >= AREA_KEY_REPEAT_DELAY_CYCLES - 1'b1) begin
                            S_area_key_dn_repeat <= 1'b1;
                            S_area_key_dn_hold_cnt <= 26'd0;
                            S_area_down_trig <= 1'b1;
                        end else begin
                            S_area_key_dn_hold_cnt <= S_area_key_dn_hold_cnt + 1'b1;
                        end
                    end else if(S_area_key_dn_hold_cnt >= AREA_KEY_REPEAT_INTERVAL_CYCLES - 1'b1) begin
                        S_area_key_dn_hold_cnt <= 26'd0;
                        S_area_down_trig <= 1'b1;
                    end else begin
                        S_area_key_dn_hold_cnt <= S_area_key_dn_hold_cnt + 1'b1;
                    end
                end else begin
                    S_area_key_dn_hold_cnt <= 26'd0;
                    S_area_key_dn_repeat <= 1'b0;
                end
            end else if(S_area_key_dn_db_cnt >= AREA_KEY_DEBOUNCE_CYCLES - 1'b1) begin
                S_area_key_dn_pressed <= S_area_key_dn_sync2;
                S_area_key_dn_db_cnt <= 21'd0;
                if(S_area_key_dn_sync2) begin
                    S_area_down_trig <= 1'b1;
                    S_area_key_dn_hold_cnt <= 26'd0;
                    S_area_key_dn_repeat <= 1'b0;
                end else begin
                    S_area_key_dn_hold_cnt <= 26'd0;
                    S_area_key_dn_repeat <= 1'b0;
                end
            end else begin
                S_area_key_dn_db_cnt <= S_area_key_dn_db_cnt + 1'b1;
            end

            if(S_area_key_up_sync2 == S_area_key_up_pressed) begin
                S_area_key_up_db_cnt <= 21'd0;
                if(S_area_key_up_pressed) begin
                    if(!S_area_key_up_repeat) begin
                        if(S_area_key_up_hold_cnt >= AREA_KEY_REPEAT_DELAY_CYCLES - 1'b1) begin
                            S_area_key_up_repeat <= 1'b1;
                            S_area_key_up_hold_cnt <= 26'd0;
                            S_area_up_trig <= 1'b1;
                        end else begin
                            S_area_key_up_hold_cnt <= S_area_key_up_hold_cnt + 1'b1;
                        end
                    end else if(S_area_key_up_hold_cnt >= AREA_KEY_REPEAT_INTERVAL_CYCLES - 1'b1) begin
                        S_area_key_up_hold_cnt <= 26'd0;
                        S_area_up_trig <= 1'b1;
                    end else begin
                        S_area_key_up_hold_cnt <= S_area_key_up_hold_cnt + 1'b1;
                    end
                end else begin
                    S_area_key_up_hold_cnt <= 26'd0;
                    S_area_key_up_repeat <= 1'b0;
                end
            end else if(S_area_key_up_db_cnt >= AREA_KEY_DEBOUNCE_CYCLES - 1'b1) begin
                S_area_key_up_pressed <= S_area_key_up_sync2;
                S_area_key_up_db_cnt <= 21'd0;
                if(S_area_key_up_sync2) begin
                    S_area_up_trig <= 1'b1;
                    S_area_key_up_hold_cnt <= 26'd0;
                    S_area_key_up_repeat <= 1'b0;
                end else begin
                    S_area_key_up_hold_cnt <= 26'd0;
                    S_area_key_up_repeat <= 1'b0;
                end
            end else begin
                S_area_key_up_db_cnt <= S_area_key_up_db_cnt + 1'b1;
            end
        end
    end


    wire S_sw1_threshold = I_sw[0];
    wire S_sw2_capture   = I_sw[1];
    wire S_sw3_right_roi = I_sw[3];
    wire S_sw4_left_roi  = I_sw[2];
    wire S_threshold_mode = ~S_sw1_threshold; // SW1拨上: 手动阈值模式
    localparam [10:0] ROI_X_MAX = 11'd1277;
    localparam [10:0] ROI_STEP = 11'd8;
    wire S_roi_left_select  = S_sw4_left_roi && !S_sw3_right_roi;
    wire S_roi_right_select = S_sw3_right_roi && !S_sw4_left_roi;
    wire S_roi_select_valid = S_roi_left_select || S_roi_right_select;
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_roi_left_config_hdmi <= 11'd320;
            S_roi_right_config_hdmi <= 11'd960;
            S_roi_toggle_hdmi <= 1'b0;
        end else if(!S_threshold_mode && S_roi_select_valid && (S_area_down_trig || S_area_up_trig)) begin
            if(S_area_down_trig) begin // K3: selected boundary moves right
                if(S_roi_left_select) begin
                    if(S_roi_left_config_hdmi > ROI_X_MAX - ROI_STEP)
                        S_roi_left_config_hdmi <= ROI_X_MAX;
                    else
                        S_roi_left_config_hdmi <= S_roi_left_config_hdmi + ROI_STEP;
                end else if(S_roi_right_select) begin
                    if(S_roi_right_config_hdmi > ROI_X_MAX - ROI_STEP)
                        S_roi_right_config_hdmi <= ROI_X_MAX;
                    else
                        S_roi_right_config_hdmi <= S_roi_right_config_hdmi + ROI_STEP;
                end
                S_roi_toggle_hdmi <= ~S_roi_toggle_hdmi;
            end else begin // K4: selected boundary moves left
                if(S_roi_left_select)
                    S_roi_left_config_hdmi <= (S_roi_left_config_hdmi < ROI_STEP) ?
                                              11'd0 : S_roi_left_config_hdmi - ROI_STEP;
                else if(S_roi_right_select)
                    S_roi_right_config_hdmi <= (S_roi_right_config_hdmi < ROI_STEP) ?
                                               11'd0 : S_roi_right_config_hdmi - ROI_STEP;
                S_roi_toggle_hdmi <= ~S_roi_toggle_hdmi;
            end
        end
    end
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_min_area_config_hdmi <= 20'd64;
            S_min_area_toggle_hdmi <= 1'b0;
        end else if(S_threshold_mode && S_area_down_trig) begin
            if(S_min_area_config_hdmi > 20'd64) begin
                S_min_area_config_hdmi <= S_min_area_config_hdmi - 20'd64;
                S_min_area_toggle_hdmi <= ~S_min_area_toggle_hdmi;
            end
        end else if(S_threshold_mode && S_area_up_trig) begin
            if(S_min_area_config_hdmi < 20'd65536) begin
                S_min_area_config_hdmi <= S_min_area_config_hdmi + 20'd64;
                S_min_area_toggle_hdmi <= ~S_min_area_toggle_hdmi;
            end
        end
    end

    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_snap_busy_sync1_hdmi <= 1'b0;
            S_snap_busy_sync2_hdmi <= 1'b0;
            S_manual_snap_trig_stretch <= 1'b0;
            S_manual_snap_stretch_cnt <= 4'd0;
        end else begin
            S_snap_busy_sync1_hdmi <= S_snap_store_busy;
            S_snap_busy_sync2_hdmi <= S_snap_busy_sync1_hdmi;
            if(S_key2_long && !S_snap_busy_hdmi) begin
                S_manual_snap_trig_stretch <= 1'b1;
                S_manual_snap_stretch_cnt <= 4'd0;
            end else if(S_manual_snap_trig_stretch) begin
                if(S_manual_snap_stretch_cnt == 4'd15)
                    S_manual_snap_trig_stretch <= 1'b0;
                else
                    S_manual_snap_stretch_cnt <= S_manual_snap_stretch_cnt + 4'd1;
            end
        end
    end

    reg  S_display_pause;

    always @(posedge S_hdmi_pixel_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_display_pause <= 1'b0;
        else if(S_key2_short)
            S_display_pause <= ~S_display_pause;
    end

    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_bbox_xmin_pause <= 22'd0;
            S_bbox_xmax_pause <= 22'd0;
            S_bbox_ymin_pause <= 20'd0;
            S_bbox_ymax_pause <= 20'd0;
            S_bbox_valid_pause <= 2'b0;
            S_fire_alarm_pause <= 1'b0;
        end else if(!S_display_pause) begin
            S_bbox_xmin_pause <= S_bbox_xmin_sync;
            S_bbox_xmax_pause <= S_bbox_xmax_sync;
            S_bbox_ymin_pause <= S_bbox_ymin_sync;
            S_bbox_ymax_pause <= S_bbox_ymax_sync;
            S_bbox_valid_pause <= S_bbox_valid_sync;
            S_fire_alarm_pause <= O_led_fire;
        end
    end

    reg [3:0]  S_debug_status_pause;
    reg [19:0] S_candidate_count_osd_pause, S_min_area_osd_pause, S_max_area_osd_pause;
    reg        S_threshold_mode_osd_pause;
    reg [19:0] S_area_th_osd_pause;
    reg [15:0] S_latency_osd_pause;
    reg [6:0]  S_fps_osd_pause;
    reg [7:0]  S_snap_count_osd_pause;
    reg        S_fire_valid_osd_pause;
    reg [7:0]  S_fire_conf_osd_pause, S_cig_conf_osd_pause;
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_debug_status_pause <= 4'd0;
            S_candidate_count_osd_pause <= 20'd0;
            S_min_area_osd_pause <= 20'd64;
            S_max_area_osd_pause <= 20'd0;
            S_threshold_mode_osd_pause <= 1'b0;
            S_area_th_osd_pause <= 20'd64;
            S_latency_osd_pause <= 16'd0;
            S_fps_osd_pause <= 7'd0;
            S_snap_count_osd_pause <= 8'd0;
            S_fire_valid_osd_pause <= 1'b0;
            S_fire_conf_osd_pause <= 8'd0;
            S_cig_conf_osd_pause <= 8'd0;
        end else if(!S_display_pause) begin
            S_debug_status_pause <= S_hdmi_debug_status;
            S_candidate_count_osd_pause <= S_candidate_count_sync;
            S_min_area_osd_pause <= S_min_area_config_hdmi;
            S_max_area_osd_pause <= S_area_sync[19:0];
            S_threshold_mode_osd_pause <= S_threshold_mode;
            S_area_th_osd_pause <= S_area_th_sync;
            S_latency_osd_pause <= S_latency_sync;
            S_fps_osd_pause <= S_fps_reg;
            S_snap_count_osd_pause <= S_snap_count;
            S_fire_valid_osd_pause <= S_fire_valid;
            S_fire_conf_osd_pause <= S_confidence_sync;
            S_cig_conf_osd_pause <= S_cig_conf_sync;
        end
    end

    // 暂停信号两级同步到DDR时钟域(video_in的I_display_pause在该域采样)
    reg S_display_pause_s1;
    reg S_display_pause_s2;
    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n) begin
            S_display_pause_s1 <= 1'b0;
            S_display_pause_s2 <= 1'b0;
        end
        else begin
            S_display_pause_s1 <= S_display_pause;
            S_display_pause_s2 <= S_display_pause_s1;
        end
    end

    // ================= 三用户 DDR 仲裁 (vi写原图 / qw写拼接帧 / vo读显示) =================
    // 优先级: vi > qw > vo; busy门控使各用户触发条件互斥, 任意两用户 wr/rd 不可能同拍:
    //   qw_ready 在 vi 使能或 busy 时拉低, vo_ready 再叠加 qw 的 en/busy 掩码 (vo 优先级最低)
    //   -> wr_en=vi|qw 至多一个为1, 读仅在两写用户都空闲时发生, 无同拍冲突/无死锁
    // qw 的 ready 掩码含 vi_busy: 防 vi/qw 同拍触发时下一拍同拍 en 丢 qw 字
    assign S_qw_ddr_wr_ready = S_ddr_user_ready & ~S_vi_ddr_wr_en & ~S_video_in_wr_busy;
    assign S_vo_ddr_rd_ready = S_ddr_user_ready & ~S_vi_ddr_wr_en & ~S_video_in_wr_busy
                                                & ~S_qw_ddr_wr_en & ~S_quad_wr_busy;

    // ---- 拼接帧写入 (quad_writer): 相机域收流 -> FIFO -> DDR 空闲区乒乓 ----
    // 基址十进制 25'd28000000/25'd30000000 (间距2M单位=4MB > 拼接帧1,382,400单位/2.76MB;
    // 25位地址上限33,554,431, 官方基址 7000000/14000000/21000000 同为十进制)
    // 帧同步锁相: I_quad_frame_start 与 video_in 同源, 偶帧头复位FIFO+行计数,
    // 上电相位任意/MIPI丢帧均在下个偶帧头自愈; 每拼接帧写完翻转乒乓指针
    quad_writer #(
        .QUAD_BASE_0 (25'd28000000),
        .QUAD_BASE_1 (25'd30000000)
    ) u_quad_writer(
        .I_rst_n            ( S_rst_n            ),
        .I_quad_clk         ( S_csi_rx_clk       ),
        .I_quad_valid       ( S_quad_tvalid_cam  ),
        .I_quad_data        ( S_quad_tdata_cam   ),
        .I_quad_frame_start ( S_ISP_O_tuser      ),
        .I_ddr_clk          ( S_ddr_clk          ),
        .I_vi_wr_busy       ( S_video_in_wr_busy ),
        .I_vo_rd_busy       ( S_video_out_rd_busy),
        .I_display_pause    ( S_display_pause_s2 ),   // KEY2暂停: 冻结乒乓翻转, 画面静止
        .I_ddr_user_ready   ( S_qw_ddr_wr_ready  ),
        .O_ddr_user_wr_en   ( S_qw_ddr_wr_en     ),
        .O_ddr_user_addr    ( S_qw_ddr_wr_addr   ),
        .O_ddr_user_wr_data ( S_qw_ddr_wr_data   ),
        .O_wr_busy          ( S_quad_wr_busy     ),
        .O_quad_rp          ( S_quad_rp          )
    );

    // ---- 四分屏读地址偏移 + 乒乓基址选择 (DDR域) ----
    // QUAD模式下: vo 读指针固定按 rp=0 行走, 实际地址 = 拼接帧基址(S_quad_rp选) + 行内偏移
    // (拼接帧布局与 vo 线性读地址完全一致: 1280x720, 行stride=1920, 60fps每拼接帧读两次)
    // 乒乓翻转发生在拼接帧写完的瞬间, 显示内容恒为完整帧(最多垂直平移数行, 不可见)
    localparam [24:0] QUAD_BASE_0_LP = 25'd28000000;
    localparam [24:0] QUAD_BASE_1_LP = 25'd30000000;
    wire S_quad_mode_hdmi;                        // 驱动源: quad_top (显示域)
    reg  S_quad_mode_s1, S_quad_mode_ddr;         // 2级同步到 DDR 域
    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n) begin
            S_quad_mode_s1  <= 1'b1;              // 复位默认 QUAD 模式, 与 quad_top 上电一致
            S_quad_mode_ddr <= 1'b1;
        end
        else begin
            S_quad_mode_s1  <= S_quad_mode_hdmi;
            S_quad_mode_ddr <= S_quad_mode_s1;
        end
    end
    wire [1:0]  S_vo_rp_eff      = S_quad_mode_ddr ? 2'd0 : S_video_out_rp;
    wire [24:0] S_vo_rd_addr_eff = S_quad_mode_ddr ?
                  (S_quad_rp ? QUAD_BASE_1_LP : QUAD_BASE_0_LP) + S_vo_ddr_rd_addr
                : S_vo_ddr_rd_addr;

    video_in u_video_in(
        .I_rst_n              (  S_rst_n             ),

        .I_camera_clk         ( S_csi_rx_clk          ),
        .I_camera_frame_start ( S_ISP_O_tuser  ),
        .I_camera_valid       ( S_ISP_O_tvalid ),
        .I_camera_data        ( S_ISP_O_tdata  ),
        .I_mipi_rx_error      ( 1'b0			      ),

        .I_ddr_clk            ( S_ddr_clk             ),
		.I_display_pause      ( S_display_pause_s2    ),   // KEY2画面暂停
        .I_video_out_rd_busy  ( S_video_out_rd_busy | S_quad_wr_busy ),
        .O_video_in_wr_busy   ( S_video_in_wr_busy    ),
        .O_video_out_rp       ( S_video_out_rp        ),

        .O_ddr_user_wr_en     ( S_vi_ddr_wr_en        ),
        .O_ddr_user_addr      ( S_vi_ddr_wr_addr      ),
        .O_ddr_user_wr_data   ( S_vi_ddr_wr_data      ),
        .I_ddr_user_ready     ( S_ddr_user_ready      )
    );



    assign S_ddr_user_wr_en    = S_vi_ddr_wr_en | S_qw_ddr_wr_en;

    assign S_ddr_user_rd_en    = S_vo_ddr_rd_en;

    assign S_ddr_user_addr     = S_vi_ddr_wr_en ? S_vi_ddr_wr_addr :
                                 S_qw_ddr_wr_en ? S_qw_ddr_wr_addr :
                                 S_vo_ddr_rd_en ? S_vo_rd_addr_eff  : 'd0;

    assign S_ddr_user_wr_data  = S_vi_ddr_wr_en ? S_vi_ddr_wr_data  : S_qw_ddr_wr_data;


    assign S_video_out_rst_n = S_hdmi_rst_n;

    video_out u_video_out(
        .I_rst_n             ( S_video_out_rst_n    ),
        .I_ddr_clk           ( S_ddr_clk            ),
 
        .O_video_out_rd_busy ( S_video_out_rd_busy  ),
        .I_video_in_wr_busy  ( S_video_in_wr_busy | S_quad_wr_busy ),
        .I_video_out_rp      ( S_vo_rp_eff         ),
 
        .O_ddr_user_rd_en    ( S_vo_ddr_rd_en       ),
        .O_ddr_user_addr     ( S_vo_ddr_rd_addr     ),
        .I_ddr_user_ready    ( S_vo_ddr_rd_ready     ),
        .I_ddr_user_rd_valid ( S_ddr_user_rd_valid  ),
        .I_ddr_user_rd_data  ( S_ddr_user_rd_data   ),

        .I_dsi_clk           ( S_hdmi_pixel_clk     ),
        .I_video_vsync       ( S_video_out_vsync    ),
        .I_video_rd_en       ( S_hdmi_window_rd_en  ),
        .O_vdieo_data        ( S_video_rd_data      )
    );



    mc_to_user_interface u_mc_to_user_interface(
        .I_clk                   ( S_ddr_clk               ),
        .I_rst_n                 ( S_rst_n                 ),

        .I_ddr_user_wr_en        ( S_ddr_user_wr_en        ),
        .I_ddr_user_rd_en        ( S_ddr_user_rd_en        ),
        .I_ddr_user_addr         ( S_ddr_user_addr         ),
        .I_ddr_user_wr_data      ( S_ddr_user_wr_data      ),
        .O_ddr_user_ready        ( S_ddr_user_ready        ),
        .O_ddr_user_rd_valid     ( S_ddr_user_rd_valid     ),
        .O_ddr_user_rd_data      ( S_ddr_user_rd_data      ),
            
        .O_mc_app_en             ( S_mc_app_en             ),
        .O_mc_app_addr           ( S_mc_app_addr           ),
        .O_mc_app_cmd            ( S_mc_app_cmd            ),
        .I_mc_app_rdy            ( S_mc_app_rdy            ),
        .O_mc_app_wdf_wren       ( S_mc_app_wdf_wren       ),
        .O_mc_app_wdf_data       ( S_mc_app_wdf_data       ),
        .O_mc_app_wdf_end        ( S_mc_app_wdf_end        ),
        .O_mc_app_wdf_mask       ( S_mc_app_wdf_mask       ),
        .I_mc_app_wdf_rdy        ( S_mc_app_wdf_rdy        ),
        .I_mc_app_rd_data        ( S_mc_app_rd_data        ),
        .I_mc_app_rd_data_end    ( S_mc_app_rd_data_end    ),
        .I_mc_app_rd_data_valid  ( S_mc_app_rd_data_valid  )
    );



    ph1p35_324_ddr_wrapper u_ph1p35_324_ddr_wrapper(
        .I_sys_clk              ( I_sys_clk              ),
        .I_sys_rst_n            ( S_rst_n                ),

        .O_ddr_clk              ( S_ddr_clk              ),
        .O_init_calib_complete  ( S_init_calib_complete  ),
        .I_mc_app_addr          ( S_mc_app_addr          ),
        .I_mc_app_cmd           ( S_mc_app_cmd           ),
        .I_mc_app_en            ( S_mc_app_en            ),
        .I_mc_app_wdf_data      ( S_mc_app_wdf_data      ),
        .I_mc_app_wdf_end       ( S_mc_app_wdf_end       ),
        .I_mc_app_wdf_mask      ( S_mc_app_wdf_mask      ),
        .I_mc_app_wdf_wren      ( S_mc_app_wdf_wren      ),
        .O_mc_app_rd_data       ( S_mc_app_rd_data       ),
        .O_mc_app_rd_data_end   ( S_mc_app_rd_data_end   ),
        .O_mc_app_rd_data_valid ( S_mc_app_rd_data_valid ),
        .O_mc_app_rdy           ( S_mc_app_rdy           ),
        .O_mc_app_wdf_rdy       ( S_mc_app_wdf_rdy       ),

        .ddr_addr               ( ddr_addr               ),
        .ddr_ba                 ( ddr_ba                 ),
        .ddr_cke                ( ddr_cke                ),
        .ddr_odt                ( ddr_odt                ),
        .ddr_cs_n               ( ddr_cs_n               ),
        .ddr_ras_n              ( ddr_ras_n              ),
        .ddr_cas_n              ( ddr_cas_n              ),
        .ddr_we_n               ( ddr_we_n               ),
        .ddr_ck_p               ( ddr_ck_p               ),
        .ddr_ck_n               ( ddr_ck_n               ),
        .ddr_dm                 ( ddr_dm                 ),
        .ddr_dq                 ( ddr_dq                 ),
        .ddr_dqs_p              ( ddr_dqs_p              ),
        .ddr_dqs_n              ( ddr_dqs_n              )
    );



    uivtc #(
        .H_ActiveSize ( 1280 ),
        .H_FrameSize  ( 1650 ),
        .H_SyncStart  ( 1390 ),
        .H_SyncEnd    ( 1430 ),
        .V_ActiveSize ( 720  ),
        .V_FrameSize  ( 750  ),
        .V_SyncStart  ( 725  ),
        .V_SyncEnd    ( 730  )
    )u_hdmi_vtc(
        .I_vtc_rstn    ( S_hdmi_rst_n    ),
        .I_vtc_clk     ( S_hdmi_pixel_clk ),
        .O_vtc_vs      ( S_hdmi_vsync     ),
        .O_vtc_hs      ( S_hdmi_hsync     ),
        .O_vtc_de_valid( S_hdmi_de        ),
        .O_vtc_user    ( S_hdmi_user      ),
        .O_vtc_last    ( S_hdmi_last      )
    );

    hdmi_mixer #(
        .H_OFFSET   ( 0    ),
        .V_OFFSET   ( 0    ),
        .IMG_WIDTH  ( 1280 ),
        .IMG_HEIGHT ( 720  ),
        .DEBUG_MODE ( 0    )
    )u_hdmi_mixer(
        .I_clk           ( S_hdmi_pixel_clk   ),
        .I_rst_n         ( S_hdmi_rst_n       ),
        .I_video_vsync   ( S_hdmi_vsync       ),
        .I_video_hsync   ( S_hdmi_hsync       ),
        .I_video_de      ( S_hdmi_de          ),
        .I_video_user    ( S_hdmi_user        ),
        .I_video_last    ( S_hdmi_last        ),
        .I_debug_status  ( S_display_pause ? S_debug_status_pause : S_hdmi_debug_status),
        .I_candidate_count(S_display_pause ? S_candidate_count_osd_pause : S_candidate_count_sync),
        .I_min_area      ( S_display_pause ? S_min_area_osd_pause : S_min_area_config_hdmi),
        .I_max_area      ( S_display_pause ? S_max_area_osd_pause : S_area_sync[19:0]),
        .I_threshold_mode( S_display_pause ? S_threshold_mode_osd_pause : S_threshold_mode),
        .O_video_rd_en   ( S_hdmi_window_rd_en),
        .I_video_rd_data ( S_video_rd_data    ),
        .O_hdmi_vsync    ( S_hdmi_out_vsync   ),
        .O_hdmi_hsync    ( S_hdmi_out_hsync   ),
        .O_hdmi_de       ( S_hdmi_out_de      ),
        .O_hdmi_data     ( S_hdmi_out_data    )
    );

    // ========== 模块2: OSD矩形框选 (仅显示明火红框；烟头候选不叠加黄框) ==========
    // 坐标源: color_mask_96候选外接框 (经toggle CDC同步到hdmi域, 帧级锁存)
    // 双目标(OBJ_MAX=2): 两个红光连通域分别画框, 任一目标有效即触发告警
    wire [23:0] S_box_rgb;
    wire [4:0]  S_box_alpha;
    wire        S_box_de, S_box_hsync, S_box_vsync;
        wire [1:0]  S_bbox_valid_display = S_process_quad_hdmi ? 2'b00 : (S_display_pause ? S_bbox_valid_pause : S_bbox_valid_sync);
    wire [2:0]  S_box_valid = {1'b0, S_bbox_valid_display};  // paused image uses its frozen frame's box-valid flags

    // 每个目标独立映射: QUAD模式缩小一半并平移到右下象限;
    // 全屏模式保留摄像头原坐标。每段打包宽度与 osd_box 的端口约定一致。
    wire [32:0] S_bbox_xmin_eff;
    wire [32:0] S_bbox_xmax_eff;
    wire [29:0] S_bbox_ymin_eff;
    wire [29:0] S_bbox_ymax_eff;
    genvar box_i;
    generate
        for(box_i=0; box_i<2; box_i=box_i+1) begin : GEN_BBOX_MAP
            wire [21:0] bbox_xmin_src = S_display_pause ? S_bbox_xmin_pause : S_bbox_xmin_sync;
            wire [21:0] bbox_xmax_src = S_display_pause ? S_bbox_xmax_pause : S_bbox_xmax_sync;
            wire [19:0] bbox_ymin_src = S_display_pause ? S_bbox_ymin_pause : S_bbox_ymin_sync;
            wire [19:0] bbox_ymax_src = S_display_pause ? S_bbox_ymax_pause : S_bbox_ymax_sync;
            wire [10:0] xmin_in = bbox_xmin_src[box_i*11 +: 11];
            wire [10:0] xmax_in = bbox_xmax_src[box_i*11 +: 11];
            wire [9:0]  ymin_in = bbox_ymin_src[box_i*10 +: 10];
            wire [9:0]  ymax_in = bbox_ymax_src[box_i*10 +: 10];

            assign S_bbox_xmin_eff[box_i*11 +: 11] = S_quad_mode_hdmi ?
                ({1'b0, xmin_in[10:1]} + 11'd640) : xmin_in;
            assign S_bbox_xmax_eff[box_i*11 +: 11] = S_quad_mode_hdmi ?
                ({1'b0, xmax_in[10:1]} + 11'd640) : xmax_in;
            assign S_bbox_ymin_eff[box_i*10 +: 10] = S_quad_mode_hdmi ?
                ({1'b0, ymin_in[9:1]} + 10'd360) : ymin_in;
            assign S_bbox_ymax_eff[box_i*10 +: 10] = S_quad_mode_hdmi ?
                ({1'b0, ymax_in[9:1]} + 10'd360) : ymax_in;
        end
    endgenerate
    // 目标2坐标仍保留供日志/调试；当前不绘制烟头候选黄框
    assign S_bbox_xmin_eff[2*11 +: 11] = S_quad_mode_hdmi ?
        ({1'b0, S_cig_xmin_sync[10:1]} + 11'd640) : S_cig_xmin_sync;
    assign S_bbox_xmax_eff[2*11 +: 11] = S_quad_mode_hdmi ?
        ({1'b0, S_cig_xmax_sync[10:1]} + 11'd640) : S_cig_xmax_sync;
    assign S_bbox_ymin_eff[2*10 +: 10] = S_quad_mode_hdmi ?
        ({1'b0, S_cig_ymin_sync[9:1]} + 10'd360) : S_cig_ymin_sync;
    assign S_bbox_ymax_eff[2*10 +: 10] = S_quad_mode_hdmi ?
        ({1'b0, S_cig_ymax_sync[9:1]} + 10'd360) : S_cig_ymax_sync;
    osd_box #(
        .OBJ_MAX  (3),
        .BORDER_T (4'd4)
    ) u_osd_box(
        .I_clk        (S_hdmi_pixel_clk),
        .I_rst_n      (S_hdmi_rst_n    ),
        .I_de         (S_hdmi_out_de   ),
        .I_hsync      (S_hdmi_out_hsync),
        .I_vsync      (S_hdmi_out_vsync),
        .I_xmin       (S_bbox_xmin_eff   ),
        .I_ymin       (S_bbox_ymin_eff   ),
        .I_xmax       (S_bbox_xmax_eff   ),
        .I_ymax       (S_bbox_ymax_eff   ),
        .I_obj_valid  (S_box_valid     ),
        .I_obj_class  (6'b100101),       // 目标2=烟头(10), 目标1/0=明火(01)
        .O_osd_pixel  (S_box_rgb       ),
        .O_osd_alpha  (S_box_alpha     ),
        .O_de_out     (S_box_de        ),
        .O_hsync_out  (S_box_hsync     ),
        .O_vsync_out  (S_box_vsync     )
    );

    // Alpha混合 (整数运算): out = (video*(16-a) + osd*a + 8) >> 4
    // 视频延迟1拍, 与osd_box输出同拍对齐 (模块内1拍流水已配平)
    reg [23:0] S_box_video_d;
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n)
            S_box_video_d <= 24'd0;
        else
            S_box_video_d <= S_hdmi_out_data;
    end
    wire [13:0] S_blend_r_full = ({6'd0, S_box_video_d[23:16]} * (5'd16 - S_box_alpha)) +
                                 ({6'd0, S_box_rgb[23:16]} * S_box_alpha) + 14'd8;
    wire [13:0] S_blend_g_full = ({6'd0, S_box_video_d[15:8]}  * (5'd16 - S_box_alpha)) +
                                 ({6'd0, S_box_rgb[15:8]}  * S_box_alpha) + 14'd8;
    wire [13:0] S_blend_b_full = ({6'd0, S_box_video_d[7:0]}   * (5'd16 - S_box_alpha)) +
                                 ({6'd0, S_box_rgb[7:0]}   * S_box_alpha) + 14'd8;
    wire [23:0] S_blend_rgb = {S_blend_r_full[11:4],
                               S_blend_g_full[11:4],
                               S_blend_b_full[11:4]};
    wire [23:0] S_hdmi_tx_data = S_box_de ? S_blend_rgb : S_box_video_d;

    // ========== 模块3: 告警OSD (检测到目标时屏幕边缘空心红框+右上角红色"ALARM") ==========
    // 触发源: S_fire_valid = 任一目标有效 (bbox CDC帧级同步, hdmi域稳定)
    // 端口无 de 输入: 模块内由 hsync/vsync 相位锚定自生成 de (与 osd_box 同域级联)
    // 接口/流水结构与 osd_box 完全一致, 第二级 alpha 混合叠加在框选之上
    wire [23:0] S_alarm_rgb;
    wire [3:0]  S_alarm_alpha;   // 4位: 0=透明 15=不透明, 混合公式 out=(v*(16-a)+osd*a+8)>>4
    wire        S_alarm_de, S_alarm_hsync, S_alarm_vsync;
    osd_alarm u_osd_alarm(
        .I_clk        (S_hdmi_pixel_clk),
        .I_rst_n      (S_hdmi_rst_n    ),
        .I_hsync      (S_box_hsync     ),
        .I_vsync      (S_box_vsync     ),
        .I_alarm      (S_process_quad_hdmi ? 1'b0 : (S_display_pause ? S_fire_alarm_pause : O_led_fire)),
        .I_quad_mode  (S_quad_mode_hdmi),   // QUAD模式: ALARM文字1x缩至右下象限
        .O_osd_pixel  (S_alarm_rgb     ),
        .O_osd_alpha  (S_alarm_alpha   ),
        .O_de_out     (S_alarm_de      ),
        .O_hsync_out  (S_alarm_hsync   ),
        .O_vsync_out  (S_alarm_vsync   )
    );

    // 视频延迟1拍, 与告警文字输出同拍对齐 (模块内1拍流水已配平)
    reg [23:0] S_box_blend_d;
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n)
            S_box_blend_d <= 24'd0;
        else
            S_box_blend_d <= S_hdmi_tx_data;
    end
    // 第二级 Alpha混合 (整数运算): out = (video*(16-a) + osd*a + 8) >> 4
    wire [13:0] S_blend2_r_full = ({6'd0, S_box_blend_d[23:16]} * (5'd16 - S_alarm_alpha)) +
                                  ({6'd0, S_alarm_rgb[23:16]} * S_alarm_alpha) + 14'd8;
    wire [13:0] S_blend2_g_full = ({6'd0, S_box_blend_d[15:8]}  * (5'd16 - S_alarm_alpha)) +
                                  ({6'd0, S_alarm_rgb[15:8]}  * S_alarm_alpha) + 14'd8;
    wire [13:0] S_blend2_b_full = ({6'd0, S_box_blend_d[7:0]}   * (5'd16 - S_alarm_alpha)) +
                                  ({6'd0, S_alarm_rgb[7:0]}   * S_alarm_alpha) + 14'd8;
    wire [23:0] S_blend2_rgb = {S_blend2_r_full[11:4],
                                S_blend2_g_full[11:4],
                                S_blend2_b_full[11:4]};
    wire [23:0] S_hdmi_tx_data_final = S_alarm_de ? S_blend2_rgb : S_box_blend_d;

    // ========== 模块3.5: 检测链状态仪表盘 OSD (osd_status, 全屏视图左下角仪器版面板) ==========
    // 第三级 OSD, 级联在告警文字之后, 同构 1 拍流水 (与 quad_top 的 vid+3拍 对齐约定不变)。
    // 显隐组合方案: I_show=(view_mode!=0) 全屏视图常显; 内部 popup 逻辑在告警时强制弹出
    // (QUAD 显示判定流, 弹窗面板自然叠加), 消退3秒后收起。
    wire [23:0] S_param_rgb;
    wire [3:0]  S_param_alpha;
    wire        S_param_de, S_param_hsync, S_param_vsync;
    wire [2:0]  S_view_mode_hdmi;
    osd_status u_osd_status(
        .I_clk        (S_hdmi_pixel_clk     ),
        .I_rst_n      (S_hdmi_rst_n         ),
        .I_freeze     (S_display_pause     ),
        .I_de         (S_alarm_de           ),
        .I_hsync      (S_alarm_hsync        ),
        .I_vsync      (S_alarm_vsync        ),
        .I_show       (S_view_mode_hdmi >= 3'd2),  // 单屏视图常显
        .I_suppress   (S_process_quad_hdmi),  // 处理对比页保持四幅图干净
        .I_frame_tick (S_frame_tick         ),
        .I_cand_count (S_display_pause ? S_candidate_count_osd_pause : S_candidate_count_sync),
        .I_area_th    (S_display_pause ? S_area_th_osd_pause : S_area_th_sync),
        .I_latency    (S_display_pause ? S_latency_osd_pause : S_latency_sync),
        .I_fps        (S_display_pause ? S_fps_osd_pause : S_fps_reg),
        .I_snap_count (S_display_pause ? S_snap_count_osd_pause : S_snap_count),
        .I_fire_valid (S_process_quad_hdmi ? 1'b0 : (S_display_pause ? S_fire_valid_osd_pause : S_fire_valid)),
        .O_osd_pixel  (S_param_rgb          ),
        .O_osd_alpha  (S_param_alpha        ),
        .O_de_out     (S_param_de           ),
        .O_hsync_out  (S_param_hsync        ),
        .O_vsync_out  (S_param_vsync        )
    );
    // 视频延迟1拍, 与面板输出同拍对齐
    reg [23:0] S_alarm_blend_d;
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n)
            S_alarm_blend_d <= 24'd0;
        else
            S_alarm_blend_d <= S_hdmi_tx_data_final;
    end
    // 第三级 Alpha混合 (整数运算): 面板底衬半透明(8/16), 文字/边框不透明
    wire [13:0] S_blend3_r_full = ({6'd0, S_alarm_blend_d[23:16]} * (5'd16 - S_param_alpha)) +
                                  ({6'd0, S_param_rgb[23:16]} * S_param_alpha) + 14'd8;
    wire [13:0] S_blend3_g_full = ({6'd0, S_alarm_blend_d[15:8]}  * (5'd16 - S_param_alpha)) +
                                  ({6'd0, S_param_rgb[15:8]}  * S_param_alpha) + 14'd8;
    wire [13:0] S_blend3_b_full = ({6'd0, S_alarm_blend_d[7:0]}   * (5'd16 - S_param_alpha)) +
                                  ({6'd0, S_param_rgb[7:0]}   * S_param_alpha) + 14'd8;
    wire [23:0] S_blend3_rgb = {S_blend3_r_full[11:4],
                                S_blend3_g_full[11:4],
                                S_blend3_b_full[11:4]};
    wire [23:0] S_hdmi_tx_data_param = S_param_de ? S_blend3_rgb : S_alarm_blend_d;

    // ========== 模块3.6: 可信度与分级告警 OSD ==========
    // 置信度在四分屏 TARGET 区与 DET 全屏显示；分数取明火/烟头两路较高值。
    wire [23:0] S_conf_rgb;
    wire [3:0]  S_conf_alpha;
    wire        S_conf_de, S_conf_hsync, S_conf_vsync;
    osd_confidence u_osd_confidence(
        .I_clk        (S_hdmi_pixel_clk),
        .I_rst_n      (S_hdmi_rst_n),
        .I_de         (S_param_de),
        .I_hsync      (S_param_hsync),
        .I_vsync      (S_param_vsync),
        .I_show       (S_view_mode_hdmi == 3'd5),
        .I_quad_mode  (S_quad_mode_hdmi && (S_view_mode_hdmi == 3'd0)),
        .I_fire_conf  (S_display_pause ? S_fire_conf_osd_pause : S_confidence_sync),
        .I_cig_conf   (S_display_pause ? S_cig_conf_osd_pause : S_cig_conf_sync),
        .O_osd_pixel  (S_conf_rgb),
        .O_osd_alpha  (S_conf_alpha),
        .O_de_out     (S_conf_de),
        .O_hsync_out  (S_conf_hsync),
        .O_vsync_out  (S_conf_vsync)
    );
    reg [23:0] S_param_blend_d;
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n)
            S_param_blend_d <= 24'd0;
        else
            S_param_blend_d <= S_hdmi_tx_data_param;
    end
    wire [13:0] S_blend4_r_full = ({6'd0, S_param_blend_d[23:16]} * (5'd16 - S_conf_alpha)) +
                                  ({6'd0, S_conf_rgb[23:16]}      * S_conf_alpha) + 14'd8;
    wire [13:0] S_blend4_g_full = ({6'd0, S_param_blend_d[15:8]} * (5'd16 - S_conf_alpha)) +
                                  ({6'd0, S_conf_rgb[15:8]}      * S_conf_alpha) + 14'd8;
    wire [13:0] S_blend4_b_full = ({6'd0, S_param_blend_d[7:0]}  * (5'd16 - S_conf_alpha)) +
                                  ({6'd0, S_conf_rgb[7:0]}       * S_conf_alpha) + 14'd8;
    wire [23:0] S_blend4_rgb = {S_blend4_r_full[11:4], S_blend4_g_full[11:4], S_blend4_b_full[11:4]};
    wire [23:0] S_hdmi_tx_data_conf = S_conf_de ? S_blend4_rgb : S_param_blend_d;
    // ========== 模块4: 四分屏/全屏视图 (KEY1 六态循环: 检测四分屏->处理四分屏->原图->边缘->掩码->目标框) ==========
    // I_vid_*: 原图流 (hdmi_mixer输出, 3拍); I_det_*: 判定流 (alarm流补1拍, 6拍)
    // 模块内: sobel_disp(原图+2拍)/mask_disp(原图+1拍)/延迟对齐, 四路统一到判定流相位
    // QUAD模式: 画面来自DDR拼接帧(quad_writer), 本模块仅叠加象限标签/分隔线/模式名
    // 全屏模式: 4:1 mux 直选对应流, 右上角叠加模式名
    wire        S_quad_ode, S_quad_ohsync, S_quad_ovsync;
    wire [23:0] S_quad_odata;
    quad_top u_quad_top(
        .I_clk        (S_hdmi_pixel_clk),
        .I_rst_n      (S_hdmi_rst_n    ),
        .I_key        (I_button[1]     ),   // KEY1: 六态循环 (低有效, 模块内消抖)
        .I_view_enable(1'b1),
        .I_sw2        (~S_sw2_capture  ),   // physical SW2 拨下(往SW2一侧)=显示抓拍使能图标
        .I_snap_count (S_snap_count_sync2_hdmi), // synchronized completed-snapshot count
        .I_sw_raw     (I_sw            ),   // 物理 SW1..SW4 原始高低电平诊断条
        .I_roi_left   (S_roi_left_config_hdmi),
        .I_roi_right  (S_roi_right_config_hdmi),
        .I_vid_de     (S_hdmi_out_de   ),
        .I_vid_hsync  (S_hdmi_out_hsync),
        .I_vid_vsync  (S_hdmi_out_vsync),
        .I_vid_data   (S_hdmi_out_data ),
        .I_det_de     (S_conf_de        ),
        .I_det_hsync  (S_conf_hsync     ),
        .I_det_vsync  (S_conf_vsync     ),
        .I_det_data   (S_hdmi_tx_data_conf ),
        .O_de         (S_quad_ode      ),
        .O_hsync      (S_quad_ohsync   ),
        .O_vsync      (S_quad_ovsync   ),
        .O_data       (S_quad_odata    ),
        .O_view_mode  (S_view_mode_hdmi),
        .O_quad_mode  (S_quad_mode_hdmi),
        .O_process_quad (S_process_quad_hdmi)
    );

    // ---- K2手动抓拍: 对HDMI最终显示流每5列/6行采样RGB565缩略图 ----
    // 长按触发后等下一帧VSYNC边界；暂停时读到的仍是冻结显示帧。
    reg S_manual_vsync_d, S_manual_de_d;
    reg S_manual_wait_frame, S_manual_capture_active;
    reg S_manual_first_line, S_manual_line_sample;
    reg [2:0] S_manual_line_phase, S_manual_x_phase;
    reg [14:0] S_manual_sample_count;
    reg [2:0] S_manual_pack_count;
    reg [127:0] S_manual_pack_data;
    wire S_manual_vsync_rise = S_quad_ovsync && !S_manual_vsync_d;
    wire S_manual_de_rise = S_quad_ode && !S_manual_de_d;
    wire S_manual_line_sample_now = S_manual_de_rise ?
                                      (S_manual_first_line || (S_manual_line_phase == 3'd5)) :
                                      S_manual_line_sample;
    wire S_manual_pixel_sample = S_manual_capture_active && S_quad_ode &&
                                  S_manual_line_sample_now && (S_manual_x_phase == 3'd0);
    wire [15:0] S_manual_pixel565 = {S_quad_odata[23:19],
                                     S_quad_odata[15:10],
                                     S_quad_odata[7:3]};
    assign S_manual_snap_fifo_we = S_manual_pixel_sample && !S_manual_snap_fifo_full;
    assign S_manual_snap_fifo_din = {S_manual_pixel565, S_manual_pack_data[111:0]};

    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_manual_vsync_d <= 1'b0;
            S_manual_de_d <= 1'b0;
            S_manual_wait_frame <= 1'b0;
            S_manual_capture_active <= 1'b0;
            S_manual_first_line <= 1'b1;
            S_manual_line_sample <= 1'b0;
            S_manual_line_phase <= 3'd0;
            S_manual_x_phase <= 3'd0;
            S_manual_sample_count <= 15'd0;
            S_manual_pack_count <= 3'd0;
            S_manual_pack_data <= 128'd0;
        end else begin
            S_manual_vsync_d <= S_quad_ovsync;
            S_manual_de_d <= S_quad_ode;

            if(S_key2_long && !S_snap_busy_hdmi) begin
                S_manual_wait_frame <= 1'b1;
                S_manual_capture_active <= 1'b0;
                S_manual_sample_count <= 15'd0;
                S_manual_pack_count <= 3'd0;
                S_manual_pack_data <= 128'd0;
            end

            if(S_manual_wait_frame && S_manual_vsync_rise) begin
                S_manual_wait_frame <= 1'b0;
                S_manual_capture_active <= 1'b1;
                S_manual_first_line <= 1'b1;
                S_manual_line_sample <= 1'b0;
                S_manual_line_phase <= 3'd0;
                S_manual_sample_count <= 15'd0;
                S_manual_pack_count <= 3'd0;
                S_manual_pack_data <= 128'd0;
            end

            if(!S_quad_ode) begin
                S_manual_x_phase <= 3'd0;
            end else if(S_manual_de_rise) begin
                S_manual_x_phase <= 3'd1;
                if(S_manual_first_line) begin
                    S_manual_first_line <= 1'b0;
                    S_manual_line_phase <= 3'd0;
                    S_manual_line_sample <= 1'b1;
                end else if(S_manual_line_phase == 3'd5) begin
                    S_manual_line_phase <= 3'd0;
                    S_manual_line_sample <= 1'b1;
                end else begin
                    S_manual_line_phase <= S_manual_line_phase + 3'd1;
                    S_manual_line_sample <= 1'b0;
                end
            end else if(S_manual_x_phase == 3'd4) begin
                S_manual_x_phase <= 3'd0;
            end else begin
                S_manual_x_phase <= S_manual_x_phase + 3'd1;
            end

            if(S_manual_pixel_sample && !S_manual_snap_fifo_full) begin
                S_manual_sample_count <= S_manual_sample_count + 15'd1;
                S_manual_pack_data[S_manual_pack_count*16 +: 16] <= S_manual_pixel565;
                if(S_manual_pack_count == 3'd7)
                    S_manual_pack_count <= 3'd0;
                else
                    S_manual_pack_count <= S_manual_pack_count + 3'd1;

                if(S_manual_sample_count == 15'd30719)
                    S_manual_capture_active <= 1'b0;
            end
        end
    end

    w128_d512_fifo u_manual_snap_fifo(
        .rst        (~S_hdmi_rst_n),
        .clkw       (S_hdmi_pixel_clk),
        .clkr       (S_csi_rx_clk),
        .we         (S_manual_snap_fifo_we),
        .di         (S_manual_snap_fifo_din),
        .re         (S_manual_snap_fifo_re),
        .dout       (S_manual_snap_fifo_dout),
        .valid      (S_manual_snap_fifo_valid),
        .full_flag  (S_manual_snap_fifo_full),
        .empty_flag (S_manual_snap_fifo_empty),
        .afull      (),
        .aempty     (),
        .wrusedw    (S_manual_snap_fifo_wrusedw),
        .rdusedw    (S_manual_snap_fifo_rdusedw)
    );

    hdmi_tx u_hdmi_tx(
        .I_pixel_clk        ( S_hdmi_pixel_clk  ),
        .I_serial_clk       ( S_hdmi_serial_clk ),
        .I_rst              ( ~S_hdmi_rst_n     ),
        .I_key_in           ( 1'b0              ),
        .I_edid_read_trig   ( 1'b0              ),
        .O_edid_read_valid  (                   ),
        .O_edid_read_data   (                   ),
        .I_video_rgb_enable ( 1'b1              ),
        .I_video_in_vs      ( S_quad_ovsync      ),
        .I_video_in_de      ( S_quad_ode         ),
        .I_video_in_user    ( 1'b0              ),
        .I_video_in_valid   ( 1'b0              ),
        .I_video_in_last    ( 1'b0              ),
        .O_video_in_ready   (                   ),
        .I_video_in_data    ( S_quad_odata       ),
        .I_audio_valid      ( 1'b0              ),
        .I_audio_left_data  ( 24'd0             ),
        .I_audio_right_data ( 24'd0             ),
        .I_i2s_BCLK         ( 1'b0              ),
        .I_i2s_LRCK         ( 1'b0              ),
        .I_i2s_DOUT         ( 1'b0              ),
        .O_ddc_scl          (                   ),
        .O_hdmi_clk_p       (                   ),
        .O_hdmi_tx_p        (                   ),
        .O_tmds_ch0_p       ( O_tmds_ch0_p      ),
        .O_tmds_ch1_p       ( O_tmds_ch1_p      ),
        .O_tmds_ch2_p       ( O_tmds_ch2_p      ),
        .O_tmds_clk_p       ( O_tmds_clk_p      )
    );


    
endmodule
