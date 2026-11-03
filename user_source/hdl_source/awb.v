/*****************************************************************
Company : MiLianKe Electronic Technology Co., Ltd.
WebSite:https://www.milianke.com
TechWeb:https://www.uisrc.com
tmall-shop:https://milianke.tmall.com
jd-shop:https://milianke.jd.com
taobao-shop: https://milianke.taobao.com
Description: 
The reference demo provided by Milianke is only used for learning. 
We cannot ensure that the demo itself is free of bugs, so users 
should be responsible for the technical problems and consequences
caused by the use of their own products.
@Author      :   XiaoQingquan 
@Time        :   2024/10 
version:     :   
@Description :   在原基础上删去了一些赘余的代码，并且优化了时序，加入BRAM IP核，
                 让从除法器出来的每一个数据都可以对上相应的像素数据进行增益，使
                 得它的适用行更加广泛了。
*****************************************************************/
module awb  #(   
    parameter IMG_HEIGHT = 1080,
    parameter IMG_WIDTH  = 1920
) 
(
    input                   I_clk  ,
    input                   I_rst_n,


    input                   I_tlast  ,
    input                   I_tuser  ,
    input [95:0]            I_tdata  ,
    input                   I_tvalid , 
    output                  I_tready ,

    output                  O_tlast  ,
    output                  O_tuser  ,
    output [95:0]           O_tdata  ,
    output                  O_tvalid ,
    input                   O_tready
);

/*****************************************************************
                          RGB565תRGB888                                        
*****************************************************************/

    // //RGB565??????888
    // wire [4:0]  rgb565_r   = I_st_data[11+:5];
    // wire [5:0]  rgb565_g   = I_st_data[5+:6];
    // wire [4:0]  rgb565_b   = I_st_data[0+:5];
    
    // wire [23:0] rgb888     = {{rgb565_r[4:0],rgb565_r[2:0]},{rgb565_g[5:0],rgb565_g[1:0]},{rgb565_b[4:0],rgb565_b[2:0]}};
    
    // wire [7:0]  rgb888_r   = rgb888[16+:8];
    // wire [7:0]  rgb888_g   = rgb888[8+:8] ;
    // wire [7:0]  rgb888_b   = rgb888[0+:8] ;
    
  /*****************************************************************
                          中间信号                                        
*****************************************************************/


  reg         I_tvalid_r0,I_tvalid_r1;
  reg  [95:0] I_tdata_d0,I_tdata_d1;


  //累加RGB通道的�??
  reg  [31:0]         sum_r[0:3];
  reg  [31:0]         sum_g[0:3];
  reg  [31:0]         sum_b[0:3];

  
  wire  [95:0]        data_d0 ;
  wire                valid_d0;
  wire                I_tuser_r0;    

  reg                 I_tuser_r1;

  reg   [95:0]        data_d1 ;
  reg                 valid_d1;
  reg   [9:0]         rgb_sum[0:3];//每个像素点RGB的和
  /*****************************************************************
                            平均值计�??                                      
*****************************************************************/

// 对输入数据和有效信号进行寄存，形成流水线
always @(posedge I_clk or negedge I_rst_n) begin
    if (!I_rst_n) begin
        I_tdata_d0  <= 0;
        I_tdata_d1  <= 0;
        {I_tvalid_r0,I_tvalid_r1} <= 0;
    end else begin
        I_tdata_d0  <= I_tdata   ;
        I_tdata_d1  <= I_tdata_d0;
        I_tvalid_r0 <= I_tvalid;
        I_tvalid_r1 <= I_tvalid_r0;
    end
end

// 将输入数据拆分为4个像素的RGB分量
wire [7:0] rgb888_r_d[0:3];
wire [7:0] rgb888_g_d[0:3];
wire [7:0] rgb888_b_d[0:3];

assign  rgb888_r_d[0] = I_tdata[16+:8];
assign  rgb888_g_d[0] = I_tdata[8+:8];
assign  rgb888_b_d[0] = I_tdata[0+:8];


// 将输入数据拆分为4个像素的RGB分量
wire [7:0] rgb888_r_d0[0:3];
wire [7:0] rgb888_g_d0[0:3];
wire [7:0] rgb888_b_d0[0:3];

assign  rgb888_r_d0[0] = I_tdata_d0[16+:8];
assign  rgb888_g_d0[0] = I_tdata_d0[8+:8];
assign  rgb888_b_d0[0] = I_tdata_d0[0+:8];


        ///削弱白色
        always @(posedge I_clk or negedge I_rst_n) begin
            if(!I_rst_n) begin
                rgb_sum[0] <= 0;
            end
            else if(I_tvalid)begin
                rgb_sum[0] <= rgb888_r_d[0] + rgb888_g_d[0] + rgb888_b_d[0];
            end
        end

        // 对每个通道的值进行累加，用于计算平均值
        always @(posedge I_clk or negedge I_rst_n) begin
            if (!I_rst_n || I_tuser) begin
                // 复位或行开始信号有效时，清零累加器
                sum_r[0] <= 1;
                sum_g[0] <= 1;
                sum_b[0] <= 1;
            end else if (I_tvalid_r0 && (rgb_sum[0] < 720)) begin
                // 当输入有效时，累加RGB分量的值
                sum_r[0] <= sum_r[0] + rgb888_r_d0[0];
                sum_g[0] <= sum_g[0] + rgb888_g_d0[0];
                sum_b[0] <= sum_b[0] + rgb888_b_d0[0];
            end
        end

  /*****************************************************************
                           计算增益                                       
*****************************************************************/

  wire [47:0] dout_R[3:0];
  wire [47:0] dout_B[3:0];

  wire done[3:0];

  wire [47:0] in_r_x1;
  wire [47:0] in_b_x1;
  assign in_r_x1 = dout_R[0][47:0];
  assign in_b_x1 = dout_B[0][47:0];



/*****************************************************************
                   IP 调用                                    
*****************************************************************/
    //为了保证合理的延迟，这里使用了除法器

      divider div_gen_G_R_r1 (
          .clk(I_clk),  // input wire aclk
          .rst(!I_rst_n),
          .start(I_tvalid_r1),
          .denominator(sum_r[0]),  // input wire [31 : 0] s_axis_divisor_tdata
          .numerator({sum_g[0],16'd0}),  // input wire [47 : 0] s_axis_dividend_tdata
          .quotient(dout_R[0]),  // output wire [79 : 0] m_axis_dout_tdata
          .done(done[0])
      );

      divider divide_G_B_r1 (
          .clk(I_clk),  // input wire aclk
          .rst(!I_rst_n),
          .start(I_tvalid_r1),
          .denominator(sum_b[0]),  // input wire [31 : 0] s_axis_divisor_tdata
          .numerator({sum_g[0],16'd0}),  // input wire [47 : 0] s_axis_dividend_tdata
          .quotient(dout_B[0]),  // output wire [79 : 0] m_axis_dout_tdata
          .done()
      );

     
    signal_delay #(
        .IMG_HEIGHT   (IMG_HEIGHT),
        .IMG_WIDTH    (IMG_WIDTH ),
        .DATA_WIDTH   (96  ),
        .DELAY_CYCLE  (50)
    )signal_delay_d (
        /*input                       */ .I_clk  (I_clk  ),
        /*input                       */ .I_rst_n(I_rst_n),
        /*input                       */ .I_tuser(I_tuser),  //synthesis keep 
        /*input  wire                 */ .I_valid(I_tvalid_r1),  //synthesis keep 
        /*input  wire [DATA_WIDTH-1:0]*/ .I_data (I_tdata_d1 ),   //synthesis keep 
        /*output wire                 */ .O_valid(valid_d0   ),  //synthesis keep 
        /*output wire [DATA_WIDTH-1:0]*/ .O_data (data_d0    ),   //synthesis keep 
        /*output wire                 */ .O_tuser(I_tuser_r0 )
    );

    

/*****************************************************************
                            ��������                                      
*****************************************************************/

    ////信号同步
    always @(posedge I_clk or negedge I_rst_n) begin
      if (!I_rst_n) begin
          valid_d1 <= 0;
          data_d1  <= 0;
          I_tuser_r1 <= 0;
      end else begin
          valid_d1 <= valid_d0;
          data_d1  <= data_d0 ;
          I_tuser_r1 <= I_tuser_r0;
      end
    end
/*****************************************************************
                           ��������                                       
*****************************************************************/

    reg [27:0] R_new_r1;
    reg [27:0] G_new_r1;
    reg [27:0] B_new_r1;

    reg [27:0] R_new_r2;
    reg [27:0] G_new_r2;
    reg [27:0] B_new_r2;

    reg [27:0] R_new_r3;
    reg [27:0] G_new_r3;
    reg [27:0] B_new_r3;

    reg [27:0] R_new_r4;
    reg [27:0] G_new_r4;
    reg [27:0] B_new_r4;


    wire   [7:0] rgb888_r_d1[0:3];
    wire   [7:0] rgb888_g_d1[0:3];
    wire   [7:0] rgb888_b_d1[0:3];

    assign  rgb888_r_d1[0] = data_d1[16+:8];
    assign  rgb888_g_d1[0] = data_d1[8+:8];
    assign  rgb888_b_d1[0] = data_d1[0+:8];

    assign  rgb888_r_d1[1] = data_d1[40+:8];
    assign  rgb888_g_d1[1] = data_d1[32+:8];
    assign  rgb888_b_d1[1] = data_d1[24+:8];

    assign  rgb888_r_d1[2] = data_d1[64+:8];
    assign  rgb888_g_d1[2] = data_d1[56+:8];
    assign  rgb888_b_d1[2] = data_d1[48+:8];

    assign  rgb888_r_d1[3] = data_d1[88+:8];
    assign  rgb888_g_d1[3] = data_d1[80+:8];
    assign  rgb888_b_d1[3] = data_d1[72+:8];

    // 1280x720 two-dimensional white-field correction:
    reg  [11:0] pixel_x;
    reg  [9:0]  pixel_y;
    wire [11:0] pixel_x_base;
    wire [11:0] pixel_x_1;
    wire [11:0] pixel_x_2;
    wire [11:0] pixel_x_3;
    wire [9:0]  pixel_y_base;
    wire [19:0] rb_gain_0;
    wire [19:0] rb_gain_1;
    wire [19:0] rb_gain_2;
    wire [19:0] rb_gain_3;

    assign pixel_x_base = I_tuser_r1 ? 12'd0 : pixel_x;
    assign pixel_x_1 = pixel_x_base + 12'd1;
    assign pixel_x_2 = pixel_x_base + 12'd2;
    assign pixel_x_3 = pixel_x_base + 12'd3;
    assign pixel_y_base = I_tuser_r1 ? 10'd0 : pixel_y;

    function [19:0] rb_gain_at_xy;
        input [11:0] x;
        input [9:0] y;
        reg [11:0] dx;
        reg [11:0] dy;
        reg [11:0] dy_scaled;
        reg [11:0] distance;
        reg [11:0] ramp;
        reg [7:0] center_x_weight;
        reg [7:0] lower_y_weight;
        reg [7:0] side_x_weight;
        reg [7:0] upper_y_weight;
        reg [7:0] green_zone_weight;
        reg [7:0] red_zone_weight;
        reg [19:0] base_gain;
        reg [19:0] gain_delta;
        reg [19:0] green_correction;
        reg [19:0] red_correction;
        begin
            dx = (x >= 12'd640) ? (x - 12'd640) : (12'd640 - x);
            dy = (y >= 10'd360) ? (y - 10'd360) : (10'd360 - y);
            dy_scaled = dy + (dy >> 1) + (dy >> 2);
            distance = (dx > dy_scaled) ? dx : dy_scaled;

            if (distance >= 12'd640) begin
                base_gain = 20'h12000;
            end else begin
                ramp = 12'd640 - distance;
                gain_delta = ramp * 20'd25;
                base_gain = 20'h12000 + gain_delta;
            end

            center_x_weight = 8'd0;
            if ((x > 12'd256) && (x < 12'd512))
                center_x_weight = x - 12'd256;
            else if ((x >= 12'd512) && (x <= 12'd768))
                center_x_weight = 8'hff;
            else if ((x > 12'd768) && (x < 12'd1024))
                center_x_weight = 12'd1024 - x;

            lower_y_weight = 8'd0;
            if ((y > 10'd256) && (y < 10'd512))
                lower_y_weight = y - 10'd256;
            else if (y >= 10'd512)
                lower_y_weight = 8'hff;

            side_x_weight = 8'd0;
            if (x <= 12'd128)
                side_x_weight = 8'hff;
            else if (x < 12'd384)
                side_x_weight = 12'd384 - x;
            else if ((x > 12'd896) && (x < 12'd1152))
                side_x_weight = x - 12'd896;
            else if (x >= 12'd1152)
                side_x_weight = 8'hff;

            upper_y_weight = 8'd0;
            if (y <= 10'd128)
                upper_y_weight = 8'hff;
            else if (y < 10'd384)
                upper_y_weight = 10'd384 - y;

            green_zone_weight = (center_x_weight < lower_y_weight) ?
                                center_x_weight : lower_y_weight;
            red_zone_weight = (side_x_weight < upper_y_weight) ?
                              side_x_weight : upper_y_weight;

            green_correction = green_zone_weight * 20'd32;
            red_correction = red_zone_weight * 20'd32;
            rb_gain_at_xy = base_gain + green_correction - red_correction;
        end
    endfunction

    assign rb_gain_0 = rb_gain_at_xy(pixel_x_base, pixel_y_base);
    assign rb_gain_1 = rb_gain_at_xy(pixel_x_1, pixel_y_base);
    assign rb_gain_2 = rb_gain_at_xy(pixel_x_2, pixel_y_base);
    assign rb_gain_3 = rb_gain_at_xy(pixel_x_3, pixel_y_base);

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            pixel_y <= 10'd0;
        end else if (I_tuser_r1) begin
            pixel_y <= 10'd0;
        end else if (valid_d2 && !valid_d1) begin
            if (pixel_y >= (IMG_HEIGHT - 1))
                pixel_y <= 10'd0;
            else
                pixel_y <= pixel_y + 10'd1;
        end
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            pixel_x <= 12'd0;
        end else if (I_tuser_r1) begin
            pixel_x <= 12'd4;
        end else if (valid_d1) begin
            if (pixel_x >= (IMG_WIDTH - 4))
                pixel_x <= 12'd0;
            else
                pixel_x <= pixel_x + 12'd4;
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            R_new_r1 <= 0; G_new_r1 <= 0; B_new_r1 <= 0;
            R_new_r2 <= 0; G_new_r2 <= 0; B_new_r2 <= 0;
            R_new_r3 <= 0; G_new_r3 <= 0; B_new_r3 <= 0;
            R_new_r4 <= 0; G_new_r4 <= 0; B_new_r4 <= 0;
        end else if (valid_d1) begin
            R_new_r1 <= rb_gain_0 * rgb888_r_d1[0];
            G_new_r1 <= rgb888_g_d1[0];
            B_new_r1 <= rb_gain_0 * rgb888_b_d1[0];

            R_new_r2 <= rb_gain_1 * rgb888_r_d1[1];
            G_new_r2 <= rgb888_g_d1[1];
            B_new_r2 <= rb_gain_1 * rgb888_b_d1[1];

            R_new_r3 <= rb_gain_2 * rgb888_r_d1[2];
            G_new_r3 <= rgb888_g_d1[2];
            B_new_r3 <= rb_gain_2 * rgb888_b_d1[2];

            R_new_r4 <= rb_gain_3 * rgb888_r_d1[3];
            G_new_r4 <= rgb888_g_d1[3];
            B_new_r4 <= rb_gain_3 * rgb888_b_d1[3];
        end
    end
    wire [7:0] R_new_x1;
    wire [7:0] G_new_x1;
    wire [7:0] B_new_x1;

    wire [7:0] R_new_x2;
    wire [7:0] G_new_x2;
    wire [7:0] B_new_x2;

    wire [7:0] R_new_x3;
    wire [7:0] G_new_x3;
    wire [7:0] B_new_x3;

    wire [7:0] R_new_x4;
    wire [7:0] G_new_x4;
    wire [7:0] B_new_x4;

    assign R_new_x1 = (R_new_r1[27:24])?255:R_new_r1[23:16];
    assign G_new_x1 =  G_new_r1;
    assign B_new_x1 = (B_new_r1[27:24])?255:B_new_r1[23:16];

    assign R_new_x2 = (R_new_r2[27:24])?255:R_new_r2[23:16];
    assign G_new_x2 =  G_new_r2;
    assign B_new_x2 = (B_new_r2[27:24])?255:B_new_r2[23:16];

    assign R_new_x3 = (R_new_r3[27:24])?255:R_new_r3[23:16];
    assign G_new_x3 =  G_new_r3;
    assign B_new_x3 = (B_new_r3[27:24])?255:B_new_r3[23:16];

    assign R_new_x4 = (R_new_r4[27:24])?255:R_new_r4[23:16];
    assign G_new_x4 =  G_new_r4;
    assign B_new_x4 = (B_new_r4[27:24])?255:B_new_r4[23:16];
/*****************************************************************
                           ͬ���ź�                                       
*****************************************************************/

    reg valid_d2,I_tuser_r2;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)begin
            valid_d2 <= 0;
            I_tuser_r2 <= 0;
        end
        else begin
            valid_d2 <= valid_d1;
            I_tuser_r2 <= I_tuser_r1;
        end
    end


    assign  O_tlast  = (valid_d1 == 0) && (valid_d2 == 1);   
    assign  O_tuser  = I_tuser ;   
    assign  O_tvalid = valid_d2;

    assign  O_tdata = O_tvalid?{R_new_x4,G_new_x4,B_new_x4,
                       R_new_x3,G_new_x3,B_new_x3,
                       R_new_x2,G_new_x2,B_new_x2,
                       R_new_x1,G_new_x1,B_new_x1}:0;
    // assign O_t_data = {R_new[7-:5],G_new[7-:6],B_new[7-:5]};
    assign  I_tready = O_tready;
 
endmodule



