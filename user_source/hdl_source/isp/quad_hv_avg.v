module quad_hv_avg (
    input            I_clk,
    input            I_rst_n,
    input            I_tvalid,
    input            I_tlast,
    input            I_tuser,
    input      [8:0] I_word_cnt,
    input     [95:0] I_tdata,
    output    [47:0] O_v_pair
);
    wire [23:0] px0 = I_tdata[95:72];
    wire [23:0] px1 = I_tdata[71:48];
    wire [23:0] px2 = I_tdata[47:24];
    wire [23:0] px3 = I_tdata[23:0];

    function [23:0] avg_rgb;
        input [23:0] a;
        input [23:0] b;
        reg [8:0] sum_r;
        reg [8:0] sum_g;
        reg [8:0] sum_b;
        begin
            sum_r = {1'b0,a[23:16]} + {1'b0,b[23:16]} + 9'd1;
            sum_g = {1'b0,a[15:8]}  + {1'b0,b[15:8]}  + 9'd1;
            sum_b = {1'b0,a[7:0]}   + {1'b0,b[7:0]}   + 9'd1;
            avg_rgb = {sum_r[8:1], sum_g[8:1], sum_b[8:1]};
        end
    endfunction

    wire [23:0] h0 = avg_rgb(px0, px1);
    wire [23:0] h1 = avg_rgb(px2, px3);

    reg [23:0] line0 [0:319];
    reg [23:0] line1 [0:319];
    always @(posedge I_clk) begin
        if (I_tvalid) begin
            line0[I_word_cnt] <= h0;
            line1[I_word_cnt] <= h1;
        end
    end

    wire [23:0] prev0 = line0[I_word_cnt];
    wire [23:0] prev1 = line1[I_word_cnt];

    reg have_previous_line;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)
            have_previous_line <= 1'b0;
        else if (I_tuser)
            have_previous_line <= 1'b0;
        else if (I_tlast)
            have_previous_line <= 1'b1;
    end

    wire use_previous = have_previous_line && !I_tuser;
    wire [23:0] v0 = use_previous ? avg_rgb(prev0, h0) : h0;
    wire [23:0] v1 = use_previous ? avg_rgb(prev1, h1) : h1;

    assign O_v_pair = {v0, v1};
endmodule
