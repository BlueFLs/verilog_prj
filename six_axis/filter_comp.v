module filter_comp
(
  input                             I_clk            ,
  input                             I_rst_n          ,
  input                             I_gyro_vld       , //first
  input                             I_acc_vld        , //later
  input        signed    [15:0]     I_acc_x          ,
  input        signed    [15:0]     I_acc_y          ,
  input        signed    [15:0]     I_acc_z          ,
  input                  [15:0]     I_gyro_x         ,

  input                             I_just_acc_en    ,
  input                  [13:0]     I_p_acc          ,
  input                  [13:0]     I_p_gyro         ,

  input                  [8 :0]     I_gyro_x_jitter  ,
  input                  [8 :0]     I_gyro_x_bias    ,
  input                  [8 :0]     I_gyro_x_start   ,
  input                  [28:0]     I_acc_jitter     ,
  input                  [15:0]     I_sec_jud_data   ,

  input        signed    [15:0]     I_gyro_y         ,
  input        signed    [15:0]     I_gyro_z         ,
  output  reg  signed    [19:0]     O_roll_cos_val   ,
  output  reg  signed    [19:0]     O_roll_sin_val   ,
  output  reg                       O_roll_cos_vld   ,
  output  reg                       O_roll_sin_vld
);

localparam signed P_2P15        =  32767    ; // 2**15-1

localparam signed P_P360_M8192  =  2949120  ; // 360*8192
localparam signed P_N360_M8192  = -2949120  ; // -360*8192

localparam signed P_P180_M8192  =  1474560  ; // 180*8192
localparam signed P_N180_M8192  = -1474560  ; // -180*8192

localparam signed P_57P3_M8192  =  469370   ; // 57.29578*8192

localparam signed P_PPI_MM8192  =  210821120; //pi * 8192 * 8192     210821120
localparam signed P_P2PI_MM8192 =  421642240; //2pi * 8192 * 8192    421642240
localparam signed P_NPPI_MM8192 = -210821120; //-pi * 8192 * 8192   -210821120

parameter         P_ST_IDLE     = 3'd1;
parameter         P_ST_JUDE     = 3'd2;
parameter         P_ST_NOT_JUDE = 3'd4;
reg                R_just_acc_en_syn0;
reg                R_just_acc_en_d1;
reg                R_just_acc_en_d2;

wire               W_accy_div_vld;
wire [31:0]        W_accy_div_dout;

wire               W_accz_div_vld;
wire [31:0]        W_accz_div_dout;

reg  [15:0]        R_tany;
reg  [15:0]        R_tanz;
reg  [31:0]        R_tan;

reg                R_tany_vld;
reg                R_tanz_vld;
reg                R_tan_vld;

reg                R_gyro_start_flag;
reg signed [13:0]  R_p_acc;
reg signed [13:0]  R_p_gyro;

wire               W_acc_tan_vld;
wire signed [15:0] W_acc_tan_dout; //2QN

reg                R_acc_tan_vld;
reg                R_acc_tan_vld_d1;
reg                R_acc_tan_vld_d2;
reg                R_acc_tan_nf_flag;
reg                R_acc_tan_vld_d3;
reg                R_acc_tan_vld_d4;
reg  signed [15:0] R_acc_tan_data;
reg  signed [15:0] R_acc_tan_data_d1;
reg  signed [15:0] R_acc_tan_data_d2;
reg  signed [15:0] R_acc_tan_data_d3;
reg  signed [15:0] R_acc_tan_data_d4;
reg  signed [15:0] R_acc_tan_data_d5;

reg  signed [31:0] R_gyro_xa;
reg  signed [31:0] R_gyro_xb;
reg  signed [31:0] R_gyro_xc;

reg                R_gyro_vld;
reg                R_gyro_vld_d1;
reg                R_gyro_vld_d2;
reg                R_gyro_vld_d3;

wire               W_gyro_rad_vld;
wire signed [31:0] W_gyro_rad_dout;

reg  signed [15:0] R_gyro_x_t;

reg  signed [15:0] R_gyro_rad_data;
reg  signed [28:0] R_acc_ms_init_t;
reg  signed [28:0] R_acc_ms_init_t_d1;
reg  signed [29:0] R_acc_ms_init_sub;
reg  signed [28:0] R_acc_ms_init;

wire signed [28:0] R_acc_ms;
wire signed [28:0] R_gyro_ms;

reg  signed [29:0] R_deg_val_t0;
reg  signed [30:0] R_deg_val_t1;
reg  signed [30:0] R_deg_val_t2;
reg  signed [15:0] R_deg_val;

reg                R_deg_vld ;
wire               W_sincos_dout_vld;
wire signed [47:0] W_sincos_dout;

reg         [15:0] R_deg_abs_val;
reg         [15:0] R_acc_tan_abs_data;
reg         [15:0] R_deg_sub_pos_val;
reg                R_deg_sec_en;
reg                R_deg_sec_flag;
reg                R_deg_sec_flag_d1;
reg         [20:0] R_deg_sec_cnt;
reg         [2 :0] R_cst;
reg         [2 :0] R_nst;
wire               W_just_acc_en_f;
wire               W_deg_sec_flag_f;
reg         [5:0]  R_start_dly_cnt;
reg         [2:0]  R_not_jude_cnt;

always @(posedge I_clk)
begin
  if(!I_rst_n)
    begin
      R_just_acc_en_syn0 <= 1'b0;
      R_just_acc_en_d1   <= 1'b0;
      R_just_acc_en_d2   <= 1'b0;
    end
  else
    begin
      R_just_acc_en_syn0 <= I_just_acc_en;
      R_just_acc_en_d1   <= R_just_acc_en_syn0;
      R_just_acc_en_d2   <= R_just_acc_en_d1;
    end
end

//--data normalization
div_fliter_16s_16s u_acc_y (
  .aclk                  (I_clk                 ),   // input wire aclk
  .s_axis_divisor_tvalid (1'b1                  ),   // input wire s_axis_divisor_tvalid
  .s_axis_divisor_tdata  (P_2P15                ),   // 2**15 input wire [15 : 0] s_axis_divisor_tdata
  .s_axis_dividend_tvalid(I_acc_vld             ),   // input wire s_axis_dividend_tvalid
  .s_axis_dividend_tdata (I_acc_y               ),   // input wire [15 : 0] s_axis_dividend_tdata
  .m_axis_dout_tvalid    (W_accy_div_vld        ),   // output wire m_axis_dout_tvalid
  .m_axis_dout_tdata     (W_accy_div_dout       )    // [30:15],[14:0],fix15_14 output wire [31 : 0] m_axis_dout_tdata
);

div_fliter_16s_16s u_acc_z (
  .aclk                  (I_clk                 ),   // input wire aclk
  .s_axis_divisor_tvalid (1'b1                  ),   // input wire s_axis_divisor_tvalid
  .s_axis_divisor_tdata  (P_2P15                ),   // input wire [15 : 0] s_axis_divisor_tdata
  .s_axis_dividend_tvalid(I_acc_vld             ),   // input wire s_axis_dividend_tvalid
  .s_axis_dividend_tdata (I_acc_z               ),   // input wire [15 : 0] s_axis_dividend_tdata
  .m_axis_dout_tvalid    (W_accz_div_vld        ),   // output wire m_axis_dout_tvalid
  .m_axis_dout_tdata     (W_accz_div_dout       )    // [30:15], [14:0], fix15_14output wire [31 : 0] m_axis_dout_tdata
);

//--

always @(posedge I_clk)
begin
  if(!I_rst_n)
    R_tany <= 16'd0;
  else if(W_accy_div_vld)
    begin
      if(W_accy_div_dout[16])
        R_tany <= 16'hc000;         // -1
      else
        R_tany <= { W_accy_div_dout[14], W_accy_div_dout[14], W_accy_div_dout[13:0]};
    end
  else
    R_tany <= R_tany;
end

always @(posedge I_clk)
begin
  if(!I_rst_n)
    R_tanz <= 16'd0;
  else if(W_accz_div_vld)
    begin
      if(W_accz_div_dout[16])
        R_tanz <= 16'hc000;         // -1
      else
        R_tanz <= { W_accz_div_dout[14], W_accz_div_dout[14], W_accz_div_dout[13:0]};
    end
  else
    R_tanz <= R_tanz;
end

always @(posedge I_clk)
begin
  if(!I_rst_n)
    R_tan <= 32'd0;
  else
    R_tan <= {R_tany, R_tanz};
end

always @(posedge I_clk)
begin
  if(!I_rst_n)
    R_tany_vld <= 1'b0;
  else if(R_tan_vld)
    R_tany_vld <= 1'b0;
  else if(W_accy_div_vld)
    R_tany_vld <= 1'b1;
  else
    R_tany_vld <= R_tany_vld;
end
always @(posedge I_clk)
begin
  if(!I_rst_n)
    R_tanz_vld <= 1'b0;
  else if(R_tan_vld)
    R_tanz_vld <= 1'b0;
  else if(W_accz_div_vld)
    R_tanz_vld <= 1'b1;
  else
    R_tanz_vld <= R_tanz_vld;
end

always @(posedge I_clk)
begin
  if(!I_rst_n)
    R_tan_vld <= 1'b0;
  else if(R_tany_vld && R_tanz_vld)
    R_tan_vld <= 1'b1;
  else
    R_tan_vld <= 1'b0;
end

//--- roll = arctan(acc_y/acc_z)

arctan_fliter u_arctan (
  .aclk                   (I_clk                  ),  // input wire aclk
  .s_axis_cartesian_tvalid(R_tan_vld              ),  // input wire s_axis_cartesian_tvalid
  .s_axis_cartesian_tdata (R_tan                  ),  // input wire [31 : 0] s_axis_cartesian_tdata
  .m_axis_dout_tvalid     (W_acc_tan_vld          ),  // output wire m_axis_dout_tvalid
  .m_axis_dout_tdata      (W_acc_tan_dout         )   // 2QN output wire [15 : 0] m_axis_dout_tdata
);

/-- gyro_xa = gyro_xa + gyro_xp(+-1000,32.8lsb/s) * dt(250hz)
//-- gyro_xa = gyro_xa + gyro_x/32.8 * 1/250
//-- gyro_xa = gyro_xa  + gyro_x
//

always @(posedge I_clk)
begin
  if(!I_rst_n)
    R_gyro_x_t <= 16'sd0;
  else if(I_gyro_vld )
    begin
      if(I_gyro_x <= I_gyro_x_jitter)
        R_gyro_x_t <= 16'sd0;
      else if(I_gyro_x >= 65536 - I_gyro_x_jitter)
        R_gyro_x_t <= 16'sd0;
      else
        R_gyro_x_t <= I_gyro_x - I_gyro_x_bias;
    end
  else
    R_gyro_x_t <= R_gyro_x_t;
end

always @(posedge I_clk)
begin
  if(!I_rst_n)
    R_gyro_start_flag <= 1'b0;
  else if(I_gyro_vld)
    begin
      if( (I_gyro_x >= I_gyro_x_start) && (~I_gyro_x[15]))
        R_gyro_start_flag <= 1'b1;
      else if(I_gyro_x <= (65536 - I_gyro_x_start) && (I_gyro_x[15]))
        R_gyro_start_flag <= 1'b1;
      else
        R_gyro_start_flag <= R_gyro_start_flag;
    end
  else begin
    R_gyro_start_flag <= R_gyro_start_flag;
  end
end

always @(posedge I_clk)
begin
  if(!I_rst_n)
    R_p_acc <= 14'sd0;
  else if(!R_gyro_start_flag || R_just_acc_en_d1 || R_deg_sec_flag)
    R_p_acc <= 14'sh1fff;
  else
    R_p_acc <= I_p_acc;
end

always @(posedge I_clk)
begin
  if(!I_rst_n)
    R_p_gyro <= 14'd0;
  else if(!R_gyro_start_flag || R_just_acc_en_d1 || R_deg_sec_flag)
    R_p_gyro <= 14'sh0;
  else
    R_p_gyro <= I_p_gyro;
end

always @(posedge I_clk)
begin
  if(!I_rst_n)
    R_gyro_xa <= 32'sd0;
  else if(!R_gyro_start_flag || R_just_acc_en_d1 || R_deg_sec_flag)
    R_gyro_xa <= 32'sd0;
  else if(R_gyro_xa >= P_P360_M8192 )
    R_gyro_xa <= R_gyro_xa - P_P360_M8192;
  else if(R_gyro_xa <= P_N360_M8192 )
    R_gyro_xa <= R_gyro_xa + P_P360_M8192;
  else if(R_gyro_vld)
    R_gyro_xa <= R_gyro_xa + R_gyro_x_t ;
  else
    R_gyro_xa <= R_gyro_xa;
end

always @(posedge I_clk)
begin
  if(!I_rst_n)
    R_gyro_xb <= 32'sd0;
  else if(R_gyro_xa >= P_P180_M8192 )
    R_gyro_xb <= R_gyro_xa - P_P360_M8192;
  else if(R_gyro_xa <= P_N180_M8192 )
    R_gyro_xb <= P_P360_M8192 + R_gyro_xa;
  else
    R_gyro_xb <= R_gyro_xa;
end

always @(posedge I_clk)
begin
  if(!I_rst_n)
    R_gyro_xc <= 'sd0;
  else
    R_gyro_xc <= R_gyro_xb;
end

always @(posedge I_clk)
begin
  if(!I_rst_n)
    begin
      R_gyro_vld    <= 1'b0;
      R_gyro_vld_d1 <= 1'b0;
      R_gyro_vld_d2 <= 1'b0;
      R_gyro_vld_d3 <= 1'b0;
    end
  else
    begin
      R_gyro_vld    <= I_gyro_vld;
      R_gyro_vld_d1 <= R_gyro_vld;
      R_gyro_vld_d2 <= R_gyro_vld_d1;
      R_gyro_vld_d3 <= R_gyro_vld_d2;
    end
end

div_fliter_32s_32s u_gyro_rad (
  .aclk                  (I_clk                 ),     // input wire aclk
  .s_axis_divisor_tvalid (1'b1                  ),     // input wire s_axis_divisor_tvalid
  .s_axis_divisor_tdata  (P_57P3_M8192          ),     // 57.29578*8192 input wire [31 : 0] s_axis_divisor_tdata
  .s_axis_dividend_tvalid(R_gyro_vld_d3         ),     // input wire s_axis_dividend_tvalid
  .s_axis_dividend_tdata (R_gyro_xc             ),     // input wire [31 : 0] s_axis_dividend_tdata
  .m_axis_dout_tvalid    (W_gyro_rad_vld        ),     // output wire m_axis_dout_tvalid
  .m_axis_dout_tdata     (W_gyro_rad_dout       )      // [45:14] [29:14],[13:0] fix14_13 output wire [31 : 0] m_axis_dout_tdata
);

//-- val = P_ACC * acc + P_GYRO * gyro

always @(posedge I_clk)
begin
  if(!I_rst_n)
    R_acc_tan_data <= 16'sd0;
  else if(W_acc_tan_vld )
    R_acc_tan_data <= W_acc_tan_dout;
  else
    R_acc_tan_data <= R_acc_tan_data;
end


mult_fliter_18s_18s u_acc_ms (
  .CLK(I_clk         ),  // input wire CLK
  .A  (R_acc_tan_data),  // input wire [17 : 0] A
  .B  (R_p_acc       ),  // input wire [17 : 0] B
  .P  (R_acc_ms      )   // output wire [35 : 0] P
);

always @(posedge I_clk)
begin
  if(!I_rst_n)
    R_acc_ms_init_t <= 29'sd0;
  else if(!R_gyro_start_flag || R_just_acc_en_d1 || R_deg_sec_flag)
    R_acc_ms_init_t <= R_acc_ms ;
  else
    R_acc_ms_init_t <= R_acc_ms_init_t;
end

always @(posedge I_clk)
begin
  if(!I_rst_n)
    begin
      R_acc_ms_init_t_d1 <= 29'sd0;
      R_acc_ms_init_t_d2 <= 29'sd0;
    end
  else
    begin
      R_acc_ms_init_t_d1 <= R_acc_ms_init_t;
      R_acc_ms_init_t_d2 <= R_acc_ms_init_t_d1;
    end
end

always @(posedge I_clk)
begin
  if(!I_rst_n)
    R_acc_ms_init_sub <= 30'sd0;
  else if(R_acc_ms_init_t >= R_acc_ms_init_t_d1)
    R_acc_ms_init_sub <= R_acc_ms_init_t - R_acc_ms_init_t_d1;
  else
    R_acc_ms_init_sub <= R_acc_ms_init_t_d1 - R_acc_ms_init_t;
end

always @(posedge I_clk)
begin
  if(!I_rst_n)
    R_acc_tan_nf_flag <= 1'b0;
  else if(R_acc_tan_vld_d4)
    R_acc_tan_nf_flag <= 1'b1;
  else
    R_acc_tan_nf_flag <= R_acc_tan_nf_flag;
end

always @(posedge I_clk)
begin
  if(!I_rst_n)
    R_acc_ms_init <= 29'sd0;
  else if(!R_acc_tan_nf_flag)
    R_acc_ms_init <= R_acc_ms_init_t;
  else if(!R_gyro_start_flag)
    begin
      if(R_acc_tan_vld_d2 && R_acc_tan_nf_flag)
        begin
          if(R_acc_ms_init_sub >= I_acc_jitter)
            R_acc_ms_init <= R_acc_ms_init_t_d1;
          else
            R_acc_ms_init <= R_acc_ms_init;
        end
      else
        R_acc_ms_init <= R_acc_ms_init;
    end
  else
    R_acc_ms_init <= R_acc_ms_init_t;
end

always @(posedge I_clk)
begin
  if(!I_rst_n)
    R_gyro_rad_data <= 16'sd0;
  else if(W_gyro_rad_vld)
    begin
      case({W_gyro_rad_dout[16], W_gyro_rad_dout[13]})
        2'b11:
          R_gyro_rad_data <= {1'b1,W_gyro_rad_dout[15]&W_gyro_rad_dout[14],(~W_gyro_rad_dout[14]), W_gyro_rad_dout[12:0]};
        2'b10:
          R_gyro_rad_data <= {1'b1, W_gyro_rad_dout[15:14], W_gyro_rad_dout[12:0]};
        2'b01:
          R_gyro_rad_data <= {1'b1, 2'b11, W_gyro_rad_dout[12:0]};
        2'b00:
          R_gyro_rad_data <= {1'b0, W_gyro_rad_dout[15:14], W_gyro_rad_dout[12:0]};
        default:
          R_gyro_rad_data <= 16'hdead;
      endcase
    end
  else
    R_gyro_rad_data <= R_gyro_rad_data;
end

mult_fliter_18s_18s u_gyro_ms (
  .CLK(I_clk          ),      // input wire CLK
  .A  (R_gyro_rad_data),      // input wire [17 : 0] A
  .B  (R_p_gyro       ),      // input wire [17 : 0] B
  .P  (R_gyro_ms      )       // output wire [35 : 0] P
);

always @(posedge I_clk)
begin
  if(!I_rst_n)
    R_deg_val_t0 <= 30'sd0;
  else if(!R_gyro_start_flag || R_just_acc_en_d1 || R_deg_sec_flag)
    R_deg_val_t0 <= R_acc_ms_init;
  else
    R_deg_val_t0 <= R_acc_ms + R_gyro_ms;
end

always @(posedge I_clk)
begin
  if(!I_rst_n)
    R_deg_val_t1 <= 31'sd0;
  else if(!R_gyro_start_flag || R_just_acc_en_d1 || R_deg_sec_flag)
    R_deg_val_t1 <= R_acc_ms_init;
  else
    R_deg_val_t1 <= R_deg_val_t0 + R_acc_ms_init;
end

always @(posedge I_clk)
begin
  if(!I_rst_n)
    R_deg_val_t2 <= 31'sd0;
  else if(R_deg_val_t1 > P_PPI_MM8192)
    R_deg_val_t2 <= R_deg_val_t1 - P_P2PI_MM8192;
  else if(R_deg_val_t1 < P_NPPI_MM8192)
    R_deg_val_t2 <= R_deg_val_t1 + P_P2PI_MM8192;
  else
    R_deg_val_t2 <= R_deg_val_t1;
end

always @(posedge I_clk)
begin
  if(!I_rst_n)
    R_deg_val <= 16'sd0;
  else
    R_deg_val <= (R_deg_val_t2 >>> 13);
end

//--self correcting

always @(posedge I_clk)
begin
  if(!I_rst_n)
    begin
      R_acc_tan_data_d1 <= 16'sd0;
      R_acc_tan_data_d2 <= 16'sd0;
      R_acc_tan_data_d3 <= 16'sd0;
      R_acc_tan_data_d4 <= 16'sd0;
      R_acc_tan_data_d5 <= 16'sd0;
    end
  else
    begin
      R_acc_tan_data_d1 <= R_acc_tan_data;
      R_acc_tan_data_d2 <= R_acc_tan_data_d1;
      R_acc_tan_data_d3 <= R_acc_tan_data_d2;
      R_acc_tan_data_d4 <= R_acc_tan_data_d3;
      R_acc_tan_data_d5 <= R_acc_tan_data_d4;
    end
end

always @(posedge I_clk)
begin
  if(!I_rst_n)
    R_start_dly_cnt <= 6'd0;
  else if(R_gyro_start_flag)
    begin
      if(&R_start_dly_cnt)
        R_start_dly_cnt <= R_start_dly_cnt;
      else
        R_start_dly_cnt <= R_start_dly_cnt + 6'd1;
    end
  else
    R_start_dly_cnt <= R_start_dly_cnt;
end

assign W_just_acc_en_f  = ~R_just_acc_en_d1 & R_just_acc_en_d2;
assign W_deg_sec_flag_f = ~R_deg_sec_flag   & R_deg_sec_flag_d1;
always @(posedge I_clk)
begin
  if(!I_rst_n)
    R_cst <= P_ST_IDLE;
  else
    R_cst <= R_nst;
end

always @(*)
begin
  case(R_cst)
    P_ST_IDLE:
      begin
        if(R_gyro_start_flag && (&R_start_dly_cnt))
          R_nst = P_ST_JUDE;
        else
          R_nst = P_ST_IDLE;
      end
    P_ST_JUDE:
      begin
        if(W_just_acc_en_f || W_deg_sec_flag_f)
          R_nst = P_ST_NOT_JUDE;
        else
          R_nst = P_ST_JUDE;
      end
    P_ST_NOT_JUDE:
      begin
        if(&R_not_jude_cnt)
          R_nst = P_ST_JUDE;
        else
          R_nst = P_ST_NOT_JUDE;
      end
    default:
      R_nst = P_ST_IDLE;
  endcase
end

always @(posedge I_clk)
begin
  if(!I_rst_n)
    R_not_jude_cnt <= 3'd0;
  else if(R_cst == P_ST_NOT_JUDE)
    R_not_jude_cnt <= R_not_jude_cnt + 3'd1;
  else
    R_not_jude_cnt <= 3'd0;
end

always @(posedge I_clk)
begin
  if(!I_rst_n)
    R_deg_abs_val <= 16'd0;
  else if(R_deg_val[15])
    R_deg_abs_val <= ~R_deg_val[15:0] + 16'd1;
  else
    R_deg_abs_val <= R_deg_val[15:0];
end

always @(posedge I_clk)
begin
  if(!I_rst_n)
    R_acc_tan_abs_data <= 16'd0;
  else if(R_acc_tan_data_d5[15])
    R_acc_tan_abs_data <= ~R_acc_tan_data_d5[15:0] + 16'd1;
  else
    R_acc_tan_abs_data <= R_acc_tan_data_d5[15:0];
end

always @(posedge I_clk)
begin
  if(!I_rst_n)
    R_deg_sub_pos_val <= 16'd0;
  else if(R_deg_abs_val >= R_acc_tan_abs_data)
    R_deg_sub_pos_val <= R_deg_abs_val - R_acc_tan_abs_data;
  else
    R_deg_sub_pos_val <= R_acc_tan_abs_data - R_deg_abs_val;
end

always @(posedge I_clk)
begin
  if(!I_rst_n)
    R_deg_sec_en <= 1'b0;
  else if( (R_deg_sub_pos_val >= I_sec_jud_data) && (R_cst == P_ST_JUDE))
    R_deg_sec_en <= 1'b1;
  else
    R_deg_sec_en <= 1'b0;
end

always @(posedge I_clk)
begin
  if(!I_rst_n)
    R_deg_sec_flag <= 1'b0;
  else if(R_deg_sec_flag && (&R_deg_sec_cnt)) // almost 20ms
    R_deg_sec_flag <= 1'b0;
  else if(R_deg_sec_en)
    R_deg_sec_flag <= 1'b1;
  else
    R_deg_sec_flag <= R_deg_sec_flag;
end

always @(posedge I_clk)
begin
  if(!I_rst_n)
    R_deg_sec_flag_d1 <= 1'b0;
  else
    R_deg_sec_flag_d1 <= R_deg_sec_flag;
end

always @(posedge I_clk)
begin
  if(!I_rst_n)
    R_deg_sec_cnt <= 21'd0;
  else if(R_deg_sec_flag)
    R_deg_sec_cnt <= R_deg_sec_cnt + 21'd1;
  else
    R_deg_sec_cnt <= 21'd0;
end

always @(posedge I_clk)
begin
  if(!I_rst_n)
    begin
      R_acc_tan_vld    <= 1'b0;
      R_acc_tan_vld_d1 <= 1'b0;
      R_acc_tan_vld_d2 <= 1'b0;
      R_acc_tan_vld_d3 <= 1'b0;
      R_acc_tan_vld_d4 <= 1'b0;
      R_deg_vld        <= 1'b0;
    end
  else
    begin
      R_acc_tan_vld    <= W_acc_tan_vld;
      R_acc_tan_vld_d1 <= R_acc_tan_vld;
      R_acc_tan_vld_d2 <= R_acc_tan_vld_d1;
      R_acc_tan_vld_d3 <= R_acc_tan_vld_d2;
      R_acc_tan_vld_d4 <= R_acc_tan_vld_d3;
      R_deg_vld        <= R_acc_tan_vld_d4;
    end
end

sincos_fliter u_sincos
(
  .aclk               (I_clk              ),  // input wire aclk
  .s_axis_phase_tvalid(R_deg_vld          ),  // input wire s_axis_phase_tvalid
  .s_axis_phase_tdata (R_deg_val          ),  // input wire [15 : 0] s_axis_phase_tdata
  .m_axis_dout_tvalid (W_sincos_dout_vld  ),  // output wire m_axis_dout_tvalid
  .m_axis_dout_tdata  (W_sincos_dout      )   // fix20_18 43:24:cos,19:0:sin output wire [47 : 0] m_axis_dout_tdata
);

always @(posedge I_clk)
begin
  if(!I_rst_n)
    begin
      O_roll_cos_vld <= 1'b0;
      O_roll_sin_vld <= 1'b0;
    end
  else
    begin
      O_roll_cos_vld <= W_sincos_dout_vld;
      O_roll_sin_vld <= W_sincos_dout_vld;
    end
end

always @(posedge I_clk)
begin
  if(!I_rst_n)
    begin
      O_roll_sin_val <= 20'sd0;
      O_roll_cos_val <= 20'sd0;
    end
  else if(W_sincos_dout_vld)
    begin
      O_roll_cos_val <= W_sincos_dout[19:0];
      O_roll_sin_val <= W_sincos_dout[43:24];
    end
  else
    begin
      O_roll_cos_val <= O_roll_cos_val;
      O_roll_sin_val <= O_roll_sin_val;
    end
end

endmodule




