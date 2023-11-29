//=====================================================
//                   var_load
//   Describe      : use load/cntvaluein
//                   not use ce/inc
//
//=====================================================//

module idelay_adjust_v2
#(
  parameter  P_STEP       = 1,           // just support 1,2,4,8
  parameter  P_DELAY_TIME = 135000      // default 1ms, adjust according to the actual situation
)
(
  input             I_clk,
  input             I_rst,

  input             I_adjust_en,         // pulse
  input             I_idelayctrl_rdy,
  input             I_align_ok,          // flag, mean SOT was detected, this tap is OK

  output reg        O_en_vtc,
  output reg  [8:0] O_cntval,
  output reg        O_load,
  output reg        O_ce,
  output reg        O_inc
);

parameter  P_STEP_A    = 512;
parameter  P_CIRCU_A   = P_STEP_A / P_STEP;
parameter  P_END_DELAY = 600; // at least 520 clk

parameter  P_ST_IDLE   = 11'd1   ;
parameter  P_ST_ST     = 11'd2   ;
parameter  P_ST_VTCL   = 11'd4   ;
parameter  P_ST_DELAY0 = 11'd8   ;
parameter  P_ST_VAL    = 11'd16  ;
parameter  P_ST_LOAD   = 11'd32  ;
parameter  P_ST_DELAY1 = 11'd64  ;
parameter  P_ST_VTCH   = 11'd128 ;
parameter  P_ST_DELAY2 = 11'd256 ;
parameter  P_ST_JUDG   = 11'd512 ;
parameter  P_ST_END    = 11'd1024;

reg  [10:0]  R_cst;
reg  [10:0]  R_nst;

reg          R_st_idle;
reg          R_st_st;
reg          R_st_vtcl;
reg          R_st_vtch;
reg          R_st_delay0;
reg          R_st_delay1;
reg          R_st_delay2;
reg          R_st_load;
reg          R_st_val;
reg          R_st_judg;
reg          R_st_judg_d1;
reg          R_st_judg_d2;
reg          R_st_end;
reg          R_st_end_d1;

reg          R_adjust_en_d1;
reg          R_adjust_en_d2;
reg          R_adjust_flag;

reg          R_idelayctrl_rdy_d1;
reg          R_idelayctrl_rdy_d2;

reg          R_st;
reg  [4 :0]  R_delay0_cnt;
reg  [17:0]  R_delay1_cnt;
reg  [9 :0]  R_delay2_cnt;

reg  [9 :0]  R_circu_cnt;
reg          R_circu_end;

reg          R_tap_ok;
reg          R_tap_err;
reg          R_tap_err_d1;
reg          R_tap_err_d2;
reg          R_tap_err_d3;

reg  [9 :0]  R_tap_len;
reg  [9 :0]  R_tap_len_d1;

reg          R_ram_len_wea   ;  //high:write, low:read
reg          R_ram_len_wea_d1;
reg          R_ram_len_wea_d2;
reg  [8 :0]  R_ram_len_waddr;
reg  [8 :0]  R_ram_len_wdata;

reg  [8 :0]  R_ram_len_raddr;
wire [8 :0]  W_ram_len_data;
reg  [8 :0]  R_ram_len_data_d1;
reg  [8 :0]  R_max_len;

reg  [8 :0]  R_ram_tap_wdata;
reg  [8 :0]  R_ram_tap_raddr;

reg  [8 :0]  R_tap_val;

wire [8 :0]  W_ram_tap_data;
always @(posedge I_clk)
begin
  if(I_rst)
    begin
      R_adjust_en_d1 <= 1'b0;
      R_adjust_en_d2 <= 1'b0;
    end
  else
    begin
      R_adjust_en_d1 <= I_adjust_en;
      R_adjust_en_d2 <= R_adjust_en_d1;
    end
end

always @(posedge I_clk)
begin
  if(I_rst)
    R_adjust_flag <= 1'b0;
  else if(R_st_st)
    R_adjust_flag <= 1'b0;
  else if(R_adjust_en_d2)
    R_adjust_flag <= 1'b1;
  else
    R_adjust_flag <= R_adjust_flag;
end

always @(posedge I_clk)
begin
  if(I_rst)
    begin
      R_idelayctrl_rdy_d1 <= 1'b0;
      R_idelayctrl_rdy_d2 <= 1'b0;
    end
  else
    begin
      R_idelayctrl_rdy_d1 <= I_idelayctrl_rdy;
      R_idelayctrl_rdy_d2 <= R_idelayctrl_rdy_d1;
    end
end

always @(posedge I_clk)
begin
  if(I_rst)
    begin
      R_st_idle   <= 1'b0;
      R_st_st     <= 1'b0;
      R_st_vtcl   <= 1'b0;
      R_st_vtch   <= 1'b0;
      R_st_delay0 <= 1'b0;
      R_st_delay1 <= 1'b0;
      R_st_delay2 <= 1'b0;
      R_st_load   <= 1'b0;
      R_st_val    <= 1'b0;
      R_st_judg   <= 1'b0;
      R_st_end    <= 1'b0;
    end
  else
    begin
      R_st_idle   <= (R_nst == P_ST_IDLE);
      R_st_st     <= (R_nst == P_ST_ST);
      R_st_vtcl   <= (R_nst == P_ST_VTCL);
      R_st_delay0 <= (R_nst == P_ST_DELAY0);
      R_st_val    <= (R_nst == P_ST_VAL);
      R_st_load   <= (R_nst == P_ST_LOAD);
      R_st_delay1 <= (R_nst == P_ST_DELAY1);
      R_st_vtch   <= (R_nst == P_ST_VTCH);
      R_st_delay2 <= (R_nst == P_ST_DELAY2);
      R_st_judg   <= (R_nst == P_ST_JUDG);
      R_st_end    <= (R_nst == P_ST_END);
    end
end
always @(posedge I_clk)
begin
  if(I_rst)
    R_st_end_d1 <= 1'b0;
  else
    R_st_end_d1 <= R_st_end;
end

always @(posedge I_clk)
begin
  if(I_rst)
    begin
      R_st_judg_d1 <= 1'b0;
      R_st_judg_d2 <= 1'b0;
    end
  else
    begin
      R_st_judg_d1 <= R_st_judg;
      R_st_judg_d2 <= R_st_judg_d1;
    end
end

always @(posedge I_clk)
begin
  if(I_rst)
    R_st <= 1'b0;
  else if(R_st_st)
    R_st <= 1'b0;
  else if(R_adjust_flag && R_idelayctrl_rdy_d2)
    R_st <= 1'b1;
  else
    R_st <= R_st;
end

always @(posedge I_clk)
begin
  if(I_rst)
    R_delay0_cnt <= 5'd0;
  else if(R_st_delay0 || R_st_delay1)
    R_delay0_cnt <= R_delay0_cnt + 5'd1;
  else
    R_delay0_cnt <= 5'd0;
end

always @(posedge I_clk)
begin
  if(I_rst)
    R_delay1_cnt <= 18'd0;
  else if(R_st_delay2)
    R_delay1_cnt <= R_delay1_cnt + 18'd1;
  else
    R_delay1_cnt <= 18'd0;
end

always @(posedge I_clk)
begin
  if(I_rst)
    R_delay2_cnt <= 10'd0;
  else if(R_st_end)
    R_delay2_cnt <= R_delay2_cnt + 10'd1;
  else
    R_delay2_cnt <= 10'd0;
end

always @(posedge I_clk)
begin
  if(I_rst)
    R_circu_cnt <= 10'd0;
  else if(R_st_idle)
    R_circu_cnt <= 10'd0;
  else if(R_st_vtch)
    R_circu_cnt <= R_circu_cnt + 10'd1;
  else
    R_circu_cnt <= R_circu_cnt;
end
always @(posedge I_clk)
begin
  if(I_rst)
    R_cst <= P_ST_IDLE;
  else
    R_cst <= R_nst;
end

always @(*)
begin
  if(I_rst)
    R_nst <= P_ST_IDLE;
  else
    begin
      case(R_cst)
        P_ST_IDLE:
          begin
            if(R_st)
              R_nst = P_ST_ST;
            else
              R_nst = R_nst;
          end
        P_ST_ST:
          R_nst = P_ST_VTCL;
        P_ST_VTCL:
          R_nst = P_ST_DELAY0;
        P_ST_DELAY0:
          begin
            if( &R_delay0_cnt)       // 32 clk, at least 10 clk
              R_nst = P_ST_VAL;
            else
              R_nst = R_nst;
          end
        P_ST_VAL:
          R_nst = P_ST_LOAD;
        P_ST_LOAD:
          R_nst = P_ST_DELAY1;
        P_ST_DELAY1:
          begin
            if( &R_delay0_cnt )      // 32 clk, at least 10 clk
              R_nst = P_ST_VTCH;
            else
              R_nst = R_nst;
          end
        P_ST_VTCH:
          R_nst = P_ST_DELAY2;
        P_ST_DELAY2:
          begin
            if( R_delay1_cnt == P_DELAY_TIME )
              R_nst = P_ST_JUDG;
            else
              R_nst = R_nst;
          end
        P_ST_JUDG:
          begin
            if( R_circu_cnt == P_CIRCU_A + 10'd1 )
              R_nst = P_ST_IDLE;
            else if( R_circu_cnt == P_CIRCU_A )
              R_nst = P_ST_END;
            else
              R_nst = P_ST_VTCL;
          end
        P_ST_END:
            if( R_delay2_cnt == P_END_DELAY )
              R_nst = P_ST_VTCL;
            else
              R_nst = R_nst;
      default:
        R_nst = P_ST_IDLE;
      endcase
    end
end
always @(posedge I_clk)
begin
  if(I_rst)
    R_tap_ok <= 1'b0;
  else if(R_st_judg && I_align_ok)
    R_tap_ok <= 1'b1;
  else
    R_tap_ok <= 1'b0;
end

always @(posedge I_clk)
begin
  if(I_rst)
    R_tap_err <= 1'b0;
  else if(R_st_judg && (!I_align_ok) )
    R_tap_err <= 1'b1;
  else
    R_tap_err <= 1'b0;
end

always @(posedge I_clk)
begin
  if(I_rst)
    begin
      R_tap_err_d1 <= 1'b0;
      R_tap_err_d2 <= 1'b0;
      R_tap_err_d3 <= 1'b0;
    end
  else
    begin
      R_tap_err_d1 <= R_tap_err;
      R_tap_err_d2 <= R_tap_err_d1;
      R_tap_err_d3 <= R_tap_err_d2;
    end
end

always @(posedge I_clk)
begin
  if(I_rst)
    R_tap_len <= 10'd0;
  else if(R_st_idle)
    R_tap_len <= 10'd0;
  else if(R_tap_err_d2)
    R_tap_len <= 10'd0;
  else if(R_tap_ok)
    R_tap_len <= R_tap_len + P_STEP;
  else
    R_tap_len <= R_tap_len;
end

always @(posedge I_clk)
begin
  if(I_rst)
    R_tap_len_d1 <= 10'd0;
  else
    R_tap_len_d1 <= R_tap_len;
end

always @(posedge I_clk)
begin
  if(I_rst)
    R_ram_len_wea <= 1'b0;
  else if(R_st_judg && (R_circu_cnt == P_CIRCU_A)) // last tap and tap ok
    R_ram_len_wea <= 1'b1;
  else if(R_tap_err)
    R_ram_len_wea <= 1'b1;
  else
    R_ram_len_wea <= 1'b0;
end

always @(posedge I_clk)
begin
  if(I_rst)
    begin
      R_ram_len_wea_d1 <= 1'b0;
      R_ram_len_wea_d2 <= 1'b0;
    end
  else
    begin
      R_ram_len_wea_d1 <= R_ram_len_wea;
      R_ram_len_wea_d2 <= R_ram_len_wea_d1;
    end
end

always @(posedge I_clk)
begin
  if(I_rst)
    R_ram_len_waddr <= 9'd0;
  else if(R_st_idle)
    R_ram_len_waddr <= 9'd0;
  else if(R_ram_len_wea_d2)
    R_ram_len_waddr <= R_ram_len_waddr + 9'd1;
  else
    R_ram_len_waddr <= R_ram_len_waddr;
end

always @(posedge I_clk)
begin
  if(I_rst)
    R_ram_len_wdata <= 9'd0;
  else
    R_ram_len_wdata <= R_tap_len;
end

always @(posedge I_clk)
begin
  if(I_rst)
    R_ram_len_raddr <= 9'd0;
  else if(R_st_end_d1)
    begin
      if(R_ram_len_raddr == (R_ram_len_waddr - 9'd1))
        R_ram_len_raddr <= R_ram_len_raddr;
      else
        R_ram_len_raddr <= R_ram_len_raddr + 1'b1;
    end
  else
    R_ram_len_raddr <= 9'd0;
end

blk_mem_9x512 u_ram_tap_len (
  .clka  (I_clk           ),    // input wire clka
  .wea   (R_ram_len_wea_d2),    // input wire [0 : 0] wea
  .addra (R_ram_len_waddr ),    // input wire [8 : 0] addra
  .dina  (R_ram_len_wdata ),    // input wire [8 : 0] dina
  .clkb  (I_clk           ),    // input wire clkb
  .addrb (R_ram_len_raddr ),    // input wire [8 : 0] addrb
  .doutb (W_ram_len_data  )     // output wire [8 : 0] doutb
);

always @(posedge I_clk)
begin
  if(I_rst)
    R_ram_len_data_d1 <= 9'd0;
  else
    R_ram_len_data_d1 <= W_ram_len_data;
end

always @(posedge I_clk)
begin
  if(I_rst)
    R_max_len <= 9'd0;
  else if(R_st_idle)
    R_max_len <= 9'd0;
  else if(R_st_end)
    begin
      if(R_ram_len_data_d1 > R_max_len)
        R_max_len <= R_ram_len_data_d1;
      else
        R_max_len <= R_max_len;
    end
  else
    R_max_len <= R_max_len;
end

always @(posedge I_clk)
begin
  if(I_rst)
    R_ram_tap_raddr <= 9'd0;
  else
    R_ram_tap_raddr <= R_max_len;
end

always @(posedge I_clk)
begin
  if(I_rst)
    R_ram_tap_wdata <= 9'd0;
  else if(R_st_judg_d2 && (R_circu_cnt == P_CIRCU_A) )
    R_ram_tap_wdata <= R_tap_val - R_tap_len[9:1];         // middle tap
  else if(R_tap_err_d1)
    R_ram_tap_wdata <= R_tap_val - R_tap_len[9:1];         // middle tap
  else
    R_ram_tap_wdata <= R_ram_tap_wdata;
end

blk_mem_9x512 u_ram_tap_data (
  .clka  (I_clk           ),  // input wire clka
  .wea   (R_ram_len_wea_d2),  // input wire [0 : 0] wea
  .addra (R_tap_len_d1    ),  // input wire [8 : 0] addra
  .dina  (R_ram_tap_wdata ),  // input wire [8 : 0] dina
  .clkb  (I_clk           ),  // input wire clkb
  .addrb (R_ram_tap_raddr ),  // input wire [8 : 0] addrb
  .doutb (W_ram_tap_data  )   // output wire [8 : 0] doutb
);

always @(posedge I_clk)
begin
  if(I_rst)
    R_tap_val <= 9'd0;
  else if(R_st_end_d1)
    R_tap_val <= W_ram_tap_data;
  else if(R_st_judg_d1)
    R_tap_val <= R_tap_val + P_STEP;
  else
    R_tap_val <= R_tap_val;
end

always @(posedge I_clk)
begin
  if(I_rst)
    O_cntval <= 9'd0;
  else if(R_st_val)
    O_cntval <= R_tap_val;
  else
    O_cntval <= O_cntval;
end

always @(posedge I_clk)
begin
  if(I_rst)
    O_en_vtc <= 1'b1;
  else if(R_st_vtcl)
    O_en_vtc <= 1'b0;
  else if(R_st_vtch)
    O_en_vtc <= 1'b1;
  else
    O_en_vtc <= O_en_vtc;
end

always @(posedge I_clk)
begin
  if(I_rst)
    O_load <= 1'b0;
  else if (R_st_load)
    O_load <= 1'b1;
  else
    O_load <= 1'b0;
end

always @(posedge I_clk)
begin
  O_ce <= 1'b0;
end

always @(posedge I_clk)
begin
  O_inc <= 1'b0;
end

endmodule



