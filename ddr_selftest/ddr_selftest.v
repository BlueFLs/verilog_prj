module ddr_selftest
(
  //clk and reset port
  input                                      I_sys_rst        ,

  //-- input parameter
  input                                      I_test_en        , // flag, high:en
  input          [1  :0]                     I_test_mode      , // 0: All,increasing; 1:All,fixed, 2:part,increasing 3:part, fixed
  input          [15 :0]                     I_test_data      , // fixed test data
  input          [28 :0]                     I_test_start_addr, // test start addr , must 8X
  input          [28 :0]                     I_test_end_addr  , // test end addr

  //-- output
  output  wire                               O_ddr4_init_done ,
  output  reg    [1  :0]                     O_test_status    , // 0:test not start; 1:testing; 2:test end and success; 3:test end and fail
  output  reg    [28 :0]                     O_fail_addr      , // fail addr
  output  reg    [511:0]                     O_fail_right_data, // the right data on fail addr
  output  reg    [511:0]                     O_fail_fact_data , // the fact data on fail addr
  output  reg    [31 :0]                     O_test_time      , // the test time one time,unit is 4ns.
  output  reg    [31 :0]                     O_test_finish_cnt, // the test finish cnt , (O_test_finish_cnt*O_test_time) ~= all test time
  output  reg    [3  :0]                     O_err_ddr_num    , // the err ddr num, {ddr_3,ddr_2,ddr_1,ddr_0}, ex: 3: ddr_0 and ddr_1 err
  output  reg                                O_st_fail        , // fsm fail

  //ddr3 port
  input                                      I_ddr4_refclk0_p ,
  input                                      I_ddr4_refclk0_n ,
  inout   wire    [63:0]                     IO_ddr4_dq       ,
  inout   wire    [7:0]                      IO_ddr4_dqs_c    ,
  inout   wire    [7:0]                      IO_ddr4_dqs_t    ,
  inout   wire    [7:0]                      IO_ddr4_dm_dbi_n ,
  output  wire    [16:0]                     O_ddr4_adr       ,
  output  wire    [1:0]                      O_ddr4_ba        ,
  output                                     O_ddr4_bg        ,
  output  wire                               O_ddr4_act_n     ,
  output  wire                               O_ddr4_cke       ,
  output  wire                               O_ddr4_odt       ,
  output  wire                               O_ddr4_cs_n      ,
  output  wire                               O_ddr4_ck_t      ,
  output  wire                               O_ddr4_ck_c      ,
  output  wire                               O_ddr4_reset_n
);  
parameter  P_T         = 8192      ;  // 8192 times, 8192 * 8 = 2**16 , one row
parameter  P_CIRCU     = 8192      ;  // 8192 circus , 2 ** 16 * 8192 , all addr
//parameter  P_CIRCU     = 3         ;// just for sim

parameter  P_WR_CMD    = 3'b000    ;
parameter  P_RD_CMD    = 3'b001    ;

parameter  P_ST_IDLE   = 7'd1      ;
parameter  P_ST_ST     = 7'd2      ;
parameter  P_ST_WR     = 7'd4      ;
parameter  P_ST_WR_END = 7'd8      ;
parameter  P_ST_RD     = 7'd16     ;
parameter  P_ST_RD_END = 7'd32     ;
parameter  P_ST_FINISH = 7'd64     ;

wire          W_uclk               ; // 1G/4 = 250MHZ
wire          W_urst               ;
reg           R_app_en             ;
wire          W_app_en             ;
wire          W_app_wdf_end        ;
wire          W_app_wdf_wren       ;

wire          W_app_rd_vld         ;
wire          W_app_rdy            ;
wire          W_app_wdf_rdy        ;

wire [28 :0]  W_app_addr           ;
reg  [2  :0]  R_app_cmd            ;
wire [511:0]  W_app_wdf_data       ;
wire [511:0]  W_app_rd_data        ;
reg  [511:0]  R_app_rd_data_d1     ;

reg           R_test_en_cdc        ;
reg           R_test_en_d1         ;
reg           R_test_en_d2         ;
reg           R_test_en_r          ;

reg  [1  :0]  R_test_mode_cdc      ;
reg  [15 :0]  R_test_data_cdc      ;
reg  [28 :0]  R_test_start_addr_cdc;
reg  [28 :0]  R_test_end_addr_cdc  ;

reg  [1  :0]  R_test_mode          ;
reg  [15 :0]  R_test_data          ;
reg  [28 :0]  R_test_start_addr    ;
reg  [28 :0]  R_test_end_addr      ;

reg  [28 :0]  R_start_addr         ;
reg  [28 :0]  R_app_wraddr         ;
reg  [28 :0]  R_app_rdaddr         ;
reg  [28 :0]  R_vld_rdaddr         ;
reg  [28 :0]  R_vld_rdaddr_d1      ;

reg  [6  :0]  R_cst                ;
reg  [6  :0]  R_nst                ;

reg           R_st                 ;
reg           R_wr_end             ;
reg           R_rd_end             ;
reg           R_rd_end_d1          ;
reg           R_rd_end_d2          ;
reg           R_rd_end_d3          ;
reg           R_finish             ;

reg           R_st_idle            ;
reg           R_st_st              ;
reg           R_st_wr              ;
reg           R_st_rd              ;
reg           R_st_wr_d1           ;
reg           R_st_rd_d1           ;
reg           R_st_wr_end          ;
reg           R_st_rd_end          ;
reg           R_st_finish          ;

reg  [13 :0]  R_circu_cnt          ;

reg  [15 :0]  R_wr_data            ;
reg  [15 :0]  R_rd_m_data          ;
wire [511:0]  W_rd_ma_data         ;
reg  [511:0]  R_rd_ma_data_d1      ;

reg  [12 :0]  R_app_cnt            ;
reg  [12 :0]  R_rd_vld_cnt         ;

reg           R_test_mode0         ;
reg           R_test_mode1         ;
reg           R_test_mode2         ;
reg           R_test_mode3         ;

reg           R_fail               ;
reg           R_fail_d1            ;
wire          W_fail_r             ;

reg [7:0]     R_ddr0_err           ;
reg [7:0]     R_ddr1_err           ;
reg [7:0]     R_ddr2_err           ;
reg [7:0]     R_ddr3_err           ;

wire          W_ddr0_err           ;
wire          W_ddr1_err           ;
wire          W_ddr2_err           ;
wire          W_ddr3_err           ;

genvar        i ;

//-- cdc
always @(posedge W_uclk)
begin
  if(W_urst)
    R_test_en_cdc <= 1'b0;
  else
    R_test_en_cdc <= I_test_en;
end

always @(posedge W_uclk)
begin
  if(W_urst)
    R_test_mode_cdc <= 2'd0;
  else
    R_test_mode_cdc <= I_test_mode;
end

always @(posedge W_uclk)
begin
  if(W_urst)
    R_test_data_cdc <= 16'd0;
  else
    R_test_data_cdc <= I_test_data;
end

always @(posedge W_uclk)
begin
  if(W_urst)
    R_test_start_addr_cdc <= 16'd0;
  else
    R_test_start_addr_cdc <= I_test_start_addr;
end

always @(posedge W_uclk)
begin
  if(W_urst)
    R_test_end_addr_cdc <= 16'd0;
  else
    R_test_end_addr_cdc <= I_test_end_addr;
end
//--syn

always @(posedge W_uclk)
begin
  if(W_urst)
    R_test_mode <= 2'd0;
  else if(R_st_idle)
    R_test_mode <= R_test_mode_cdc;
  else
    R_test_mode <= R_test_mode;
end

always @(posedge W_uclk)
begin
  if(W_urst)
    R_test_data <= 2'd0;
  else if(R_st_idle)
    R_test_data <= R_test_data_cdc;
  else
    R_test_data <= R_test_data;
end

always @(posedge W_uclk)
begin
  if(W_urst)
    R_test_start_addr <= 2'd0;
  else if(R_st_idle)
    R_test_start_addr <= R_test_start_addr_cdc;
  else
    R_test_start_addr <= R_test_start_addr;
end

always @(posedge W_uclk)
begin
  if(W_urst)
    R_test_end_addr <= 2'd0;
  else if(R_st_idle)
    R_test_end_addr <= R_test_end_addr_cdc;
  else
    R_test_end_addr <= R_test_end_addr;
end

//-- test mode
always @(posedge W_uclk)
begin
  if(W_urst)
    begin
      R_test_mode0 <= 1'b0;
      R_test_mode1 <= 1'b0;
      R_test_mode2 <= 1'b0;
      R_test_mode3 <= 1'b0;
    end
  else
    begin
      R_test_mode0 <= (R_test_mode == 2'd0);
      R_test_mode1 <= (R_test_mode == 2'd1);
      R_test_mode2 <= (R_test_mode == 2'd2);
      R_test_mode3 <= (R_test_mode == 2'd3);
    end
end

//-- R_start_addr

always @(posedge W_uclk)
begin
  if(W_urst)
    R_start_addr <= 29'd0;
  else if(R_test_mode0 || R_test_mode1)
    R_start_addr <= 29'd0;
  else if(R_test_mode2 || R_test_mode3)
    R_start_addr <= R_test_start_addr;
  else
    R_start_addr <= R_start_addr;
end

always @(posedge W_uclk)
begin
  if(W_urst)
    begin
      R_st_idle   <= 1'b0;
      R_st_st     <= 1'b0;
      R_st_wr     <= 1'b0;
      R_st_rd     <= 1'b0;
      R_st_wr_end <= 1'b0;
      R_st_rd_end <= 1'b0;
      R_st_finish <= 1'b0;
    end
  else
    begin
      R_st_idle   <= (R_nst == P_ST_IDLE);
      R_st_st     <= (R_nst == P_ST_ST);
      R_st_wr     <= (R_nst == P_ST_WR);
      R_st_rd     <= (R_nst == P_ST_RD);
      R_st_wr_end <= (R_nst == P_ST_WR_END);
      R_st_rd_end <= (R_nst == P_ST_RD_END);
      R_st_finish <= (R_nst == P_ST_FINISH);
    end
end

always @(posedge W_uclk)
begin
  if(W_urst)
    R_st_wr_d1 <= 1'b0;
  else
    R_st_wr_d1 <= R_st_wr;
end

always @(posedge W_uclk)
begin
  if(W_urst)
    R_st_rd_d1 <= 1'b0;
  else
    R_st_rd_d1 <= R_st_rd;
end

//-- R_st
always @(posedge W_uclk)
begin
  if(W_urst)
    begin
      R_test_en_d1 <= 1'b0;
      R_test_en_d2 <= 1'b0;
    end
  else
    begin
      R_test_en_d1 <= R_test_en_cdc;
      R_test_en_d2 <= R_test_en_d1;
    end
end

always @(posedge W_uclk)
begin
  if(W_urst)
    R_test_en_r <= 1'b0;
  else
    R_test_en_r <= ~R_test_en_d2 & R_test_en_d1;
end

always @(posedge W_uclk)
begin
  if(W_urst)
    R_st <= 1'b0;
  else if(R_test_en_cdc && O_ddr4_init_done && R_st_idle && (!R_fail))
    R_st <= 1'b1;
  else if(R_test_en_r && O_ddr4_init_done && R_st_idle && R_fail)
    R_st <= 1'b1;
  else
    R_st <= 1'b0;
end

//-- app
//-- app_en/app_cmd /app_addr

always @(posedge W_uclk)
begin
  if(W_urst)
    R_app_en <= 1'b0;
  else if(R_st_rd && R_fail)
    R_app_en <= 1'b0;
  else if(R_st_wr && (R_app_cnt == P_T - 13'd1) && W_app_rdy && W_app_wdf_rdy)
    R_app_en <= 1'b0;
  else if(R_st_rd && (R_app_cnt == P_T - 13'd1) && W_app_rdy)
    R_app_en <= 1'b0;
  else if(R_st_wr && (R_app_cnt < P_T - 13'd1))
    R_app_en <= 1'b1;
  else if(R_st_rd && (R_app_cnt < P_T - 13'd1))
    R_app_en <= 1'b1;
  else
    R_app_en <= R_app_en;
end

assign W_app_en = (R_st_wr_d1) ? (R_app_en && W_app_rdy && W_app_wdf_rdy) : (R_app_en && W_app_rdy);

always @(posedge W_uclk)
begin
  if(W_urst)
    R_app_cmd <= 3'd0;
  else if(R_st_wr)
    R_app_cmd <= P_WR_CMD;
  else if(R_st_rd)
    R_app_cmd <= P_RD_CMD;
  else
    R_app_cmd <= 3'd0;
end

always @(posedge W_uclk)
begin
  if(W_urst)
    R_app_wraddr <= 29'd0;
  else if(R_st_st)
    R_app_wraddr <= R_start_addr;
  else if(R_app_en && W_app_rdy && W_app_wdf_rdy && R_st_wr_d1)
    R_app_wraddr <= R_app_wraddr + 29'd8;  // one burst 8 addr
  else
    R_app_wraddr <= R_app_wraddr;
end

always @(posedge W_uclk)
begin
  if(W_urst)
    R_app_rdaddr <= 29'd0;
  else if(R_st_st)
    R_app_rdaddr <= R_start_addr;
  else if(R_app_en && W_app_rdy && R_st_rd_d1)
    R_app_rdaddr <= R_app_rdaddr + 29'd8;  // one burst 8 addr
  else
    R_app_rdaddr <= R_app_rdaddr;
end

assign W_app_addr = (R_st_wr_d1) ? R_app_wraddr : R_app_rdaddr;

//-- app write
//-- app_wdf_wren / app_wdf_end / app_wdf_data

assign W_app_wdf_wren = R_app_en && W_app_wdf_rdy && W_app_rdy && R_st_wr;
assign W_app_wdf_end  = W_app_wdf_wren;

always @(posedge W_uclk)
begin
  if(W_urst)
    R_app_cnt <= 13'd0;
  else if(R_st_wr_end || R_st_rd_end || R_st_idle)
    R_app_cnt <= 13'd0;
  else if( W_app_rdy && W_app_wdf_rdy && R_st_wr_d1 && (R_app_cnt < (P_T - 13'd1)) )
    R_app_cnt <= R_app_cnt + 13'd1;
  else if( W_app_rdy && R_st_rd_d1 && (R_app_cnt < (P_T - 13'd1)) )
    R_app_cnt <= R_app_cnt + 13'd1;
  else
    R_app_cnt <= R_app_cnt;
end

always @(posedge W_uclk)
begin
  if(W_urst)
    R_wr_data <= 16'd0;
  else if(R_st_idle)
    R_wr_data <= 16'd0;
  else if(W_app_wdf_wren && W_app_wdf_rdy && W_app_rdy && (R_test_mode0 || R_test_mode2))
    R_wr_data <= R_wr_data + 16'd1;
  else if( R_st_wr && (R_test_mode1 || R_test_mode3))
    R_wr_data <= R_test_data;
  else
    R_wr_data <= R_wr_data;
end

assign W_app_wdf_data = {32{R_wr_data}};

always @(posedge W_uclk)
begin
  if(W_urst)
    R_wr_end <= 1'b0;
  else if(R_st_wr_end)
    R_wr_end <= 1'b0;
  else if(R_st_wr && (R_app_cnt == P_T - 13'd1) && W_app_rdy && W_app_wdf_rdy)
    R_wr_end <= 1'b1;
  else
    R_wr_end <= R_wr_end;
end

//---
always @(posedge W_uclk)
begin
  if(W_urst)
    R_rd_vld_cnt <= 13'd0;
  else if(R_st_rd_end || R_st_idle)
    R_rd_vld_cnt <= 13'd0;
  else if(W_app_rd_vld)
    R_rd_vld_cnt <= R_rd_vld_cnt + 13'd1;
  else
    R_rd_vld_cnt <= R_rd_vld_cnt;
end

always @(posedge W_uclk)
begin
  if(W_urst)
    R_rd_end <= 1'b0;
  else if(R_st_rd_end || R_st_idle)
    R_rd_end <= 1'b0;
  else if(R_st_rd && (R_rd_vld_cnt == P_T - 13'b1) && W_app_rd_vld)
    R_rd_end <= 1'b1;
  else
    R_rd_end <= R_rd_end;
end

always @(posedge W_uclk)
begin
  if(W_urst)
    begin
      R_rd_end_d1 <= 1'b0;
      R_rd_end_d2 <= 1'b0;
      R_rd_end_d3 <= 1'b0;
    end
  else
    begin
      R_rd_end_d1 <= R_rd_end;
      R_rd_end_d2 <= R_rd_end_d1;
      R_rd_end_d3 <= R_rd_end_d2;
    end
end

always @(posedge W_uclk)
begin
  if(W_urst)
    R_circu_cnt <= 14'd0;
  else if(R_st_st)
    R_circu_cnt <= 14'd0;
  else if(R_st_wr_end)
    R_circu_cnt <= R_circu_cnt + 14'd1;
  else
    R_circu_cnt <= R_circu_cnt;
end

always @(posedge W_uclk)
begin
  if(W_urst)
    R_finish <= 1'b0;
  else if(R_st_st)
    R_finish <= 1'b0;
  else if( (R_circu_cnt >= P_CIRCU ) && (R_test_mode0 || R_test_mode1) )
    R_finish <= 1'b1;
  else if( (R_app_rdaddr >= R_test_end_addr) && (R_test_mode2 || R_test_mode3) )
    R_finish <= 1'b1;
  else
    R_finish <= R_finish;
end
always @(posedge W_uclk)
begin
  if(W_urst)
    R_rd_m_data <= 16'd0;
  else if(R_st_st)
    R_rd_m_data <= 16'd0;
  else if(W_app_rd_vld && (R_test_mode0 || R_test_mode2))
    R_rd_m_data <= R_rd_m_data + 16'd1;
  else if(R_test_mode1 || R_test_mode3)
    R_rd_m_data <= R_test_data;
  else
    R_rd_m_data <= R_rd_m_data;
end

assign W_rd_ma_data = {32{R_rd_m_data}};

always @(posedge W_uclk)
begin
  if(W_urst)
    R_rd_ma_data_d1 <= 512'd0;
  else
    R_rd_ma_data_d1 <= W_rd_ma_data;
end

always @(posedge W_uclk)
begin
  if(W_urst)
    R_app_rd_data_d1 <= 512'd0;
  else
    R_app_rd_data_d1 <= W_app_rd_data;
end

always @(posedge W_uclk)
begin
  if(W_urst)
    R_vld_rdaddr <= 29'd0;
  else if(R_st_st)
    R_vld_rdaddr <= R_start_addr;
  else if(W_app_rd_vld)
    R_vld_rdaddr <= R_vld_rdaddr + 29'd8;
  else
    R_vld_rdaddr <= R_vld_rdaddr;
end

always @(posedge W_uclk)
begin
  if(W_urst)
    R_vld_rdaddr_d1 <= 29'd0;
  else
    R_vld_rdaddr_d1 <= R_vld_rdaddr;
end
always @(posedge W_uclk)
begin
  if(W_urst)
    R_fail <= 1'b0;
  else if(R_st_st)
    R_fail <= 1'b0;
  else if((W_rd_ma_data != W_app_rd_data) && W_app_rd_vld)
    R_fail <= 1'b1;
  else
    R_fail <= R_fail;
end

always @(posedge W_uclk)
begin
  if(W_urst)
    R_fail_d1 <= 1'b0;
  else
    R_fail_d1 <= R_fail;
end

assign W_fail_r = ~R_fail_d1 & R_fail;

//-- output
//-- O_test_status / O_fail_addr / O_fail_fact_data / O_fail_right_data
//-- O_st_fail     / O_test_time / O_test_finish_cnt/ O_err_ddr_num

lways @(posedge W_uclk)
begin
  if(W_urst)
    O_test_status <= 2'd0;
  else
  begin
    case(R_cst)
      P_ST_IDLE:
        begin
          if(!R_finish && !R_fail)
            O_test_status <= 2'd0;
          else
            O_test_status <= O_test_status;
        end
      P_ST_ST,P_ST_WR,P_ST_WR_END,P_ST_RD,P_ST_RD_END:
        O_test_status <= 2'd1;
      P_ST_FINISH:
      begin
        if(R_fail)
          O_test_status <= 2'd3;
        else
          O_test_status <= 2'd2;
      end
      default:
        O_test_status <= O_test_status;
    endcase
  end
end

always @(posedge W_uclk)
begin
  if(W_urst)
    O_fail_addr <= 29'd0;
  else if(R_st_st)
    O_fail_addr <= 29'd0;
  else if(W_fail_r)
    O_fail_addr <= R_vld_rdaddr_d1;
  else
    O_fail_addr <= O_fail_addr;
end

always @(posedge W_uclk)
begin
  if(W_urst)
    O_fail_right_data <= 512'd0;
  else if(R_st_st)
    O_fail_right_data <= 512'd0;
  else if(W_fail_r)
    O_fail_right_data <= R_rd_ma_data_d1;
  else
    O_fail_right_data <= O_fail_right_data;
end

always @(posedge W_uclk)
begin
  if(W_urst)
    O_fail_fact_data <= 512'd0;
  else if(R_st_st)
    O_fail_fact_data <= 512'd0;
  else if(W_fail_r)
    O_fail_fact_data <= R_app_rd_data_d1;
  else
    O_fail_fact_data <= O_fail_fact_data;
end

generate

for(i = 0; i<8; i = i+1)
  begin: ddr_err_loop

    always @(posedge W_uclk)
    begin
      if(W_urst)
        R_ddr0_err[i] <= 1'b0;
      else if(R_st_st)
        R_ddr0_err[i] <= 1'b0;
      else if(O_fail_right_data[i*64 +: 16] != O_fail_fact_data[i*64 +: 16])
        R_ddr0_err[i] <= 1'b1;
      else
        R_ddr0_err[i] <= R_ddr0_err[i];
    end

    always @(posedge W_uclk)
    begin
      if(W_urst)
        R_ddr1_err[i] <= 1'b0;
      else if(R_st_st)
        R_ddr1_err[i] <= 1'b0;
      else if(O_fail_right_data[i*64+16 +: 16] != O_fail_fact_data[i*64+16 +: 16])
        R_ddr1_err[i] <= 1'b1;
      else
        R_ddr1_err[i] <= R_ddr1_err[i];
    end

    always @(posedge W_uclk)
    begin
      if(W_urst)
        R_ddr2_err[i] <= 1'b0;
      else if(R_st_st)
        R_ddr2_err[i] <= 1'b0;
      else if(O_fail_right_data[i*64+32 +: 16] != O_fail_fact_data[i*64+32 +: 16])
        R_ddr2_err[i] <= 1'b1;
      else
        R_ddr2_err[i] <= R_ddr2_err[i];
    end

    always @(posedge W_uclk)
    begin
      if(W_urst)
        R_ddr3_err[i] <= 1'b0;
      else if(R_st_st)
        R_ddr3_err[i] <= 1'b0;
      else if(O_fail_right_data[i*64+48 +: 16] != O_fail_fact_data[i*64+48 +: 16])
        R_ddr3_err[i] <= 1'b1;
      else
        R_ddr3_err[i] <= R_ddr3_err[i];
    end

  end

endgenerate

assign W_ddr0_err = |R_ddr0_err;
assign W_ddr1_err = |R_ddr1_err;
assign W_ddr2_err = |R_ddr2_err;
assign W_ddr3_err = |R_ddr3_err;

always @(posedge W_uclk)
begin
  if(W_urst)
    O_err_ddr_num <= 4'd0;
  else if(R_st_st)
    O_err_ddr_num <= 4'd0;
  else
    O_err_ddr_num <= {W_ddr3_err, W_ddr2_err, W_ddr1_err, W_ddr0_err};
end

always @(posedge W_uclk)
begin
  if(W_urst)
    O_st_fail <= 1'b0;
  else if(!R_test_en_cdc)
    O_st_fail <= 1'b0;
  else if(R_st_idle || R_st_st || R_st_wr || R_st_rd || R_st_wr_end || R_st_rd_end || R_st_finish)
    O_st_fail <= 1'b0;
  else
    O_st_fail <= 1'b1;
end

always @(posedge W_uclk)
begin
  if(W_urst)
    O_test_time <= 32'd0;
  else if(O_test_status == 2'd0)
    O_test_time <= 32'd0;
  else if(O_test_status == 2'd1)
    O_test_time <= O_test_time + 32'd1;
  else
    O_test_time <= O_test_time;
end

always @(posedge W_uclk)
begin
  if(W_urst)
    O_test_finish_cnt <= 32'd0;
  else if(!R_test_en_cdc)
    O_test_finish_cnt <= 32'd0;
  else if(R_st_finish)
    O_test_finish_cnt <= O_test_finish_cnt + 32'd1;
  else
    O_test_finish_cnt <= O_test_finish_cnt;
end

//--status
always @(posedge W_uclk)
begin
  if(W_urst)
    R_cst <= P_ST_IDLE;
  else
    R_cst <= R_nst;
end

always @(*)
begin
  if(W_urst)
    R_nst = P_ST_IDLE;
  else
    begin
      case(R_cst)
        P_ST_IDLE:
          begin
            if(R_st)
              R_nst = P_ST_ST;
            else
              R_nst = P_ST_IDLE;
          end
        P_ST_ST:
          R_nst = P_ST_WR;
        P_ST_WR:
          begin
            if(R_wr_end)
              R_nst = P_ST_WR_END;
            else
              R_nst = P_ST_WR;
          end
        P_ST_WR_END:
          R_nst = P_ST_RD;
        P_ST_RD:
          begin
            if(R_fail)
              R_nst = P_ST_FINISH;
            else if(R_rd_end_d3)
              R_nst = P_ST_RD_END;
            else
              R_nst = P_ST_RD;
          end
        P_ST_RD_END:
          begin
            if(R_finish)
              R_nst = P_ST_FINISH;
            else
              R_nst = P_ST_WR;
          end
        P_ST_FINISH:
          R_nst = P_ST_IDLE;
        default:
          R_nst = P_ST_IDLE;
      endcase
    end
end
  
ddr4_mig ddr4_mig_u (
  .c0_init_calib_complete   (O_ddr4_init_done         ),        // output wire c0_init_calib_complete
  .dbg_clk                  (                         ),        // output wire dbg_clk
  .c0_sys_clk_p             (I_ddr4_refclk0_p         ),        // input wire c0_sys_clk_p
  .c0_sys_clk_n             (I_ddr4_refclk0_n         ),        // input wire c0_sys_clk_n
  .dbg_bus                  (                         ),        // output wire [511 : 0] dbg_bus
  .c0_ddr4_adr              (O_ddr4_adr               ),        // output wire [16 : 0] c0_ddr4_adr
  .c0_ddr4_ba               (O_ddr4_ba                ),        // output wire [1 : 0] c0_ddr4_ba
  .c0_ddr4_cke              (O_ddr4_cke               ),        // output wire [0 : 0] c0_ddr4_cke
  .c0_ddr4_cs_n             (O_ddr4_cs_n              ),        // output wire [0 : 0] c0_ddr4_cs_n
  .c0_ddr4_dm_dbi_n         (IO_ddr4_dm_dbi_n         ),        // inout wire [7 : 0] c0_ddr4_dm_dbi_n
  .c0_ddr4_dq               (IO_ddr4_dq               ),        // inout wire [63 : 0] c0_ddr4_dq
  .c0_ddr4_dqs_c            (IO_ddr4_dqs_c            ),        // inout wire [7 : 0] c0_ddr4_dqs_c
  .c0_ddr4_dqs_t            (IO_ddr4_dqs_t            ),        // inout wire [7 : 0] c0_ddr4_dqs_t
  .c0_ddr4_odt              (O_ddr4_odt               ),        // output wire [0 : 0] c0_ddr4_odt
  .c0_ddr4_bg               (O_ddr4_bg                ),        // output wire [0 : 0] c0_ddr4_bg
  .c0_ddr4_reset_n          (O_ddr4_reset_n           ),        // output wire c0_ddr4_reset_n
  .c0_ddr4_act_n            (O_ddr4_act_n             ),        // output wire c0_ddr4_act_n
  .c0_ddr4_ck_c             (O_ddr4_ck_c              ),        // output wire [0 : 0] c0_ddr4_ck_c
  .c0_ddr4_ck_t             (O_ddr4_ck_t              ),        // output wire [0 : 0] c0_ddr4_ck_t
  .c0_ddr4_ui_clk           (W_uclk                   ),        // output wire c0_ddr4_ui_clk
  .c0_ddr4_ui_clk_sync_rst  (W_urst                   ),        // output wire c0_ddr4_ui_clk_sync_rst
  .c0_ddr4_app_en           (W_app_en                 ),        // input wire c0_ddr4_app_en
  .c0_ddr4_app_hi_pri       (1'b0                     ),        // input wire c0_ddr4_app_hi_pri
  .c0_ddr4_app_wdf_end      (W_app_wdf_end            ),        // input wire c0_ddr4_app_wdf_end
  .c0_ddr4_app_wdf_wren     (W_app_wdf_wren           ),        // input wire c0_ddr4_app_wdf_wren
  .c0_ddr4_app_rd_data_end  (                         ),        // output wire c0_ddr4_app_rd_data_end
  .c0_ddr4_app_rd_data_valid(W_app_rd_vld             ),        // output wire c0_ddr4_app_rd_data_valid
  .c0_ddr4_app_rdy          (W_app_rdy                ),        // output wire c0_ddr4_app_rdy
  .c0_ddr4_app_wdf_rdy      (W_app_wdf_rdy            ),        // output wire c0_ddr4_app_wdf_rdy
  .c0_ddr4_app_addr         (W_app_addr               ),        // input wire [28 : 0] c0_ddr4_app_addr
  .c0_ddr4_app_cmd          (R_app_cmd                ),        // input wire [2 : 0] c0_ddr4_app_cmd
  .c0_ddr4_app_wdf_data     (W_app_wdf_data           ),        // input wire [511 : 0] c0_ddr4_app_wdf_data
  .c0_ddr4_app_wdf_mask     (64'd0                    ),        // input wire [63 : 0] c0_ddr4_app_wdf_mask
  .c0_ddr4_app_rd_data      (W_app_rd_data            ),        // output wire [511 : 0] c0_ddr4_app_rd_data
  .sys_rst                  (I_sys_rst                )         // input wire sys_rst
);

endmodule

  




  
  
