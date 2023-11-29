module icap_ctrl #(
  parameter P_WBSTAR = 32'h00_01_00_00  // the 2nd bit addr, need selsectmap,bit-swapped. original:32'h00_80_00_00
)
(
  input        I_clk,
  input        I_rst,
  input        I_icap_en,

  output reg   O_icap_done,
  output reg   O_icap_err,
  output reg   O_st_err
);

localparam P_DUMMY    =  32'hff_ff_ff_ff; // selsectmap,bit-swapped. original: 32'hff_ff_ff_ff
localparam P_SYNC     =  32'h55_99_aa_66; // selsectmap,bit-swapped. original: 32'hAA_99_55_66
localparam P_NOOP     =  32'h04_00_00_00; // selsectmap,bit-swapped. original: 32'h20_00_00_00
localparam P_WRWBSTAR =  32'h0c_40_00_80; // selsectmap,bit-swapped. original: 32'h30_02_00_01
localparam P_WRCMD    =  32'h0c_00_01_80; // selsectmap,bit-swapped. original: 32'h30_00_80_01
localparam P_IPROG    =  32'h00_00_00_f0; // selsectmap,bit-swapped. original: 32'h00_00_00_0F

parameter  P_ST_IDLE  =  8'd1           ;
parameter  P_ST_ST    =  8'd2           ;
parameter  P_ST_WRL   =  8'd4           ;
parameter  P_ST_CSL   =  8'd8           ;
parameter  P_ST_SEND  =  8'd16          ;
parameter  P_ST_CSH   =  8'd32          ;
parameter  P_ST_WRH   =  8'd64          ;
parameter  P_ST_DONE  =  8'd128         ;

reg [7 :0]  R_cst          ;
reg [7 :0]  R_nst          ;

reg         R_icap_en_syn  ;
reg         R_icap_en_d1   ;
reg         R_icap_en_d2   ;
reg         R_icap_en_rflag;

reg         R_st           ;
reg         R_send_done    ;

reg [2 :0]  R_send_cnt     ;

reg         R_st_idle      ;
reg         R_st_st        ;
reg         R_st_wrl       ;
reg         R_st_csl       ;
reg         R_st_send      ;
reg         R_st_csh       ;
reg         R_st_wrh       ;
reg         R_st_done      ;

wire        W_avail        ;
wire [31:0] W_out          ;
wire        W_prerror      ;
wire        W_prdone       ;

reg [31:0]  R_in           ;
reg         R_csib         ;
reg         R_rdwrb        ;

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
          R_nst = P_ST_WRL;
        P_ST_WRL:
          R_nst = P_ST_CSL;
        P_ST_CSL:
          R_nst = P_ST_SEND;
        P_ST_SEND:
          begin
            if(R_send_done)
              R_nst = P_ST_CSH;
            else
              R_nst = P_ST_SEND;
          end
        P_ST_CSH:
          R_nst = P_ST_WRH;
        P_ST_WRH:
          R_nst = P_ST_DONE;
        P_ST_DONE:
          begin
            if(R_st)
              R_nst = P_ST_ST;
            else
              R_nst = P_ST_DONE;
          end
        default:R_nst = P_ST_IDLE;
      endcase
    end
end

always @(posedge I_clk)
begin
  if(I_rst)
    begin
      R_icap_en_syn <= 1'b0;
      R_icap_en_d1  <= 1'b0;
      R_icap_en_d2  <= 1'b0;
    end
  else
    begin
      R_icap_en_syn <= I_icap_en;
      R_icap_en_d1  <= R_icap_en_syn;
      R_icap_en_d2  <= R_icap_en_d1;
    end
end

always @(posedge I_clk)
begin
  if(I_rst)
    R_icap_en_rflag <= 1'b0;
  else if(R_st_st)
    R_icap_en_rflag <= 1'b0;
  else if(~R_icap_en_d2 & R_icap_en_d1)
    R_icap_en_rflag <= 1'b1;
  else
    R_icap_en_rflag <= R_icap_en_rflag;
end

always @(posedge I_clk)
begin
  if(I_rst)
    R_st <= 1'b0;
  else if(R_st_st)
    R_st <= 1'b0;
  else if(R_icap_en_rflag && W_avail)
    R_st <= 1'b1;
  else
    R_st <= R_st;
end
always @(posedge I_clk)
begin
  if(I_rst)
    begin
      R_st_idle  <= 1'b0;
      R_st_st    <= 1'b0;
      R_st_wrl   <= 1'b0;
      R_st_csl   <= 1'b0;
      R_st_send  <= 1'b0;
      R_st_csh   <= 1'b0;
      R_st_wrh   <= 1'b0;
      R_st_done  <= 1'b0;
    end
  else
    begin
      R_st_idle  <= (R_nst == P_ST_IDLE);
      R_st_st    <= (R_nst == P_ST_ST);
      R_st_wrl   <= (R_nst == P_ST_WRL);
      R_st_csl   <= (R_nst == P_ST_CSL);
      R_st_send  <= (R_nst == P_ST_SEND);
      R_st_csh   <= (R_nst == P_ST_CSH);
      R_st_wrh   <= (R_nst == P_ST_WRH);
      R_st_done  <= (R_nst == P_ST_DONE);
    end
end

always @(posedge I_clk)
begin
  if(I_rst)
    R_send_cnt <= 3'd0;
  else if(R_nst == P_ST_SEND)
    R_send_cnt <= R_send_cnt + 3'd1;
  else
    R_send_cnt <= 3'd0;
end

always @(posedge I_clk)
begin
  if(I_rst)
    R_send_done <= 1'b0;
  else
    R_send_done <= &R_send_cnt;
end

always @(posedge I_clk)
begin
  if(I_rst)
    R_in <= 32'd0;
  else if(R_nst == P_ST_SEND)
    begin
      case(R_send_cnt)
        3'd0: R_in <= P_DUMMY;
        3'd1: R_in <= P_SYNC;
        3'd2: R_in <= P_NOOP;
        3'd3: R_in <= P_WRWBSTAR;
        3'd4: R_in <= P_WBSTAR;
        3'd5: R_in <= P_WRCMD;
        3'd6: R_in <= P_IPROG;
        3'd7: R_in <= P_NOOP;
        default: R_in <= P_DUMMY;
      endcase
    end
  else
    R_in <= 32'd0;
end

always @(posedge I_clk)
begin
  if(I_rst)
    R_csib <= 1'b1;
  else if(R_st_csl || (R_nst == P_ST_SEND))
    R_csib <= 1'b0;
  else
    R_csib <= 1'b1;
end

always @(posedge I_clk)
begin
  if(I_rst)
    R_rdwrb <= 1'b1;
  else if(R_st_wrl || R_st_csl || R_st_send || R_st_csh)
    R_rdwrb <= 1'b0;
  else
    R_rdwrb <= 1'b1;
end

always @(posedge I_clk)
begin
  if(I_rst)
    O_st_err <= 1'b0;
  else if(R_st_idle || R_st_st || R_st_wrl || R_st_csl || R_st_send || R_st_csh || R_st_wrh || R_st_done)
    O_st_err <= 1'b0;
  else
    O_st_err <= 1'b1;
end

always @(posedge I_clk)
begin
  if(I_rst)
    O_icap_err <= 1'b0;
  else
    O_icap_err <= W_prerror;
end

always @(posedge I_clk)
begin
  if(I_rst)
    O_icap_done <= 1'b0;
  else if(R_st_done && W_prdone)
    O_icap_done <= 1'b1;
  else
    O_icap_done <= 1'b0;
end

//KU060: IDCODE: 32'h03_91_90_93
ICAPE3 #(
  //.DEVICE_ID        (32'h03628093),  // Specifies the pre-programmed Device ID value to be used for simulation purposes.
  .DEVICE_ID        (32'h03_91_90_93),  // Specifies the pre-programmed Device ID value to be used for simulation purposes.
  .ICAP_AUTO_SWITCH ("DISABLE"   ),  // Enable switch ICAP using sync word
  .SIM_CFG_FILE_NAME("NONE"      )   // Specifies the Raw Bitstream (RBT) file to be parsed by the simulation model
)
ICAPE3_inst
(
  .AVAIL            (W_avail   ),    // 1-bit output: Availability status of ICAP
  .O                (W_out     ),    // 32-bit output: Configuration data output bus
  .PRDONE           (W_prdone  ),    // 1-bit output: Indicates completion of Partial Reconfiguration
  .PRERROR          (W_prerror ),    // 1-bit output: Indicates Error during Partial Reconfiguration
  .CLK              (I_clk     ),    // 1-bit input: Clock input
  .CSIB             (R_csib    ),    // 1-bit input: Active-Low ICAP enable
  .I                (R_in      ),    // 32-bit input: Configuration data input bus
  .RDWRB            (R_rdwrb   )     // 1-bit input: Read/Write Select input  low:write , high:read
);
endmodule
  
