module CPU(
  clk, 
  rst_n, 
  IO_stall,
  awid_m_inf, 
  awaddr_m_inf, 
  awsize_m_inf, 
  awburst_m_inf, 
  awlen_m_inf, 
  awvalid_m_inf, 
  awready_m_inf,
  // AXI Write Data
  wdata_m_inf, 
  wlast_m_inf, 
  wvalid_m_inf, 
  wready_m_inf,
  // AXI Write Response
  bid_m_inf, 
  bresp_m_inf, 
  bvalid_m_inf, 
  bready_m_inf,
  // AXI Read Address
  arid_m_inf, 
  araddr_m_inf, 
  arlen_m_inf, 
  arsize_m_inf, 
  arburst_m_inf, 
  arvalid_m_inf, 
  arready_m_inf,
  // AXI Read Data
  rid_m_inf, 
  rdata_m_inf, 
  rresp_m_inf, 
  rlast_m_inf, 
  rvalid_m_inf, 
  rready_m_inf
);
input  wire clk, rst_n;
output reg  IO_stall;

parameter ID_WIDTH=4, ADDR_WIDTH=32, DATA_WIDTH=16, DRAM_NUMBER=2, WRIT_NUMBER=1;

// ----------------------------- AXI Ports -----------------------------
// write address channel 
output wire [WRIT_NUMBER*ID_WIDTH-1:0]   awid_m_inf;
output wire [WRIT_NUMBER*ADDR_WIDTH-1:0] awaddr_m_inf;
output wire [WRIT_NUMBER*3-1:0]          awsize_m_inf;
output wire [WRIT_NUMBER*2-1:0]          awburst_m_inf;
output wire [WRIT_NUMBER*7-1:0]          awlen_m_inf;
output wire [WRIT_NUMBER-1:0]            awvalid_m_inf;
input  wire [WRIT_NUMBER-1:0]            awready_m_inf;
// write data channel 
output wire [WRIT_NUMBER*DATA_WIDTH-1:0] wdata_m_inf;
output wire [WRIT_NUMBER-1:0]            wlast_m_inf;
output wire [WRIT_NUMBER-1:0]            wvalid_m_inf;
input  wire [WRIT_NUMBER-1:0]            wready_m_inf;
// write response channel
input  wire [WRIT_NUMBER*ID_WIDTH-1:0]   bid_m_inf;
input  wire [WRIT_NUMBER*2-1:0]          bresp_m_inf;
input  wire [WRIT_NUMBER-1:0]            bvalid_m_inf;
output wire [WRIT_NUMBER-1:0]            bready_m_inf;
// read address channel 
output wire [DRAM_NUMBER*ID_WIDTH-1:0]   arid_m_inf;
output wire [DRAM_NUMBER*ADDR_WIDTH-1:0] araddr_m_inf; //[63:32]:DRAM_inst, [31:0]:DRAM_data
output wire [DRAM_NUMBER*7-1:0]          arlen_m_inf;  //14h'3FFF : 128 data/burst
output wire [DRAM_NUMBER*3-1:0]          arsize_m_inf; //2 bytes/burst
output wire [DRAM_NUMBER*2-1:0]          arburst_m_inf;
output wire [DRAM_NUMBER-1:0]            arvalid_m_inf; //read address is vaild
input  wire [DRAM_NUMBER-1:0]            arready_m_inf; //DRAM is ready to accept an address
// read data channel 
input  wire [DRAM_NUMBER*ID_WIDTH-1:0]   rid_m_inf;
input  wire [DRAM_NUMBER*DATA_WIDTH-1:0] rdata_m_inf; //data from DRAM, [31:16]:DRAM_inst, [15:0]:DRAM_data
input  wire [DRAM_NUMBER*2-1:0]          rresp_m_inf;
input  wire [DRAM_NUMBER-1:0]            rlast_m_inf; //last data
input  wire [DRAM_NUMBER-1:0]            rvalid_m_inf;
output wire [DRAM_NUMBER-1:0]            rready_m_inf;

// ----------------------------- Registers -----------------------------
reg  signed [15:0] core_r0 , core_r1 , core_r2 , core_r3 ;
reg  signed [15:0] core_r4 , core_r5 , core_r6 , core_r7 ;
reg  signed [15:0] core_r8 , core_r9 , core_r10, core_r11;
reg  signed [15:0] core_r12, core_r13, core_r14, core_r15;

// ----------------------------- FSM Parameter-----------------------------
parameter PREPARE=0, FETCH_INST_DRAM=1, READ_INST_CACHE=2, READ_INST_CACHE_D=3, ID_STAGE=4, ID_STAGE_D1=5, EXE_STAGE=6, PREPARE_DATA=7;
parameter FETCH_DATA_DRAM=8, DATA_CACHE_ACCESS=9, DATA_CACHE_ACCESS_D1=10, DRAM_WRITE=11, OUTPUT=12;



// ----------------------------- Wires / Regs -----------------------------
reg  [3:0] state, state_next;

// cache
reg  [7:0]  cache_addr;
reg  [15:0] cache_din;
wire [15:0] cache_dout;
reg         cache_WEB;


// cpu
reg  signed [15:0] pc;
reg  [15:0] cache_idx_lb_r, cache_idx_ub_r;
reg  [15:0] cache_idx_lb,   cache_idx_ub;

reg  [15:0] inst;
wire [2:0]  opcode;
wire [3:0]  rs_idx, rt_idx, rd_idx;
wire        inst_func;
reg  signed [4:0] imme;
wire [12:0] jump_addr;

reg  signed [15:0] rs_data, rt_data;
reg  signed [15:0] data_wb;
reg  [3:0]  reg_wb;

wire signed [15:0] OFFSET;
reg  [15:0] dc;
wire [15:0] dc_t;
reg  [15:0] data_idx_lb_r, data_idx_ub_r;
reg  [15:0] data_idx_lb,   data_idx_ub;

wire data_cache_miss_flag;
wire inst_cache_miss_flag;

CACHE_256 ISNT_DATA_CACHE(
  .A0(cache_addr[0]), .A1(cache_addr[1]), .A2(cache_addr[2]), .A3(cache_addr[3]),
  .A4(cache_addr[4]), .A5(cache_addr[5]), .A6(cache_addr[6]), .A7(cache_addr[7]),
  .DO0(cache_dout[0]), .DO1(cache_dout[1]), .DO2(cache_dout[2]), .DO3(cache_dout[3]),
  .DO4(cache_dout[4]), .DO5(cache_dout[5]), .DO6(cache_dout[6]), .DO7(cache_dout[7]),
  .DO8(cache_dout[8]), .DO9(cache_dout[9]), .DO10(cache_dout[10]), .DO11(cache_dout[11]),
  .DO12(cache_dout[12]), .DO13(cache_dout[13]), .DO14(cache_dout[14]), .DO15(cache_dout[15]),
  .DI0(cache_din[0]), .DI1(cache_din[1]), .DI2(cache_din[2]), .DI3(cache_din[3]),
  .DI4(cache_din[4]), .DI5(cache_din[5]), .DI6(cache_din[6]), .DI7(cache_din[7]),
  .DI8(cache_din[8]), .DI9(cache_din[9]), .DI10(cache_din[10]), .DI11(cache_din[11]),
  .DI12(cache_din[12]), .DI13(cache_din[13]), .DI14(cache_din[14]), .DI15(cache_din[15]),
  .CK(clk), .WEB(cache_WEB), .OE(1'b1), .CS(1'b1)
);


always @(posedge clk or negedge rst_n) begin
  if(!rst_n) cache_addr <= 8'd0;
  else begin
    case(state)
      PREPARE:           cache_addr <= 8'd255;
      FETCH_INST_DRAM:   cache_addr <= (rvalid_m_inf[1]) ? (cache_addr + 8'd1) : cache_addr;
      READ_INST_CACHE:   cache_addr <= (pc - cache_idx_lb_r) >> 1; //calculate the address of the cache miss instruction pc to cache_idx_lb_r(>>1 since SRAM stores 2 bytes data)
      PREPARE_DATA:      cache_addr <= 8'd127;
      FETCH_DATA_DRAM:   cache_addr <= (rvalid_m_inf[0]) ? (cache_addr + 8'd1) : cache_addr;
      DATA_CACHE_ACCESS,
      DATA_CACHE_ACCESS_D1:cache_addr <= ((dc - data_idx_lb_r) >> 1) + 8'd128;
      DRAM_WRITE:        cache_addr <= dc[7:0];
      default:           cache_addr <= cache_addr;
    endcase
  end
end

always @(posedge clk or negedge rst_n) begin
  if(!rst_n) cache_din <= 16'd0;
  else begin
    case(state)
      FETCH_INST_DRAM:  cache_din <= (rvalid_m_inf[1]) ? rdata_m_inf[31:16] : 16'd0;
      FETCH_DATA_DRAM:  cache_din <= (rvalid_m_inf[0]) ? rdata_m_inf[15:0]  : 16'd0;
      DATA_CACHE_ACCESS:  cache_din <= (opcode==3'b010)    ? rt_data           : 16'd0;
      default:          cache_din <= 16'd0;
    endcase
  end
end

always @(posedge clk or negedge rst_n) begin
  if(!rst_n) cache_WEB <= 1'b1;
  else begin
    case(state)
      FETCH_INST_DRAM: cache_WEB <= rvalid_m_inf[1] ? 1'b0 : 1'b1;
      FETCH_DATA_DRAM: cache_WEB <= rvalid_m_inf[0] ? 1'b0 : 1'b1;
      DATA_CACHE_ACCESS: cache_WEB <= (opcode==3'b010) ? 1'b0 : 1'b1;
      default:         cache_WEB <= 1'b1;
    endcase
  end
end

// ----------------------------- FSM -----------------------------
always @(posedge clk or negedge rst_n) begin
  if(!rst_n) state <= PREPARE;
  else       state <= state_next;
end

always @* begin
  case(state)
    PREPARE:          state_next = FETCH_INST_DRAM;
    FETCH_INST_DRAM:  state_next = rlast_m_inf[1] ? READ_INST_CACHE : FETCH_INST_DRAM;
    READ_INST_CACHE:  state_next = READ_INST_CACHE_D; //wait for SRAM delay
    READ_INST_CACHE_D:state_next = ID_STAGE; 
    ID_STAGE:         state_next = ID_STAGE_D1;
    ID_STAGE_D1:      state_next = EXE_STAGE;
    EXE_STAGE: begin
      case(opcode)
        3'b000:    state_next = OUTPUT; //add, sub
        3'b001:    state_next = OUTPUT; //slt, mult
        3'b011:          state_next = data_cache_miss_flag ? PREPARE_DATA : DATA_CACHE_ACCESS; //load
        3'b010:         state_next = DRAM_WRITE; //store
        default:       state_next = OUTPUT;
      endcase
    end
    PREPARE_DATA:     state_next = FETCH_DATA_DRAM;
    FETCH_DATA_DRAM:  state_next = rlast_m_inf[0] ? DATA_CACHE_ACCESS : FETCH_DATA_DRAM;
    DATA_CACHE_ACCESS:    state_next = (opcode==3'b010) ? OUTPUT : DATA_CACHE_ACCESS_D1;
    DATA_CACHE_ACCESS_D1: state_next = OUTPUT;
    DRAM_WRITE:       state_next = bvalid_m_inf ? ((dc < data_idx_lb_r || dc > data_idx_ub_r) ? OUTPUT : DATA_CACHE_ACCESS) : DRAM_WRITE;
    OUTPUT:           state_next = inst_cache_miss_flag ? PREPARE : READ_INST_CACHE;
    default:          state_next = state;
  endcase
end

// ----------------------------- IF -----------------------------
always @(posedge clk or negedge rst_n) begin
  if(!rst_n) pc <= 16'h1000;
  else begin
    case(state)
      ID_STAGE: pc <= pc + 16'd2;
      EXE_STAGE: begin
        if(opcode==3'b101 && rs_data==rt_data) //BEQ
          pc <= pc + ({{11{imme[4]}},imme} <<< 1);  //<<<1 : *2 to 2 bytes 
        else if(opcode==3'b100) //JUMP
          pc <= {3'b000, jump_addr};
      end
    endcase
  end
end

always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
    cache_idx_lb_r <= 16'h1000;
    cache_idx_ub_r <= 16'h10FE; //16'h1000 + 254
  end else begin
    cache_idx_lb_r <= cache_idx_lb;
    cache_idx_ub_r <= cache_idx_ub;
  end
end

always @(*)begin
  case(state)
    PREPARE: begin
      if(pc < 16'h1040)       begin cache_idx_lb=16'h1000; cache_idx_ub=16'h10FE; end
      else if(pc > 16'h2F40)  begin cache_idx_lb=16'h2F00; cache_idx_ub=16'h2FFE; end
      else                    begin cache_idx_lb=pc-16'd64;  cache_idx_ub=pc+16'd190; end //32 instruction, pc, 95 instruction
    end                                                                                   //64('h40) bytes, 2 bytes, 130('hBE) bytes
    default: begin cache_idx_lb=cache_idx_lb_r; cache_idx_ub=cache_idx_ub_r; end
  endcase
end

// ----------------------------- ID -----------------------------
always @(posedge clk or negedge rst_n) begin
  if(!rst_n) inst <= 16'd0;
  else if(state==ID_STAGE) inst <= cache_dout;
end
assign opcode    = inst[15:13];
assign rs_idx    = inst[12:9];
assign rt_idx    = inst[8:5];
assign rd_idx    = inst[4:1];
assign inst_func = inst[0];
assign jump_addr = inst[12:0];

always @(posedge clk or negedge rst_n) begin
  if(!rst_n) rs_data <= 16'd0;
  else if(state==ID_STAGE_D1) begin
    case(rs_idx)
      4'd0: rs_data<=core_r0 ; 4'd1: rs_data<=core_r1 ; 4'd2: rs_data<=core_r2 ; 4'd3: rs_data<=core_r3 ;
      4'd4: rs_data<=core_r4 ; 4'd5: rs_data<=core_r5 ; 4'd6: rs_data<=core_r6 ; 4'd7: rs_data<=core_r7 ;
      4'd8: rs_data<=core_r8 ; 4'd9: rs_data<=core_r9 ; 4'd10:rs_data<=core_r10; 4'd11:rs_data<=core_r11;
      4'd12:rs_data<=core_r12; 4'd13:rs_data<=core_r13; 4'd14:rs_data<=core_r14; default: rs_data<=core_r15;
    endcase
  end
end

always @(posedge clk or negedge rst_n) begin
  if(!rst_n) rt_data <= 16'd0;
  else if(state==ID_STAGE_D1) begin
    case(rt_idx)
      4'd0: rt_data<=core_r0 ; 4'd1: rt_data<=core_r1 ; 4'd2: rt_data<=core_r2 ; 4'd3: rt_data<=core_r3 ;
      4'd4: rt_data<=core_r4 ; 4'd5: rt_data<=core_r5 ; 4'd6: rt_data<=core_r6 ; 4'd7: rt_data<=core_r7 ;
      4'd8: rt_data<=core_r8 ; 4'd9: rt_data<=core_r9 ; 4'd10:rt_data<=core_r10; 4'd11:rt_data<=core_r11;
      4'd12:rt_data<=core_r12; 4'd13:rt_data<=core_r13; 4'd14:rt_data<=core_r14; default: rt_data<=core_r15;
    endcase
  end
end

always @(posedge clk or negedge rst_n) begin
  if(!rst_n) imme <= 5'd0;
  else if(state==ID_STAGE) imme <= cache_dout[4:0];  // 5-bit signed
end

// ----------------------------- EXE -----------------------------
always @(posedge clk or negedge rst_n) begin
  if(!rst_n) data_wb <= 16'd0;
  else begin
    case(state)
      EXE_STAGE: begin
        case(opcode)
          3'b000:   data_wb <= (inst_func==1'b1) ? (rs_data + rt_data) : (rs_data - rt_data);
          3'b001:   data_wb <= (inst_func==1'b1) ? ( (rs_data<rt_data)?16'd1:16'd0 ) : (rs_data*rt_data);
          default:      data_wb <= data_wb;
        endcase
      end
    endcase
  end
end

always @(posedge clk or negedge rst_n) begin
  if(!rst_n) reg_wb <= 4'd0;
  else       reg_wb <= opcode[1] ? rt_idx : rd_idx; 
end

wire signed [15:0] imm_sext = {{11{imme[4]}}, imme};

// calcualte dram address: (rs + imm) * 2 + 0x1000
assign OFFSET = 16'h1000;
assign dc_t   = ( ( $signed(rs_data) + imm_sext ) <<< 1 ) + OFFSET;
assign data_cache_miss_flag = (dc_t < data_idx_lb_r) || (dc_t > data_idx_ub_r);

always @(posedge clk or negedge rst_n) begin
  if(!rst_n) dc <= 16'd0;
  else if(state==EXE_STAGE) begin
    case(opcode)
      3'b011 : dc <= dc_t;
      3'b010 : dc <= dc_t;
    endcase
  end
end

always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
    data_idx_lb_r <= 16'h1000;
    data_idx_ub_r <= 16'h0000;
  end else begin
    data_idx_lb_r <= data_idx_lb;
    data_idx_ub_r <= data_idx_ub;
  end
end

always @(*) begin
  case(state)
    PREPARE_DATA: begin
      if(dc < 16'h107E)      begin data_idx_lb=16'h1000; data_idx_ub=16'h10FE; end
      else if(dc > 16'h2F7E) begin data_idx_lb=16'h2F00; data_idx_ub=16'h2FFE; end
      else                   begin data_idx_lb=dc-16'd126; data_idx_ub=dc+16'd128; end
    end
    default: begin data_idx_lb=data_idx_lb_r; data_idx_ub=data_idx_ub_r; end
  endcase
end

// ----------------------------- WB -----------------------------
assign inst_cache_miss_flag = (pc < cache_idx_lb_r) || (pc > cache_idx_ub_r);

always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
    core_r0<=0; core_r1<=0; core_r2<=0; core_r3<=0; core_r4<=0; core_r5<=0; core_r6<=0; core_r7<=0;
    core_r8<=0; core_r9<=0; core_r10<=0; core_r11<=0; core_r12<=0; core_r13<=0; core_r14<=0; core_r15<=0;
  end else if(state==OUTPUT) begin
    if(opcode == 3'b000 || opcode == 3'b001) begin
      case(reg_wb)
        4'd0: core_r0  <= data_wb; 4'd1: core_r1  <= data_wb; 4'd2: core_r2  <= data_wb; 4'd3: core_r3  <= data_wb;
        4'd4: core_r4  <= data_wb; 4'd5: core_r5  <= data_wb; 4'd6: core_r6  <= data_wb; 4'd7: core_r7  <= data_wb;
        4'd8: core_r8  <= data_wb; 4'd9: core_r9  <= data_wb; 4'd10:core_r10 <= data_wb; 4'd11:core_r11 <= data_wb;
        4'd12:core_r12 <= data_wb; 4'd13:core_r13 <= data_wb; 4'd14:core_r14 <= data_wb; default: core_r15 <= data_wb;
      endcase
    end
    else if(opcode == 3'b011) begin //LOAD
      case(reg_wb)
        4'd0: core_r0  <= cache_dout; 4'd1: core_r1  <= cache_dout; 4'd2: core_r2  <= cache_dout; 4'd3: core_r3  <= cache_dout;
        4'd4: core_r4  <= cache_dout; 4'd5: core_r5  <= cache_dout; 4'd6: core_r6  <= cache_dout; 4'd7: core_r7  <= cache_dout;
        4'd8: core_r8  <= cache_dout; 4'd9: core_r9  <= cache_dout; 4'd10:core_r10 <= cache_dout; 4'd11:core_r11 <= cache_dout;
        4'd12:core_r12 <= cache_dout; 4'd13:core_r13 <= cache_dout; 4'd14:core_r14 <= cache_dout; default: core_r15 <= cache_dout;
      endcase
    end
  end
end

//####################################################
//                        AXI
//####################################################


// -------- Read channel constants (2 DRAM{inst, data}) --------
assign arid_m_inf    = {4'd0, 4'd0};
assign arsize_m_inf  = {3'b001, 3'b001};     // 16-bit
assign arburst_m_inf = {2'b01 , 2'b01 };     // INCR
assign arlen_m_inf   = {7'd127, 7'd127};     // 128 beats

// -------- Write channel constants --------
assign awlen_m_inf   = 7'd0;                 // single beat
assign awid_m_inf    = {ID_WIDTH{1'b0}};
assign awsize_m_inf  = 3'b001;               // 16-bit
assign awburst_m_inf = 2'b01;                // INCR

// -------- AXI Submodules --------
READ_INST_DRAM AXI_READ_INST (
  .clk(clk), .rst_n(rst_n),
  .state(state),
  .cache_idx_lb(cache_idx_lb),
  .arready_m_inf(arready_m_inf[1]),
  .rlast_m_inf(rlast_m_inf[1]),
  .araddr_m_inf(araddr_m_inf[63:32]),
  .arvalid_m_inf(arvalid_m_inf[1]),
  .rready_m_inf(rready_m_inf[1])
);

READ_DATA_DRAM AXI_READ_DATA (
  .clk(clk), .rst_n(rst_n),
  .state(state),
  .data_idx_lb(data_idx_lb),
  .arready_m_inf(arready_m_inf[0]),
  .rlast_m_inf(rlast_m_inf[0]),
  .araddr_m_inf(araddr_m_inf[31:0]),
  .arvalid_m_inf(arvalid_m_inf[0]),
  .rready_m_inf(rready_m_inf[0])
);

WRITE_DATA_DRAM #(
  .ADDR_WIDTH(ADDR_WIDTH),
  .DATA_WIDTH(DATA_WIDTH),
  .WRIT_NUMBER(WRIT_NUMBER)
) AXI_WRITE_DATA (
  .clk(clk), .rst_n(rst_n),
  .state(state),
  .opcode(opcode),
  .dc_t(dc_t),
  .rt_data(rt_data),
  .awaddr_m_inf(awaddr_m_inf),
  .awvalid_m_inf(awvalid_m_inf),
  .awready_m_inf(awready_m_inf),
  .wdata_m_inf(wdata_m_inf),
  .wlast_m_inf(wlast_m_inf),
  .wvalid_m_inf(wvalid_m_inf),
  .wready_m_inf(wready_m_inf),
  .bvalid_m_inf(bvalid_m_inf),
  .bready_m_inf(bready_m_inf)
);


// ----------------------------- OUTPUT -----------------------------
always @(posedge clk or negedge rst_n) begin
  if(!rst_n) IO_stall <= 1'b1;
  else       IO_stall <= (state==OUTPUT) ? 1'b0 : 1'b1;
end

endmodule

// ====================================================================
// READ_INST_DRAM : if cache miss, request DRAM for new instreuction
// ====================================================================
module READ_INST_DRAM(  
  input clk, input rst_n,
  input [3:0] state,
  input [15:0] cache_idx_lb,
  input        arready_m_inf,
  input        rlast_m_inf,
  output reg [31:0] araddr_m_inf,
  output reg        arvalid_m_inf,
  output reg        rready_m_inf
);
  parameter PREPARE = 0;
  reg ar_busy;

  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
      ar_busy<=0; arvalid_m_inf<=0; araddr_m_inf<=32'd0;
    end else begin
      if(!ar_busy && state==PREPARE) begin
        araddr_m_inf  <= {16'd0, cache_idx_lb};
        arvalid_m_inf <= 1'b1;
        ar_busy       <= 1'b1;
      end
      if(arvalid_m_inf && arready_m_inf)
        arvalid_m_inf <= 1'b0;
      if(rready_m_inf && rlast_m_inf)
        ar_busy <= 1'b0;
    end
  end

  always @(posedge clk or negedge rst_n) begin
    if(!rst_n)          rready_m_inf <= 1'b0;
    else if(ar_busy)    rready_m_inf <= 1'b1;
    else                rready_m_inf <= 1'b0;
  end
endmodule

// ====================================================================
// READ_DATA_DRAM 
// ====================================================================
module READ_DATA_DRAM(
  input clk, input rst_n,
  input [3:0] state,
  input [15:0] data_idx_lb,
  input        arready_m_inf,
  input        rlast_m_inf,
  output reg [31:0] araddr_m_inf,
  output reg        arvalid_m_inf,
  output reg        rready_m_inf
);
  parameter PREPARE_DATA = 7;
  reg ar_busy;

  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
      ar_busy<=0; arvalid_m_inf<=0; araddr_m_inf<=32'd0;
    end else begin
      if(!ar_busy && state==PREPARE_DATA) begin
        araddr_m_inf  <= {16'd0, data_idx_lb};
        arvalid_m_inf <= 1'b1;
        ar_busy       <= 1'b1;
      end
      if(arvalid_m_inf && arready_m_inf)
        arvalid_m_inf <= 1'b0;
      if(rready_m_inf && rlast_m_inf)
        ar_busy <= 1'b0;
    end
  end

  always @(posedge clk or negedge rst_n) begin
    if(!rst_n)          rready_m_inf <= 1'b0;
    else if(ar_busy)    rready_m_inf <= 1'b1;
    else                rready_m_inf <= 1'b0;
  end
endmodule

// ====================================================================
// WRITE_DATA_DRAM 
// ====================================================================
module WRITE_DATA_DRAM #(
  parameter ADDR_WIDTH = 32,
  parameter DATA_WIDTH = 16,
  parameter WRIT_NUMBER = 1
)(
  input  wire clk,
  input  wire rst_n,
  input  wire [3:0]  state,
  input  wire [2:0]  opcode,
  input  wire [15:0] dc_t,
  input  wire signed [15:0] rt_data,
  
  // AXI Write Address Channel
  output reg  [WRIT_NUMBER*ADDR_WIDTH-1:0] awaddr_m_inf,
  output reg  [WRIT_NUMBER-1:0]            awvalid_m_inf,
  input  wire [WRIT_NUMBER-1:0]            awready_m_inf,
  
  // AXI Write Data Channel
  output reg  [WRIT_NUMBER*DATA_WIDTH-1:0] wdata_m_inf,
  output reg  [WRIT_NUMBER-1:0]            wlast_m_inf,
  output reg  [WRIT_NUMBER-1:0]            wvalid_m_inf,
  input  wire [WRIT_NUMBER-1:0]            wready_m_inf,
  
  // AXI Write Response Channel
  input  wire [WRIT_NUMBER-1:0]            bvalid_m_inf,
  output reg  [WRIT_NUMBER-1:0]            bready_m_inf
);

  parameter EXE_STAGE = 6;
  reg wr_busy, aw_done, w_done, b_done;

  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
      wr_busy       <= 1'b0; 
      aw_done       <= 1'b0; 
      w_done        <= 1'b0; 
      b_done        <= 1'b0;
      awaddr_m_inf  <= 0; 
      awvalid_m_inf <= 1'b0;
      wdata_m_inf   <= 0;  
      wvalid_m_inf  <= 1'b0; 
      wlast_m_inf   <= 1'b0;
      bready_m_inf  <= 1'b0;
    end else begin
      // Trigger Write Request at EXE_STAGE when instruction is STORE (3'b010)
      if(state == EXE_STAGE && opcode == 3'b010 && !wr_busy) begin
        wr_busy       <= 1'b1;
        awaddr_m_inf  <= { {(ADDR_WIDTH-16){1'b0}}, dc_t }; 
        awvalid_m_inf <= 1'b1;
        aw_done       <= 1'b0;
        w_done        <= 1'b0;
        b_done        <= 1'b0;
      end

      // AW Channel Handshake
      if(wr_busy && !aw_done) begin
        if(awvalid_m_inf && awready_m_inf) begin
          awvalid_m_inf <= 1'b0;
          aw_done       <= 1'b1;
        end
      end

      // W Channel Handshake
      if(wr_busy && aw_done && !w_done) begin
        wvalid_m_inf <= 1'b1;
        wlast_m_inf  <= 1'b1;
        wdata_m_inf  <= rt_data;
        if(wvalid_m_inf && wready_m_inf) begin
          wvalid_m_inf <= 1'b0;
          wlast_m_inf  <= 1'b0;
          w_done       <= 1'b1;
        end
      end

      // B Channel Handshake
      if(wr_busy && aw_done && w_done && !b_done) begin
        bready_m_inf <= 1'b1;
        if(bvalid_m_inf && bready_m_inf) begin
          bready_m_inf <= 1'b0;
          b_done       <= 1'b1;
        end
      end

      // Write Transaction Complete
      if(wr_busy && aw_done && w_done && b_done) begin
        wr_busy <= 1'b0;
      end
    end
  end

endmodule