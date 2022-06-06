`timescale 1ns/10ps

`define CPU_BITS 32
`define CPU_REGS 32
`define CPU_ALUW 4
`define REG_WIDTH 5

`define ACTIVE_LOW 0
`define INACTIVE_HIGH 1

`define RTYPE 51
`define ITYPE 3
`define STYPE 35
`define BTYPE 32
`define UTYPE 42
`define SBTYPE 99

`define AND 0
`define OR 1
`define ADD 2
`define SLL 3
`define SLT 4
`define SLTU 5
`define SUB 6
`define XOR 7
`define SRL 8
`define SRA 9


module clock (
    clk
);

output [0:0] clk;
reg [0:0] clk;

always #10 clk=~clk;

endmodule

module pc_mux (
    reset,
    pc,
    pc_addr,
    jmp_addr,
    pc_sel
);

input reset;
output [31:0] pc;
reg [31:0] pc;
input [31:0] pc_addr;
input [31:0] jmp_addr;
input [0:0] pc_sel;

always @(reset, pc_sel, jmp_addr, pc_addr) begin: PC_MUX_PMUX
    if ((reset == 1)) begin
        if (pc_sel) begin
            pc = jmp_addr;
        end
        else begin
            pc = pc_addr;
        end
    end
end

endmodule

module wda_mux (
    reset,
    wda,
    mem_to_rgs,
    result,
    read_data
);

input reset;
output [31:0] wda;
reg [31:0] wda;
input [0:0] mem_to_rgs;
input [31:0] result;
input [31:0] read_data;

always @(reset, mem_to_rgs, read_data, result) begin: WDA_MUX_WMUX
    if ((reset == 1)) begin
        if (mem_to_rgs) begin
            wda = read_data;
        end
        else begin
            wda = result;
        end
    end
end

endmodule

module alu_mux (
    reset,
    im_gen,
    rdb,
    rdx,
    alu_src
);

input reset;
input [31:0] im_gen;
input [31:0] rdb;
output [31:0] rdx;
reg [31:0] rdx;
input [0:0] alu_src;

always @(reset, im_gen, alu_src, rdb) begin: ALU_MUX_AMUX
    if ((reset == 1)) begin
        if (alu_src) begin
            rdx = im_gen;
        end
        else begin
            rdx = rdb;
        end
    end
end

endmodule

module taken (
    result,
    brnch,
    pc_sel
);

input [31:0] result;
input [0:0] brnch;
output [0:0] pc_sel;
reg [0:0] pc_sel;

always @(brnch, result) begin: TAKEN_TAKE
    if (((result == 0) & brnch)) begin
        pc_sel = 1'b1;
    end
    else begin
        pc_sel = 1'b0;
    end
end

endmodule

module reg_file (
    reset,
    clk,
    ra,
    rb,
    wa,
    wda,
    reg_wr,
    rda,
    rdb
);

input reset;
input [0:0] clk;
input [4:0] ra;
input [4:0] rb;
input [4:0] wa;
input [31:0] wda;
input [0:0] reg_wr;
output [31:0] rda;
reg [31:0] rda;
output [31:0] rdb;
reg [31:0] rdb;

reg [31:0] registers [0:32-1];



always @(reset, rb, ra, registers[0], registers[1], registers[2], registers[3], registers[4], registers[5], registers[6], registers[7], registers[8], registers[9], registers[10], registers[11], registers[12], registers[13], registers[14], registers[15], registers[16], registers[17], registers[18], registers[19], registers[20], registers[21], registers[22], registers[23], registers[24], registers[25], registers[26], registers[27], registers[28], registers[29], registers[30], registers[31]) begin: REG_FILE_READ
    if ((reset == 1)) begin
        if (ra) begin
            rda = registers[ra];
        end
        if (rb) begin
            rdb = registers[rb];
        end
    end
end


always @(posedge clk) begin: REG_FILE_WRITE
    if ((reset == 1)) begin
        if (reg_wr) begin
            if (wa) begin
                registers[wa] <= wda;
            end
        end
    end
end

endmodule

module decode (
    reset,
    ifid_reg,
    ra,
    rb,
    wa,
    opcode
);

input reset;
input [63:0] ifid_reg;
output [4:0] ra;
reg [4:0] ra;
output [4:0] rb;
reg [4:0] rb;
output [4:0] wa;
reg [4:0] wa;
output [6:0] opcode;
reg [6:0] opcode;

always @(reset, ifid_reg) begin: DECODE_DCODE
    if ((reset == 1)) begin
        if ((ifid_reg[7-1:0] == 51)) begin
            ra = ifid_reg[20-1:15];
            rb = ifid_reg[25-1:20];
            opcode = ifid_reg[7-1:0];
            wa = ifid_reg[12-1:7];
        end
        else if ((ifid_reg[7-1:0] == 3)) begin
            ra = ifid_reg[20-1:15];
            opcode = ifid_reg[7-1:0];
            wa = ifid_reg[12-1:7];
        end
        else if ((ifid_reg[7-1:0] == 35)) begin
            ra = ifid_reg[20-1:15];
            rb = ifid_reg[25-1:20];
            opcode = ifid_reg[7-1:0];
            wa = ifid_reg[25-1:20];
        end
        else if ((ifid_reg[7-1:0] == 99)) begin
            ra = ifid_reg[20-1:15];
            rb = ifid_reg[25-1:20];
            opcode = ifid_reg[7-1:0];
        end
    end
end

endmodule

module idex_pipl (
    reset,
    idex_reg,
    instruction,
    ra,
    rb,
    wa,
    im_gen,
    rda,
    rdb,
    alu_op,
    brnch,
    mem_rd,
    mem_to_rgs,
    mem_wr,
    alu_src,
    reg_wr
);

input reset;
output [152:0] idex_reg;
reg [152:0] idex_reg;
input [31:0] instruction;
input [4:0] ra;
input [4:0] rb;
input [4:0] wa;
input [31:0] im_gen;
input [31:0] rda;
input [31:0] rdb;
input [3:0] alu_op;
input [0:0] brnch;
input [0:0] mem_rd;
input [0:0] mem_to_rgs;
input [0:0] mem_wr;
input [0:0] alu_src;
input [0:0] reg_wr;

always @(rdb, alu_op, instruction, im_gen, wa, alu_src, reset, rb, brnch, reg_wr, mem_to_rgs, rda, mem_rd, ra, mem_wr) begin: IDEX_PIPL_ID_EX
    if ((reset == 1)) begin
        idex_reg[32-1:0] = instruction;
        idex_reg[(32 + 5)-1:32] = ra;
        idex_reg[((2 * 5) + 32)-1:(5 + 32)] = rb;
        idex_reg[((3 * 5) + 32)-1:((2 * 5) + 32)] = wa;
        idex_reg[((3 * 5) + (2 * 32))-1:((3 * 5) + 32)] = im_gen;
        idex_reg[((3 * 5) + (3 * 32))-1:((3 * 5) + (2 * 32))] = rda;
        idex_reg[((3 * 5) + (4 * 32))-1:((3 * 5) + (3 * 32))] = rdb;
        idex_reg[(((3 * 5) + (4 * 32)) + 4)-1:((3 * 5) + (4 * 32))] = alu_op;
        idex_reg[(((1 + (3 * 5)) + (4 * 32)) + 4)-1:(((0 + (3 * 5)) + (4 * 32)) + 4)] = brnch;
        idex_reg[(((2 + (3 * 5)) + (4 * 32)) + 4)-1:(((1 + (3 * 5)) + (4 * 32)) + 4)] = mem_rd;
        idex_reg[(((3 + (3 * 5)) + (4 * 32)) + 4)-1:(((2 + (3 * 5)) + (4 * 32)) + 4)] = mem_to_rgs;
        idex_reg[(((4 + (3 * 5)) + (4 * 32)) + 4)-1:(((3 + (3 * 5)) + (4 * 32)) + 4)] = mem_wr;
        idex_reg[(((5 + (3 * 5)) + (4 * 32)) + 4)-1:(((4 + (3 * 5)) + (4 * 32)) + 4)] = alu_src;
        idex_reg[(((6 + (3 * 5)) + (4 * 32)) + 4)-1:(((5 + (3 * 5)) + (4 * 32)) + 4)] = reg_wr;
    end
end

endmodule

module ifid_pipl (
    reset,
    ifid_reg,
    instruction,
    pc
);

input reset;
output [63:0] ifid_reg;
reg [63:0] ifid_reg;
input [31:0] instruction;
input [31:0] pc;

always @(reset, pc, instruction) begin: IFID_PIPL_IF_ID
    if ((reset == 1)) begin
        ifid_reg[(32 + 32)-1:32] = pc;
        ifid_reg[32-1:0] = instruction;
    end
end

endmodule

module alu (
    reset,
    alu_decode,
    idex_reg,
    rdx,
    result
);

input reset;
input [3:0] alu_decode;
input [152:0] idex_reg;
input [31:0] rdx;
output [31:0] result;
reg [31:0] result;

always @(reset, alu_decode, idex_reg, rdx) begin: ALU_OPERATION
    if ((reset == 1)) begin
        case (alu_decode)
            'h0: begin
                result = (idex_reg[((3 * 5) + (3 * 32))-1:((3 * 5) + (2 * 32))] & rdx);
            end
            'h1: begin
                result = (idex_reg[((3 * 5) + (3 * 32))-1:((3 * 5) + (2 * 32))] | rdx);
            end
            'h2: begin
                result = (idex_reg[((3 * 5) + (3 * 32))-1:((3 * 5) + (2 * 32))] + rdx);
            end
            'h6: begin
                result = (idex_reg[((3 * 5) + (3 * 32))-1:((3 * 5) + (2 * 32))] - rdx);
            end
            'h7: begin
                result = (idex_reg[((3 * 5) + (3 * 32))-1:((3 * 5) + (2 * 32))] ^ rdx);
            end
            'h3: begin
                result = (idex_reg[((3 * 5) + (3 * 32))-1:((3 * 5) + (2 * 32))] << rdx);
            end
            'h8: begin
                result = $signed($signed(idex_reg[((3 * 5) + (3 * 32))-1:((3 * 5) + (2 * 32))]) >>> rdx);
            end
            'h4: begin
                result = ($signed(idex_reg[((3 * 5) + (3 * 32))-1:((3 * 5) + (2 * 32))]) < $signed(rdx)) ? 1'b1 : 1'b0;
            end
            'h5: begin
                result = (idex_reg[((3 * 5) + (3 * 32))-1:((3 * 5) + (2 * 32))] < rdx) ? 1'b1 : 1'b0;
            end
            'h9: begin
                if ((idex_reg[((3 * 5) + (3 * 32))-1:((3 * 5) + (2 * 32))][31] == 0)) begin
                    result = $signed($signed(idex_reg[((3 * 5) + (3 * 32))-1:((3 * 5) + (2 * 32))]) >>> rdx);
                end
                else if ((idex_reg[((3 * 5) + (3 * 32))-1:((3 * 5) + (2 * 32))][31] == 1)) begin
                    result = $signed($signed(idex_reg[((3 * 5) + (3 * 32))-1:((3 * 5) + (2 * 32))]) >>> rdx);
                    result[32-1:(31 - $signed({1'b0, rdx}))] = ((2 ** rdx) - 1);
                end
            end
        endcase
    end
end

endmodule

module alu_control (
    reset,
    instruction,
    alu_op,
    alu_decode
);

input reset;
input [31:0] instruction;
input [3:0] alu_op;
output [3:0] alu_decode;
reg [3:0] alu_decode;

always @(instruction, reset, alu_op) begin: ALU_CONTROL_ALUCONT
    if ((reset == 1)) begin
        case (alu_op)
            'h2: begin
                if ((instruction[32-1:25] == 0)) begin
                    if ((instruction[15-1:12] == 0)) begin
                        alu_decode = 2;
                    end
                    else if ((instruction[15-1:12] == 1)) begin
                        alu_decode = 3;
                    end
                    else if ((instruction[15-1:12] == 2)) begin
                        alu_decode = 4;
                    end
                    else if ((instruction[15-1:12] == 3)) begin
                        alu_decode = 5;
                    end
                    else if ((instruction[15-1:12] == 4)) begin
                        alu_decode = 7;
                    end
                    else if ((instruction[15-1:12] == 5)) begin
                        alu_decode = 8;
                    end
                    else if ((instruction[15-1:12] == 6)) begin
                        alu_decode = 1;
                    end
                    else if ((instruction[15-1:12] == 7)) begin
                        alu_decode = 0;
                    end
                end
                else if ((instruction[32-1:25] == 32)) begin
                    if ((instruction[15-1:12] == 0)) begin
                        alu_decode = 6;
                    end
                    else if ((instruction[15-1:12] == 5)) begin
                        alu_decode = 9;
                    end
                end
            end
            'h0: begin
                alu_decode = 2;
            end
            'h7: begin
                if ((instruction[15-1:12] == 0)) begin
                    alu_decode = 7;
                end
            end
        endcase
    end
end

endmodule

module imm_gen (
    reset,
    instruction,
    im_gen
); 

input reset;
input [31:0] instruction;
output [31:0] im_gen;
reg [31:0] im_gen;

always @(reset, instruction) begin: IMM_GEN_IMMGEN
    if((reset==1)) begin
        if ((instruction[7-1:0] == 3)) begin
            im_gen[12-1:0] = instruction[32-1:20];
        end
        else if ((instruction[7-1:0] == 35)) begin
            im_gen[12-1:5] = instruction[32-1:25];
            im_gen[5-1:0] = instruction[12-1:7];
        end
        else if ((instruction[7-1:0] == 99)) begin
            im_gen[12] = instruction[31];
            im_gen[11-1:5] = instruction[31-1:25];
            im_gen[11] = instruction[7];
            im_gen[5-1:1] = instruction[12-1:8];
            im_gen[0] = 0;
        end  
        if ((instruction[31] == 0)) begin
            im_gen[32-1:(31 - 20)] = (0);
        end
        else begin
            im_gen[32-1:(31 - 20)] = ((2 ** 20) - 1);
        end
    end
end

endmodule    

module control (
    reset,
    opcode,
    brnch,
    mem_rd,
    mem_to_rgs,
    alu_op,
    mem_wr,
    alu_src,
    reg_wr
);


input reset;
input [6:0] opcode;
output [0:0] brnch;
reg [0:0] brnch;
output [0:0] mem_rd;
reg [0:0] mem_rd;
output [0:0] mem_to_rgs;
reg [0:0] mem_to_rgs;
output [3:0] alu_op;
reg [3:0] alu_op;
output [0:0] mem_wr;
reg [0:0] mem_wr;
output [0:0] alu_src;
reg [0:0] alu_src;
output [0:0] reg_wr;
reg [0:0] reg_wr;

wire [0:0] clk;
wire [0:0] step;

assign clk = 1'd0;
assign step = 1'd0;


always @(reset, opcode) begin: CONTROL_CONT
    if ((reset == 1)) begin
        case (opcode)
            'h33: begin
                alu_src = 1'b0;
                mem_to_rgs = 1'b0;
                reg_wr = 1'b1;
                mem_rd = 1'b0;
                mem_wr = 1'b0;
                brnch = 1'b0;
                alu_op = 2;
            end
            'h3: begin
                alu_src = 1'b1;
                mem_to_rgs = 1'b1;
                reg_wr = 1'b1;
                mem_rd = 1'b1;
                mem_wr = 1'b0;
                brnch = 1'b0;
                alu_op = 0;
            end
            'h23: begin
                alu_src = 1'b1;
                mem_to_rgs = 1'b0;
                reg_wr = 1'b0;
                mem_rd = 1'b0;
                mem_wr = 1'b1;
                brnch = 1'b0;
                alu_op = 0;
            end
            'h63: begin
                alu_src = 1'b0;
                mem_to_rgs = 1'b0;
                reg_wr = 1'b0;
                mem_rd = 1'b0;
                mem_wr = 1'b0;
                brnch = 1'b1;
                alu_op = 7;
            end
        endcase
    end
end

endmodule

module data_mem (
    reset,
    clk, 
    result, 
    mem_wr, 
    mem_rd, 
    rdb, 
    read_data
 );

 input reset;
 input [0:0] clk;
 input [31:0] result;
 input [0:0] mem_wr;
 input [0:0] mem_rd;
 input [31:0] rdb;
 output [31:0] read_data;
 reg [31:0] read_data;


 integer k;
 wire [31:0] data_ram [0:128-1];

 initial 
   begin 
    $readmemb("mc_data", data_ram); 
   end

 always @(posedge clk, data_ram[0], data_ram[1], data_ram[2], data_ram[3], data_ram[4], data_ram[5], data_ram[6], data_ram[7], data_ram[8], data_ram[9], data_ram[10], data_ram[11], data_ram[12], data_ram[13], data_ram[14], data_ram[15], data_ram[16], data_ram[17], data_ram[18], data_ram[19], data_ram[20], data_ram[21], data_ram[22], data_ram[23], data_ram[24], data_ram[25], data_ram[26], data_ram[27], data_ram[28], data_ram[29], data_ram[30], data_ram[31], data_ram[32], data_ram[33], data_ram[34], data_ram[35], data_ram[36], data_ram[37], data_ram[38], data_ram[39], data_ram[40], data_ram[41], data_ram[42], data_ram[43], data_ram[44], data_ram[45], data_ram[46], data_ram[47], data_ram[48], data_ram[49], data_ram[50], data_ram[51], data_ram[52], data_ram[53], data_ram[54], data_ram[55], data_ram[56], data_ram[57], data_ram[58], data_ram[59], data_ram[60], data_ram[61], data_ram[62], data_ram[63], data_ram[64], data_ram[65], data_ram[66], data_ram[67], data_ram[68], data_ram[69], data_ram[70], data_ram[71], data_ram[72], data_ram[73], data_ram[74], data_ram[75], data_ram[76], data_ram[77], data_ram[78], data_ram[79], data_ram[80], data_ram[81], data_ram[82], data_ram[83], data_ram[84], data_ram[85], data_ram[86], data_ram[87], data_ram[88], data_ram[89], data_ram[90], data_ram[91], data_ram[92], data_ram[93], data_ram[94], data_ram[95], data_ram[96], data_ram[97], data_ram[98], data_ram[99], data_ram[100], data_ram[101], data_ram[102], data_ram[103], data_ram[104], data_ram[105], data_ram[106], data_ram[107], data_ram[108], data_ram[109], data_ram[110], data_ram[111], data_ram[112], data_ram[113], data_ram[114], data_ram[115], data_ram[116], data_ram[117], data_ram[118], data_ram[119], data_ram[120], data_ram[121], data_ram[122], data_ram[123], data_ram[124], data_ram[125], data_ram[126], data_ram[127])
 begin: DATA_MEM_DTCM
  if ((reset == 1)) 
  begin
    if((mem_wr))
    begin
        
        data_ram[result] = rdb;
    end
    else if((mem_rd))
    begin
        k= data_ram[result];
        read_data=k; 
    end
  end
 end
endmodule

module inst_mem (
    reset,
    read_addr,
    instruction
);


input reset;
input [31:0] read_addr;
output [31:0] instruction;
reg [31:0] instruction;

wire [31:0] inst_ram [0:128-1];

integer j; 
 
initial 
  begin 
    $readmemb("mc_code", inst_ram); 
  end


always @(reset, read_addr, inst_ram[0], inst_ram[1], inst_ram[2], inst_ram[3], inst_ram[4], inst_ram[5], inst_ram[6], inst_ram[7], inst_ram[8], inst_ram[9], inst_ram[10], inst_ram[11], inst_ram[12], inst_ram[13], inst_ram[14], inst_ram[15], inst_ram[16], inst_ram[17], inst_ram[18], inst_ram[19], inst_ram[20], inst_ram[21], inst_ram[22], inst_ram[23], inst_ram[24], inst_ram[25], inst_ram[26], inst_ram[27], inst_ram[28], inst_ram[29], inst_ram[30], inst_ram[31], inst_ram[32], inst_ram[33], inst_ram[34], inst_ram[35], inst_ram[36], inst_ram[37], inst_ram[38], inst_ram[39], inst_ram[40], inst_ram[41], inst_ram[42], inst_ram[43], inst_ram[44], inst_ram[45], inst_ram[46], inst_ram[47], inst_ram[48], inst_ram[49], inst_ram[50], inst_ram[51], inst_ram[52], inst_ram[53], inst_ram[54], inst_ram[55], inst_ram[56], inst_ram[57], inst_ram[58], inst_ram[59], inst_ram[60], inst_ram[61], inst_ram[62], inst_ram[63], inst_ram[64], inst_ram[65], inst_ram[66], inst_ram[67], inst_ram[68], inst_ram[69], inst_ram[70], inst_ram[71], inst_ram[72], inst_ram[73], inst_ram[74], inst_ram[75], inst_ram[76], inst_ram[77], inst_ram[78], inst_ram[79], inst_ram[80], inst_ram[81], inst_ram[82], inst_ram[83], inst_ram[84], inst_ram[85], inst_ram[86], inst_ram[87], inst_ram[88], inst_ram[89], inst_ram[90], inst_ram[91], inst_ram[92], inst_ram[93], inst_ram[94], inst_ram[95], inst_ram[96], inst_ram[97], inst_ram[98], inst_ram[99], inst_ram[100], inst_ram[101], inst_ram[102], inst_ram[103], inst_ram[104], inst_ram[105], inst_ram[106], inst_ram[107], inst_ram[108], inst_ram[109], inst_ram[110], inst_ram[111], inst_ram[112], inst_ram[113], inst_ram[114], inst_ram[115], inst_ram[116], inst_ram[117], inst_ram[118], inst_ram[119], inst_ram[120], inst_ram[121], inst_ram[122], inst_ram[123], inst_ram[124], inst_ram[125], inst_ram[126], inst_ram[127])
begin: INST_MEM_ITCM
    if ((reset == 1)) 
    begin
        j=inst_ram[read_addr];
        instruction = j;
    end
end

endmodule

module pc_adder (
    reset,
    step,
    pc,
    pc_addr
);

input reset;
input [0:0] step;
input [31:0] pc;
output [31:0] pc_addr;
reg [31:0] pc_addr;

always @(posedge step) begin: PC_ADDER_PADDER
    if ((reset == 1)) begin
        pc_addr <= (pc + 1);
    end
end

endmodule

module pc_assign (
    reset,
    read_addr,
    pc
);

input reset;
output [31:0] read_addr;
reg [31:0] read_addr;
input [31:0] pc;

always @(reset, pc) begin: PC_ASSIGN_ASSIGN
    if ((reset == 1)) begin
        read_addr = pc;
    end
end

endmodule

module jmp_adder (
    reset,
    read_addr,
    shl,
    jmp_addr
);


input reset;
input [31:0] read_addr;
input [31:0] shl;
output [31:0] jmp_addr;
reg [31:0] jmp_addr;

always @(read_addr, reset, shl) begin: JMP_ADDER_JADDER
    if ((reset == 1)) begin
        jmp_addr = (read_addr + shl);
    end
end

endmodule

module cpu_top (
    clk,
    reset
 );

 input [0:0] clk;
 input reset;

 reg [0:0] step;

 initial begin
  ra<={5{1'b0}};
  rb<={5{1'b0}};
  wa<={5{1'b0}};
  opcode <= {7{1'b0}};
  wda<= {32{1'b0}};
  rda<= {32{1'b0}};
  rdb<= {32{1'b0}};
  rdx<= {32{1'b0}};
  alu_op<= {4{1'b0}};
  brnch<=1'b0;
  mem_rd<=1'b0;
  mem_to_rgs<=1'b0;
  mem_wr<=1'b0;
  alu_src<=1'b0;
  reg_wr <=1'b0;
  step <= 1'b0;
  pc_sel <=1'b0;
  result<={32{1'b0}};
  read_data<= {32{1'b0}};
  pc<= {32{1'b0}};
  shl <= {32{1'b0}};
  im_gen <= {32{1'b0}};
  alu_decode <= {4{1'b0}};
  read_addr<= {32{1'b0}};
  instruction <= {32{1'b0}};
  pc_addr <= {32{1'b0}};
  jmp_addr <= {32{1'b0}};
 end
 control cont(reset, opcode, brnch, mem_rd, mem_to_rgs, alu_op, mem_wr, alu_src, reg_wr);
 data_mem dmem(reset, clk, result, mem_wr, mem_rd, rdb, read_data);
 inst_mem imem(reset, read_addr, instruction);
 alu alux(reset, alu_decode, rda, rdx, result);
 reg_file regf(reset, clk, ra, rb, wa, wda, reg_wr, rda, rdb);
 pc_adder padr(reset, step, pc, pc_addr);
 jmp_adder jadr(reset, read_addr, shl, jmp_addr);
 pc_mux pcmx(reset, pc, pc_addr, jmp_addr, pc_sel);
 alu_mux almx(reset, im_gen, rdb, rdx, alu_src);
 wda_mux wdmx(reset, wda, mem_to_rgs, result, read_data);
 alu_control aluc(reset, instruction, alu_op, alu_decode);
 imm_gen imgn(reset, instruction, im_gen);
 pc_assign nxpc(reset, read_addr, pc);
 taken tken(result, brnch, pc_sel);
 decode decd(reset, ifid_reg, ra, rb, wa, opcode);
 idex_pipl idxp(reset, idex_reg, instruction, ra, rb, wa, im_gen, rda, rdb, alu_op, brnch, mem_rd, mem_to_rgs, mem_wr, alu_src, reg_wr);
 ifid_pipl ifdp(reset, ifid_reg, instruction, pc);

 always @(posedge step)
 begin: CPU_TOP_CPU
    if ((pc == 1)) 
    begin
        pc<=pc+1;
    end
 end

 initial begin: CPU_TOP_EVENT
    integer idle;
    integer i;
    idle = 7;
    while (1'b1) begin
        for (i=0; i<idle; i=i+1) begin
            @(posedge clk);
        end
        if ((reset == 1)) begin
            step <= (!step);
            idle = 3;
        end
    end
 end

endmodule
