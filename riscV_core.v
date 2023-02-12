`timescale 1ns/100ps

`define CPU_BITS 32
`define CPU_REGS 32
`define ALU_OP_WIDTH 4
`define REG_WIDTH 5

`define ACTIVE_LOW 0
`define INACTIVE_HIGH 1

`define RTYPE 51
`define ITYPE 3
`define STYPE 35
`define BTYPE 32
`define UTYPE 42
`define JTYPE 99

`define ALU_OP_AND 0
`define ALU_OP_OR 1
`define ALU_OP_ADD 2
`define ALU_OP_SLL 3
`define ALU_OP_SLT 4
`define ALU_OP_SLTU 5
`define ALU_OP_SUB 6
`define ALU_OP_XOR 7
`define ALU_OP_SRL 8
`define ALU_OP_SRA 9

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

module clock(clk);
  output reg clk;

  always #5 clk <= ~clk;
endmodule

module pc_mux(
  input wire reset,
  output reg [31:0] pc,
  input wire [31:0] pc_addr,
  input wire [31:0] jmp_addr,  
  input wire pc_sel
);

  always @(reset, pc_sel, jmp_addr, pc_addr) begin
    if (reset == 1) begin
      pc <= 0;
    end 
    else begin
      if (pc_sel) begin
        pc <= jmp_addr;
      end else begin
        pc <= pc_addr;
      end
    end
  end
endmodule


module wda_mux (
  input wire reset,
  output reg [31:0] wda,
  input wire mem_to_rgs,
  input wire [31:0] result,
  input wire [31:0] read_data
);

always @(reset, mem_to_rgs, read_data, result) begin
  if (reset == 1) begin
  wda<=0;
  end
  if (reset == 0) begin
    if (mem_to_rgs)
      wda <= read_data;
    else
      wda <= result;
  end
end

endmodule

module alu_mux (
    input wire reset,
    input wire [31:0] im_gen,
    input wire [31:0] rdb,
    output reg [31:0] rdx,
    input wire alu_src
    
);
    
    always @(reset, im_gen, alu_src, rdb) begin
        if (reset == 1) begin
          rdx<=0;
        end
        if (reset == 0) begin
            if (alu_src) begin
                rdx <= im_gen;
            end else begin
                rdx <= rdb;
            end
        end
    end
endmodule


module taken(input wire [31:0] result, input wire brnch,output reg pc_sel);

always @(brnch, result) begin
    if (!result && brnch)
        pc_sel <= 1;
    else
        pc_sel <= 0;
end

endmodule


module reg_file(
  input wire reset,
  input wire clk,
  input wire [4:0] ra,
  input wire [4:0] rb,
  input wire [4:0] wa,
  input wire [31:0] wda,
  input wire reg_wr,
  output reg [31:0] rda,
  output reg [31:0] rdb
);
  reg [31:0] registers [0:31];
  integer i,file;
  always @(reset, rb, ra, registers) begin
    if(reset == 1) begin
      rda<=0;
      rdb<=0;
      for(i=0; i<32; i=i+1) begin
        registers[i]<=1;
      end
      
    end
    if (reset == 0) begin
      if (ra != 0) begin
        rda <= registers[ra];
      end
      if (rb != 0) begin
        rdb <= registers[rb];
      end
    end
  end
  
  always @(posedge clk) begin
    if (reset == 0) begin
      if (reg_wr && (wa > 0)) begin
        registers[wa] <= wda;

        file = $fopen("D:/PROJECTS BTECH/risc_cpu/riscVcpu9feb/output_riscV.txt", "a");
        $fwrite(file, "%d\n", wda);
      end
    end
  end
endmodule


module alu (
    input wire reset, 
    input wire [`ALU_OP_WIDTH-1:0] alu_decode, 
    input wire [`CPU_BITS-1:0] rda, 
    input wire [`CPU_BITS-1:0] rdx, 
    output reg [`CPU_BITS-1:0] result
);
    
    always @(reset, alu_decode, rda, rdx) begin
        if (reset==1) begin
          result <= 0;
        end
        if (reset == 0) begin
            case (alu_decode)
                `ALU_OP_AND: result <= rda & rdx;
                `ALU_OP_OR: result <= rda | rdx;
                `ALU_OP_ADD: result <= rda + rdx;
                `ALU_OP_SUB: result <= rda - rdx;
                `ALU_OP_XOR: result <= rda ^ rdx;
                `ALU_OP_SLL: result <= rda << rdx;
                `ALU_OP_SRL: result <= rda >> rdx;
                `ALU_OP_SLT: result <= (rda < rdx) ? 1 : 0;
                `ALU_OP_SLTU: result <= (rda < rdx) ? 1 : 0;
                `ALU_OP_SRA: result <= (rda[31] == 1) ? (rda >> rdx) | ((1 << (`CPU_BITS - rdx)) - 1) : (rda >> rdx);
                default: result <= 0;
            endcase

        end
    end
    
endmodule


module alu_control(
  input wire reset,
  input wire [31:0] instruction,
  input wire [3:0] alu_op,
  output reg [3:0] alu_decode
);
  always @(instruction, reset, alu_op) begin
    if(reset == 1) begin
      alu_decode<=0;
    end
    if (reset == 0) begin
      if (alu_op == 2) begin
        if (instruction[31:25] == 0) begin
          if (instruction[14:12] == 0) begin
            alu_decode <= `ADD;
          end else if (instruction[14:12] == 1) begin
            alu_decode <= `SLL;
          end else if (instruction[14:12] == 2) begin
            alu_decode <= `SLT;
          end else if (instruction[14:12] == 3) begin
            alu_decode <= `SLTU;
          end else if (instruction[14:12] == 4) begin
            alu_decode<= `XOR;
          end else if (instruction[14:12] == 5) begin
            alu_decode <= `SRL;
          end else if (instruction[14:12] == 6) begin
            alu_decode <= `OR;
          end else if (instruction[14:12] == 7) begin
            alu_decode <= `AND;
          end
        end else if (instruction[31:25] == 32) begin
          if (instruction[14:12] == 0) begin
            alu_decode <= `SUB;
          end else if (instruction[14:12] == 5) begin
            alu_decode <= `SRA;
          end
        end
      end else if (alu_op == 0) begin
        alu_decode <= `ADD;
      end else if (alu_op == 7) begin
        if (instruction[14:12] == 0) begin
          alu_decode <= `XOR;
        end
      end
    end
  end
endmodule


module control (
  input wire reset,
  input wire [6:0] opcode,
  output reg brnch,
  output reg mem_rd,
  output reg mem_to_rgs,
  output reg [3:0] alu_op,
  output reg mem_wr,
  output reg alu_src,
  output reg reg_wr
);

always @(reset, opcode) begin
  if(reset == 1)begin
              alu_src <= 0;
              mem_to_rgs <= 0;
              reg_wr <= 0;
              mem_rd <= 0;
              mem_wr <= 0;
              brnch <= 0;
              alu_op <= 0;
  end
  if ((reset == 0)) begin
      case (opcode)
          'h33: begin
              alu_src <= 0;
              mem_to_rgs <= 0;
              reg_wr <= 1;
              mem_rd <= 0;
              mem_wr <= 0;
              brnch <= 0;
              alu_op <= 2;
          end
          'h3: begin
              alu_src <= 1;
              mem_to_rgs <= 1;
              reg_wr <= 1;
              mem_rd <= 1;
              mem_wr <= 0;
              brnch <= 0;
              alu_op <= 0;
          end
          'h23: begin
              alu_src <= 1;
              mem_to_rgs <= 0;
              reg_wr <= 0;
              mem_rd <= 0;
              mem_wr <= 1;
              brnch <= 0;
              alu_op <= 0;
          end
          'h63: begin
              alu_src <= 0;
              mem_to_rgs <= 0;
              reg_wr <= 0;
              mem_rd <= 0;
              mem_wr <= 0;
              brnch <= 1;
              alu_op <= 7;
          end
      endcase
  end
end

  

endmodule


module imm_gen(
  input wire reset,
  input wire [31:0] instruction,
  output reg [31:0] im_gen
);
  reg [19:0] pad;

  always @(reset, instruction) begin
    if(reset == 1) begin
      im_gen<=0;
    end
    if (reset == 0) begin
      if (instruction[6:0] == `ITYPE) begin
        im_gen[12-1:0] <= instruction[31:20];
      end else if (instruction[6:0] == `STYPE) begin
        im_gen[11:5] <= instruction[31:25];
        im_gen[4:0] <= instruction[11:7];
      end else if (instruction[6:0] == `JTYPE) begin
        im_gen[12] <= instruction[31];
        im_gen[10:5] <= instruction[30:25];
        im_gen[11] <= instruction[7];
        im_gen[5-1:1] <= instruction[11:8];
        im_gen[0] <= 0;
      end
      if (instruction[31] == 0) begin
        pad <= 0;
        im_gen[31:12] <= pad;
      end else begin
        pad = 20'hfffff;
        im_gen[31:12] <= pad;
      end
    end
  end
endmodule


module data_mem(
    input wire reset,
    input wire clk,
    input wire [31:0] result,
    input wire mem_wr,
    input wire mem_rd,
    input wire [31:0] rdb,
    output reg [31:0] read_data
);
    reg [31:0] data_ram[0:2048-1];

    initial begin
        $readmemb("D:/PROJECTS BTECH/risc_cpu/mc_data.txt", data_ram);
    end

    always @(reset) begin
      if (reset == 1) begin
        read_data<=0;
      end
    end

    always @(posedge clk, reset) begin
        if (reset == 1) begin
          read_data<=0;
        end
        if (reset == 0) begin
            if (mem_wr) begin
                data_ram[result] <= rdb;
            end
            else if (mem_rd) begin
                read_data <= data_ram[result];
            end
        end
    end
endmodule


module inst_mem(
  input wire reset,
  input wire [`CPU_BITS-1:0] read_addr,
  output reg [`CPU_BITS-1:0] instruction,
  output reg [4:0] ra,
  output reg [4:0] rb,
  output reg [4:0] wa,
  output reg [6:0] opcode
);

  reg [`CPU_BITS-1:0] inst_ram [0:1048576-1];
  


  initial begin
    
    $readmemb("D:/PROJECTS BTECH/risc_cpu/mc_code.txt", inst_ram);
  end

  always @(reset, read_addr,inst_ram) begin
    if(reset ==1) begin
      instruction<=0;
      wa<=0;
      ra<=0;
      rb<=0;
      opcode<=0;
    end
    if (reset==0) begin
      instruction <= inst_ram[read_addr];
      case (inst_ram[read_addr][6:0])
        `RTYPE: begin
          ra <= inst_ram[read_addr][19:15];
          rb <= inst_ram[read_addr][24:20];
          opcode <= inst_ram[read_addr][6:0];
          wa <= inst_ram[read_addr][11:7];
        end
        `ITYPE: begin
          ra <= inst_ram[read_addr][19:15];
          opcode <= inst_ram[read_addr][6:0];
          wa <= inst_ram[read_addr][11:7];
        end
        `STYPE: begin
          ra <= inst_ram[read_addr][19:15];
          rb <= inst_ram[read_addr][24:20];
          opcode <= inst_ram[read_addr][6:0];
          wa <= inst_ram[read_addr][24:20];
        end
        `JTYPE: begin
          ra <= inst_ram[read_addr][19:15];
          rb <= inst_ram[read_addr][24:20];
          opcode <= inst_ram[read_addr][6:0];
        end
      endcase
    end
  end

endmodule


module pc_adder(input wire reset, input wire step, input wire [`CPU_BITS-1:0] pc, output reg [`CPU_BITS-1:0] pc_addr);
  always @(posedge step, posedge reset) begin
    if (reset == 1) begin
      pc_addr <= 0;
    end
    else begin
    if (reset == 0) begin
      pc_addr <= pc + 1;
    end
    end
  end
endmodule


module jmp_adder(
    input wire reset,
    input wire [31:0] read_addr,
    input wire [31:0] im_gen,
    output reg [31:0] jmp_addr
);

always @(read_addr, reset, im_gen) begin
  if (reset == 1) begin
    jmp_addr<=0;
  end   
  if (reset == 0) begin
      jmp_addr = (read_addr + im_gen);
  end
end


endmodule


module pc_assign(reset, read_addr, pc);
  input reset;
  output reg [31:0] read_addr;
  input [31:0] pc;

  always @(reset, pc) begin
    if(reset == 1) begin
      read_addr <=0;
    end
    if (reset == 0) begin
      read_addr <= pc;
    end
  end
endmodule


module cpu_top(
input wire clk,
input wire reset

);

  wire [4:0] ra, rb, wa;
  wire [`CPU_BITS-1:0] wda, rda, rdb, rdx;
  wire [3:0] alu_op;
  wire brnch, mem_rd, mem_to_rgs, mem_wr, alu_src, reg_wr;
  wire [6:0] opcode;
  wire [`CPU_BITS-1:0] pc, result, read_data, shl;
  reg [`CPU_BITS-1:0] aux;
  wire pc_sel;
  wire [`CPU_BITS-1:0] im_gen;
  wire [3:0] alu_decode;

  reg step;

  wire [`CPU_BITS-1:0] read_addr, instruction, pc_addr, jmp_addr;

    initial begin

      step <= 0;
    end
    
    
  
  control cont(reset, opcode, brnch, mem_rd, mem_to_rgs, alu_op, mem_wr, alu_src, reg_wr);
  data_mem dmem(reset, clk, result, mem_wr, mem_rd, rdb, read_data);
  inst_mem imem(reset, read_addr, instruction, ra, rb, wa, opcode);
  alu alux(reset, alu_decode, rda, rdx, result);
  reg_file regf(reset, clk, ra, rb, wa, wda, reg_wr, rda, rdb);
  pc_adder padr(reset, step, pc, pc_addr);
  jmp_adder jadr(reset, read_addr, im_gen, jmp_addr);
  pc_mux pcmx(reset, pc, pc_addr, jmp_addr, pc_sel);
  alu_mux almx(reset, im_gen, rdb, rdx, alu_src);
  wda_mux wdmx(reset, wda, mem_to_rgs, result, read_data);
  alu_control aluc(reset, instruction, alu_op, alu_decode);
  imm_gen imgn(reset, instruction, im_gen);
  pc_assign nxpc(reset, read_addr, pc);
  taken tken(result, brnch, pc_sel);



  always @(posedge clk)
  begin
  if (reset == 0)
    begin
      step <= ~step;
    end
  end

endmodule

