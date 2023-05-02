// riscvpipelined.sv

// RISC-V pipelined processor
// From Section 7.6 of Digital Design & Computer Architecture: RISC-V Edition
// 27 April 2020
// David_Harris@hmc.edu 
// Sarah.Harris@unlv.edu

// run 210
// Expect simulator to print "Simulation succeeded"
// when the value 25 (0x19) is written to address 100 (0x64)

// Pipelined implementation of RISC-V (RV32I)
// User-level Instruction Set Architecture V2.2 (May 7, 2017)
// Implements a subset of the base integer instructions:
//    lw, sw
//    add, sub, and, or, slt, 
//    addi, andi, ori, slti
//    beq
//    jal
// Exceptions, traps, and interrupts not implemented
// little-endian memory

// 31 32-bit registers x1-x31, x0 hardwired to 0
// R-Type instructions
//   add, sub, and, or, slt
//   INSTR rd, rs1, rs2
//   Instr[31:25] = funct7 (funct7b5 & opb5 = 1 for sub, 0 for others)
//   Instr[24:20] = rs2
//   Instr[19:15] = rs1
//   Instr[14:12] = funct3
//   Instr[11:7]  = rd
//   Instr[6:0]   = opcode
// I-Type Instructions
//   lw, I-type ALU (addi, andi, ori, slti)
//   lw:         INSTR rd, imm(rs1)
//   I-type ALU: INSTR rd, rs1, imm (12-bit signed)
//   Instr[31:20] = imm[11:0]
//   Instr[24:20] = rs2
//   Instr[19:15] = rs1
//   Instr[14:12] = funct3
//   Instr[11:7]  = rd
//   Instr[6:0]   = opcode
// S-Type Instruction
//   sw rs2, imm(rs1) (store rs2 into address specified by rs1 + immm)
//   Instr[31:25] = imm[11:5] (offset[11:5])
//   Instr[24:20] = rs2 (src)
//   Instr[19:15] = rs1 (base)
//   Instr[14:12] = funct3
//   Instr[11:7]  = imm[4:0]  (offset[4:0])
//   Instr[6:0]   = opcode
// B-Type Instruction
//   beq rs1, rs2, imm (PCTarget = PC + (signed imm x 2))
//   Instr[31:25] = imm[12], imm[10:5]
//   Instr[24:20] = rs2
//   Instr[19:15] = rs1
//   Instr[14:12] = funct3
//   Instr[11:7]  = imm[4:1], imm[11]
//   Instr[6:0]   = opcode
// J-Type Instruction
//   jal rd, imm  (signed imm is multiplied by 2 and added to PC, rd = PC+4)
//   Instr[31:12] = imm[20], imm[10:1], imm[11], imm[19:12]
//   Instr[11:7]  = rd
//   Instr[6:0]   = opcode

//   Instruction  opcode    funct3    funct7
//   add          0110011   000       0000000
//   sub          0110011   000       0100000
//   and          0110011   111       0000000
//   or           0110011   110       0000000
//   slt          0110011   010       0000000
//   addi         0010011   000       immediate
//   andi         0010011   111       immediate
//   ori          0010011   110       immediate
//   slti         0010011   010       immediate
//   beq          1100011   000       immediate
//   lw	          0000011   010       immediate
//   sw           0100011   010       immediate
//   jal          1101111   immediate immediate

module testbench();

   logic        clk;
   logic        reset;

   logic [31:0] WriteData, DataAdr;
   logic        MemWrite;

   // instantiate device to be tested
   top dut(clk, reset, WriteData, DataAdr, MemWrite);

   initial
     begin
	string memfilename;
        memfilename = {"../riscvtest/riscvtest.memfile"};
	$readmemh(memfilename, dut.imem.RAM);
     end
   
   // initialize test
   initial
     begin
	reset <= 1; # 22; reset <= 0;
     end

   // generate clock to sequence tests
   always
     begin
	clk <= 1; # 5; clk <= 0; # 5;
     end

   // check results
   always @(negedge clk)
     begin
	if(MemWrite) begin
           if(DataAdr === 100 & WriteData === 25) begin
              $display("Simulation succeeded");
              $stop;
           end else if (DataAdr !== 96) begin
              $display("Simulation failed");
              $stop;
           end
	end
     end
endmodule

module top(input  logic        clk, reset, 
           output logic [31:0] WriteDataM, DataAdrM, 
           output logic        MemWriteM);

   logic [31:0] 	       PCF, InstrF, ReadDataM;
   
   // instantiate processor and memories
   riscv rv32pipe (clk, reset, PCF, InstrF, MemWriteM, DataAdrM, 
		   WriteDataM, ReadDataM);
   imem imem (PCF, InstrF);
   dmem dmem (clk, MemWriteM, DataAdrM, WriteDataM, ReadDataM);
   
endmodule

module riscv(input  logic        clk, reset,
             output logic [31:0] PCF,
             input logic [31:0]  InstrF,
             output logic 	 MemWriteM,
             output logic [31:0] ALUResultM, WriteDataM,
             input logic [31:0]  ReadDataM);

   logic [6:0] 			 opD;
   logic [2:0] 			 funct3D;
   logic 			 funct7b5D;
   logic [1:0] 			 ImmSrcD;
   logic 			 ZeroE;
   logic 			 BPWrongE;
   logic [2:0] 			 ALUControlE;
   logic 			 ALUSrcE;
   logic 			 ResultSrcEb0;
   logic 			 RegWriteM;
   logic [1:0] 			 ResultSrcW;
   logic 			 RegWriteW;

   logic [1:0] 			 ForwardAE, ForwardBE;
   logic 			 StallF, StallD, FlushD, FlushE;

   logic [4:0] 			 Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW;
   
   controller c(clk, reset,
		opD, funct3D, funct7b5D, ImmSrcD,
		FlushE, ZeroE, BPWrongE, ALUControlE, ALUSrcE, ResultSrcEb0,
		MemWriteM, RegWriteM, 
		RegWriteW, ResultSrcW);

   datapath dp(clk, reset,
               StallF, PCF, InstrF,
	       opD, funct3D, funct7b5D, StallD, FlushD, ImmSrcD,
	       FlushE, ForwardAE, ForwardBE, BPWrongE, ALUControlE, ALUSrcE, ZeroE,
               MemWriteM, WriteDataM, ALUResultM, ReadDataM,
               RegWriteW, ResultSrcW,
               Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW);

   hazard  hu(Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW,
              BPWrongE, ResultSrcEb0, RegWriteM, RegWriteW,
              ForwardAE, ForwardBE, StallF, StallD, FlushD, FlushE);			 
endmodule


module controller(input  logic		 clk, reset,
                  // Decode stage control signals
                  input logic [6:0]  opD,
                  input logic [2:0]  funct3D,
                  input logic 	     funct7b5D,
                  output logic [1:0] ImmSrcD,
                  // Execute stage control signals
                  input logic 	     FlushE, 
                  input logic 	     ZeroE, 
                  output logic 	     BPWrongE, // for datapath and Hazard Unit
                  output logic [2:0] ALUControlE, 
                  output logic 	     ALUSrcE,
                  output logic 	     ResultSrcEb0, // for Hazard Unit
                  // Memory stage control signals
                  output logic 	     MemWriteM,
                  output logic 	     RegWriteM, // for Hazard Unit				  
                  // Writeback stage control signals
                  output logic 	     RegWriteW, // for datapath and Hazard Unit
                  output logic [1:0] ResultSrcW);

   // pipelined control signals
   logic 			     RegWriteD, RegWriteE;
   logic [1:0] 			     ResultSrcD, ResultSrcE, ResultSrcM;
   logic 			     MemWriteD, MemWriteE;
   logic 			     JumpD, JumpE;
   logic 			     BranchD, BranchE;
   logic [1:0] 			     ALUOpD;
   logic [2:0] 			     ALUControlD;
   logic 			     ALUSrcD;
   
   // Decode stage logic
   maindec md(opD, ResultSrcD, MemWriteD, BranchD,
              ALUSrcD, RegWriteD, JumpD, ImmSrcD, ALUOpD);
   aludec  ad(opD[5], funct3D, funct7b5D, ALUOpD, ALUControlD);
   
   // Execute stage pipeline control register and logic
   floprc #(10) controlregE(clk, reset, FlushE,
                            {RegWriteD, ResultSrcD, MemWriteD, JumpD, BranchD, ALUControlD, ALUSrcD},
                            {RegWriteE, ResultSrcE, MemWriteE, JumpE, BranchE, ALUControlE, ALUSrcE});

   assign BPWrongE = (BranchE & ZeroE) | JumpE;
   assign ResultSrcEb0 = ResultSrcE[0];
   
   // Memory stage pipeline control register
   flopr #(4) controlregM(clk, reset,
                          {RegWriteE, ResultSrcE, MemWriteE},
                          {RegWriteM, ResultSrcM, MemWriteM});
   
   // Writeback stage pipeline control register
   flopr #(3) controlregW(clk, reset,
                          {RegWriteM, ResultSrcM},
                          {RegWriteW, ResultSrcW});     
endmodule

module maindec(input  logic [6:0] op,
               output logic [1:0] ResultSrc,
               output logic 	  MemWrite,
               output logic 	  Branch, ALUSrc,
               output logic 	  RegWrite, Jump,
               output logic [1:0] ImmSrc,
               output logic [1:0] ALUOp);

   logic [10:0] 		  controls;

   assign {RegWrite, ImmSrc, ALUSrc, MemWrite,
           ResultSrc, Branch, ALUOp, Jump} = controls;

   always_comb
     case(op)
       // RegWrite_ImmSrc_ALUSrc_MemWrite_ResultSrc_Branch_ALUOp_Jump
       7'b0000011: controls = 11'b1_00_1_0_01_0_00_0; // lw
       7'b0100011: controls = 11'b0_01_1_1_00_0_00_0; // sw
       7'b0110011: controls = 11'b1_xx_0_0_00_0_10_0; // R-type 
       7'b1100011: controls = 11'b0_10_0_0_00_1_01_0; // beq
       7'b0010011: controls = 11'b1_00_1_0_00_0_10_0; // I-type ALU
       7'b1101111: controls = 11'b1_11_0_0_10_0_00_1; // jal
       7'b0000000: controls = 11'b0_00_0_0_00_0_00_0; // need valid values at reset
       default:    controls = 11'bx_xx_x_x_xx_x_xx_x; // non-implemented instruction
     endcase
endmodule

module aludec(input  logic       opb5,
              input logic [2:0]  funct3,
              input logic 	 funct7b5, 
              input logic [1:0]  ALUOp,
              output logic [2:0] ALUControl);

   logic 			 RtypeSub;
   assign RtypeSub = funct7b5 & opb5;  // TRUE for R-type subtract instruction

   always_comb
     case(ALUOp)
       2'b00:                ALUControl = 3'b000; // addition
       2'b01:                ALUControl = 3'b001; // subtraction
       default: case(funct3) // R-type or I-type ALU
                  3'b000:  if (RtypeSub) 
                    ALUControl = 3'b001; // sub
                  else          
                    ALUControl = 3'b000; // add, addi
                  3'b010:    ALUControl = 3'b101; // slt, slti
                  3'b110:    ALUControl = 3'b011; // or, ori
                  3'b111:    ALUControl = 3'b010; // and, andi
                  default:   ALUControl = 3'bxxx; // ???
		endcase
     endcase
endmodule

module datapath(input logic clk, reset,
                // Fetch stage signals
                input logic 	    StallF,
                output logic [31:0] PCF,
                input logic [31:0]  InstrF,
                // Decode stage signals
                output logic [6:0]  opD,
                output logic [2:0]  funct3D, 
                output logic 	    funct7b5D,
                input logic 	    StallD, FlushD,
                input logic [1:0]   ImmSrcD,
                // Execute stage signals
                input logic 	    FlushE,
                input logic [1:0]   ForwardAE, ForwardBE,
                input logic 	    BPWrongE,
                input logic [2:0]   ALUControlE,
                input logic 	    ALUSrcE,
                output logic 	    ZeroE,
                // Memory stage signals
                input logic 	    MemWriteM, 
                output logic [31:0] WriteDataM, ALUResultM,
                input logic [31:0]  ReadDataM,
                // Writeback stage signals
                input logic 	    RegWriteW, 
                input logic [1:0]   ResultSrcW,
                // Hazard Unit signals 
                output logic [4:0]  Rs1D, Rs2D, Rs1E, Rs2E,
                output logic [4:0]  RdE, RdM, RdW);

   // Fetch stage signals
   logic [31:0] 		    PCNextF, PCPlus4F;
   // Decode stage signals
   logic [31:0] 		    InstrD;
   logic [31:0] 		    PCD, PCPlus4D;
   logic [31:0] 		    RD1D, RD2D;
   logic [31:0] 		    ImmExtD;
   logic [4:0] 			    RdD;
   // Execute stage signals
   logic [31:0] 		    RD1E, RD2E;
   logic [31:0] 		    PCE, ImmExtE;
   logic [31:0] 		    SrcAE, SrcBE;
   logic [31:0] 		    ALUResultE;
   logic [31:0] 		    WriteDataE;
   logic [31:0] 		    PCPlus4E;
   logic [31:0] 		    PCTargetE;
   // Memory stage signals
   logic [31:0] 		    PCPlus4M;
   // Writeback stage signals
   logic [31:0] 		    ALUResultW;
   logic [31:0] 		    ReadDataW;
   logic [31:0] 		    PCPlus4W;
   logic [31:0] 		    ResultW;

   // Fetch stage pipeline register and logic
   mux2    #(32) pcmux(PCPlus4F, PCTargetE, BPWrongE, PCNextF);
   flopenr #(32) pcreg(clk, reset, ~StallF, PCNextF, PCF);
   adder         pcadd(PCF, 32'h4, PCPlus4F);

   // Decode stage pipeline register and logic
   flopenrc #(96) regD(clk, reset, FlushD, ~StallD, 
                       {InstrF, PCF, PCPlus4F},
                       {InstrD, PCD, PCPlus4D});
   assign opD       = InstrD[6:0];
   assign funct3D   = InstrD[14:12];
   assign funct7b5D = InstrD[30];
   assign Rs1D      = InstrD[19:15];
   assign Rs2D      = InstrD[24:20];
   assign RdD       = InstrD[11:7];
   
   regfile        rf(clk, RegWriteW, Rs1D, Rs2D, RdW, ResultW, RD1D, RD2D);
   extend         ext(InstrD[31:7], ImmSrcD, ImmExtD);
   
   // Execute stage pipeline register and logic
   floprc #(175) regE(clk, reset, FlushE, 
                      {RD1D, RD2D, PCD, Rs1D, Rs2D, RdD, ImmExtD, PCPlus4D}, 
                      {RD1E, RD2E, PCE, Rs1E, Rs2E, RdE, ImmExtE, PCPlus4E});
   
   mux3   #(32)  faemux(RD1E, ResultW, ALUResultM, ForwardAE, SrcAE);
   mux3   #(32)  fbemux(RD2E, ResultW, ALUResultM, ForwardBE, WriteDataE);
   mux2   #(32)  srcbmux(WriteDataE, ImmExtE, ALUSrcE, SrcBE);
   alu           alu(SrcAE, SrcBE, ALUControlE, ALUResultE, ZeroE);
   adder         branchadd(ImmExtE, PCE, PCTargetE);

   // Memory stage pipeline register
   flopr  #(101) regM(clk, reset, 
                      {ALUResultE, WriteDataE, RdE, PCPlus4E},
                      {ALUResultM, WriteDataM, RdM, PCPlus4M});
   
   // Writeback stage pipeline register and logic
   flopr  #(101) regW(clk, reset, 
                      {ALUResultM, ReadDataM, RdM, PCPlus4M},
                      {ALUResultW, ReadDataW, RdW, PCPlus4W});
   mux3   #(32)  resultmux(ALUResultW, ReadDataW, PCPlus4W, ResultSrcW, ResultW);	
endmodule

// Hazard Unit: forward, stall, and flush
module hazard(input  logic [4:0] Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW,
              input logic 	 BPWrongE, ResultSrcEb0, 
              input logic 	 RegWriteM, RegWriteW,
              output logic [1:0] ForwardAE, ForwardBE,
              output logic 	 StallF, StallD, FlushD, FlushE);

   logic 			 lwStallD;
   
   // forwarding logic
   always_comb begin
      ForwardAE = 2'b00;
      ForwardBE = 2'b00;
      if (Rs1E != 5'b0)
	if      ((Rs1E == RdM) & RegWriteM) ForwardAE = 2'b10;
	else if ((Rs1E == RdW) & RegWriteW) ForwardAE = 2'b01;
      
      if (Rs2E != 5'b0)
	if      ((Rs2E == RdM) & RegWriteM) ForwardBE = 2'b10;
	else if ((Rs2E == RdW) & RegWriteW) ForwardBE = 2'b01;
   end
   
   // stalls and flushes
   assign lwStallD = ResultSrcEb0 & ((Rs1D == RdE) | (Rs2D == RdE));  
   assign StallD = lwStallD;
   assign StallF = lwStallD;
   assign FlushD = BPWrongE;
   assign FlushE = lwStallD | BPWrongE;
endmodule

module regfile(input  logic        clk, 
               input logic 	   we3, 
               input logic [ 4:0]  a1, a2, a3, 
               input logic [31:0]  wd3, 
               output logic [31:0] rd1, rd2);

   logic [31:0] 		   rf[31:0];

   // three ported register file
   // read two ports combinationally (A1/RD1, A2/RD2)
   // write third port on rising edge of clock (A3/WD3/WE3)
   // write occurs on falling edge of clock
   // register 0 hardwired to 0

   always_ff @(negedge clk)
     if (we3) rf[a3] <= wd3;	

   assign rd1 = (a1 != 0) ? rf[a1] : 0;
   assign rd2 = (a2 != 0) ? rf[a2] : 0;
endmodule

module adder(input  [31:0] a, b,
             output [31:0] y);

   assign y = a + b;
endmodule

module extend(input  logic [31:7] instr,
              input logic [1:0]   immsrc,
              output logic [31:0] immext);
   
   always_comb
     case(immsrc) 
       // I-type 
       2'b00:   immext = {{20{instr[31]}}, instr[31:20]};  
       // S-type (stores)
       2'b01:   immext = {{20{instr[31]}}, instr[31:25], instr[11:7]}; 
       // B-type (branches)
       2'b10:   immext = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0}; 
       // J-type (jal)
       2'b11:   immext = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0}; 
       default: immext = 32'bx; // undefined
     endcase             
endmodule

module flopr #(parameter WIDTH = 8)
   (input  logic             clk, reset,
    input logic [WIDTH-1:0]  d, 
    output logic [WIDTH-1:0] q);

   always_ff @(posedge clk, posedge reset)
     if (reset) q <= 0;
     else       q <= d;
endmodule

module flopenr #(parameter WIDTH = 8)
   (input  logic             clk, reset, en,
    input logic [WIDTH-1:0]  d, 
    output logic [WIDTH-1:0] q);

   always_ff @(posedge clk, posedge reset)
     if (reset)   q <= 0;
     else if (en) q <= d;
endmodule

module flopenrc #(parameter WIDTH = 8)
   (input  logic             clk, reset, clear, en,
    input logic [WIDTH-1:0]  d, 
    output logic [WIDTH-1:0] q);

   always_ff @(posedge clk, posedge reset)
     if (reset)   q <= 0;
     else if (en) 
       if (clear) q <= 0;
       else       q <= d;
endmodule

module floprc #(parameter WIDTH = 8)
   (input  logic clk,
    input logic 	     reset,
    input logic 	     clear,
    input logic [WIDTH-1:0]  d, 
    output logic [WIDTH-1:0] q);

   always_ff @(posedge clk, posedge reset)
     if (reset) q <= 0;
     else       
       if (clear) q <= 0;
       else       q <= d;
endmodule

module mux2 #(parameter WIDTH = 8)
   (input  logic [WIDTH-1:0] d0, d1, 
    input logic 	     s, 
    output logic [WIDTH-1:0] y);

   assign y = s ? d1 : d0; 
endmodule

module mux3 #(parameter WIDTH = 8)
   (input  logic [WIDTH-1:0] d0, d1, d2,
    input logic [1:0] 	     s, 
    output logic [WIDTH-1:0] y);

   assign y = s[1] ? d2 : (s[0] ? d1 : d0); 
endmodule

module imem (input  logic [31:0] a,
	     output logic [31:0] rd);
   
   logic [31:0] 		 RAM[63:0];
   
   assign rd = RAM[a[31:2]]; // word aligned
   
endmodule // imem

module dmem (input  logic        clk, we,
	     input  logic [31:0] a, wd,
	     output logic [31:0] rd);
   
   logic [31:0] 		 RAM[255:0];
   
   assign rd = RAM[a[31:2]]; // word aligned
   always_ff @(posedge clk)
     if (we) RAM[a[31:2]] <= wd;
   
endmodule // dmem

module alu(input  logic [31:0] a, b,
           input logic [2:0]   alucontrol,
           output logic [31:0] result,
           output logic        zero);

   logic [31:0] 	       condinvb, sum;
   logic 		       v;              // overflow
   logic 		       isAddSub;       // true when is add or sub

   assign condinvb = alucontrol[0] ? ~b : b;
   assign sum = a + condinvb + alucontrol[0];
   assign isAddSub = ~alucontrol[2] & ~alucontrol[1] |
                     ~alucontrol[1] &  alucontrol[0];

   always_comb
     case (alucontrol)
       3'b000:  result = sum;         // add
       3'b001:  result = sum;         // subtract
       3'b010:  result = a & b;       // and
       3'b011:  result = a | b;       // or
       3'b100:  result = a ^ b;       // xor
       3'b101:  result = sum[31] ^ v; // slt
       3'b110:  result = a << b[4:0]; // sll
       3'b111:  result = a >> b[4:0]; // srl
       default: result = 32'bx;
     endcase

   assign zero = (result == 32'b0);
   assign v = ~(alucontrol[0] ^ a[31] ^ b[31]) & (a[31] ^ sum[31]) & isAddSub;
   
endmodule

///////////////////////////////////////////
// bpred.sv
//
// Written: Ross Thomposn ross1728@gmail.com
// Created: 12 February 2021
// Modified: 19 January 2023
//
// Purpose: Branch direction prediction and jump/branch target prediction.
//          Prediction made during the fetch stage and corrected in the execution stage.
// 
// A component of the CORE-V-WALLY configurable RISC-V project.
// 
// Copyright (C) 2021-23 Harvey Mudd College & Oklahoma State University
//
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
//
// Licensed under the Solderpad Hardware License v 2.1 (the “License”); you may not use this file 
// except in compliance with the License, or, at your option, the Apache License version 2.0. You 
// may obtain a copy of the License at
//
// https://solderpad.org/licenses/SHL-2.1/
//
// Unless required by applicable law or agreed to in writing, any work distributed under the 
// License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, 
// either express or implied. See the License for the specific language governing permissions 
// and limitations under the License.
////////////////////////////////////////////////////////////////////////////////////////////////


`define INSTR_CLASS_PRED 1

module bpred (
  input  logic             clk, reset,
  input  logic             StallF, StallD, StallE, StallM, StallW,
  input  logic             FlushD, FlushE, FlushM, FlushW,
  // Fetch stage
  // the prediction
  input  logic [31:0]      InstrD,                    // Decompressed decode stage instruction. Used to decode instruction class
  input  logic [32-1:0] PCNextF,                   // Next Fetch Address
  input  logic [32-1:0] PCPlus2or4F,               // PCF+2/4
  output logic [32-1:0] PC1NextF,                  // Branch Predictor predicted or corrected fetch address on miss prediction
  output logic [32-1:0] NextValidPCE,              // Address of next valid instruction after the instruction in the Memory stage

  // Update Predictor
  input  logic [32-1:0] PCF,                       // Fetch stage instruction address
  input  logic [32-1:0] PCD,                       // Decode stage instruction address. Also the address the branch predictor took
  input  logic [32-1:0] PCE,                       // Execution stage instruction address
  input  logic [32-1:0] PCM,                       // Memory stage instruction address

  input  logic [31:0]      PostSpillInstrRawF,        // Instruction

  // Branch and jump outcome
  input  logic             InstrValidD, InstrValidE,
  input  logic             BranchD, BranchE,
  input  logic             JumpD, JumpE,
  //input  logic             BPWrongE,                    // Executation stage branch is taken
  input  logic [32-1:0] IEUAdrE,                   // The branch/jump target address
  input  logic [32-1:0] IEUAdrM,                   // The branch/jump target address
  input  logic [32-1:0] PCLinkE,                   // The address following the branch instruction. (AKA Fall through address)
  output logic [3:0]       InstrClassM,               // The valid instruction class. 1-hot encoded as call, return, jr (not return), j, br

  // Report branch prediction status
  output logic             BPWrongE,                  // Prediction is wrong
  output logic             BPWrongM,                  // Prediction is wrong
  output logic             BPDirPredWrongM,           // Prediction direction is wrong
  //output logic             BTAWrongM,                 // Prediction target wrong
  //output logic             RASPredPCWrongM,           // RAS prediction is wrong
  output logic             IClassWrongM               // Class prediction is wrong
  );

  logic [1:0]              BPDirPredF;

  logic [32-1:0]        BPBTAF, RASPCF;
  logic                    BPPCWrongE;
  logic                    IClassWrongE;
  logic                    BPDirPredWrongE;
  
  logic                    BPPCSrcF;
  logic [32-1:0]        BPPCF;
  logic [32-1:0]        PC0NextF;
  logic [32-1:0]        PCCorrectE;
  logic [3:0]              WrongPredInstrClassD;

  logic                    BTBTargetWrongE;
  logic                    RASTargetWrongE;

  logic [32-1:0]        BPBTAD;

  logic                    BTBCallF, BTBReturnF, BTBJumpF, BTBBranchF;
  logic                    BPBranchF, BPJumpF, BPReturnF, BPCallF;
  logic                    BPBranchD, BPJumpD, BPReturnD, BPCallD;
  logic                    ReturnD, CallD;
  logic                    ReturnE, CallE;
  logic                    BranchM, JumpM, ReturnM, CallM;
  logic                    BranchW, JumpW, ReturnW, CallW;
  logic                    BPReturnWrongD;
  logic [32-1:0]        BPBTAE;
  
  // Part 1 branch direction prediction
  // look into the 2 port Sram model. something is wrong. 
    gshare #(16, 8) DirPredictor(.clk, .reset, .StallF, .StallD, .StallE, .StallM, .StallW, .FlushD, .FlushE, .FlushM, .FlushW,
      .PCNextF, .PCF, .PCD, .PCE, .PCM, .BPDirPredF, .BPDirPredWrongE,
      .BPBranchF, .BranchD, .BranchE, .BranchM, .BranchW, 
      .BPWrongE);

/* -----\/----- EXCLUDED -----\/-----
    localHistoryPredictor DirPredictor(.clk,
      .reset, .StallF, .StallE,
      .LookUpPC(PCNextF),
      .Prediction(BPDirPredF),
      // update
      .UpdatePC(PCE),
      .UpdateEN(InstrClassE[0] & ~StallE),
      .BPWrongE,
      .UpdatePrediction(InstrClassE[0]));
 -----/\----- EXCLUDED -----/\----- */

  // Part 2 Branch target address prediction
  // BTB contains target address for all CFI

  btb #(10) 
    TargetPredictor(.clk, .reset, .StallF, .StallD, .StallE, .StallM, .StallW, .FlushD, .FlushE, .FlushM, .FlushW,
      .PCNextF, .PCF, .PCD, .PCE, .PCM,
      .BPBTAF, .BPBTAD, .BPBTAE,
      .BTBIClassF({BTBCallF, BTBReturnF, BTBJumpF, BTBBranchF}),
      .IClassWrongM, .IClassWrongE,
      .IEUAdrE, .IEUAdrM,
      .InstrClassD({CallD, ReturnD, JumpD, BranchD}), 
      .InstrClassE({CallE, ReturnE, JumpE, BranchE}), 
      .InstrClassM({CallM, ReturnM, JumpM, BranchM}),
      .InstrClassW({CallW, ReturnW, JumpW, BranchW}));

  icpred #(`INSTR_CLASS_PRED) icpred(.clk, .reset, .StallF, .StallD, .StallE, .StallM, .StallW, .FlushD, .FlushE, .FlushM, .FlushW,
    .PostSpillInstrRawF, .InstrD, .BranchD, .BranchE, .JumpD, .JumpE, .BranchM, .BranchW, .JumpM, .JumpW,
    .CallD, .CallE, .CallM, .CallW, .ReturnD, .ReturnE, .ReturnM, .ReturnW, .BTBCallF, .BTBReturnF, .BTBJumpF,
    .BTBBranchF, .BPCallF, .BPReturnF, .BPJumpF, .BPBranchF, .IClassWrongM, .IClassWrongE, .BPReturnWrongD);

  // Part 3 RAS
  RASPredictor RASPredictor(.clk, .reset, .StallF, .StallD, .StallE, .StallM, .FlushD, .FlushE, .FlushM,
    .BPReturnF, .ReturnD, .ReturnE, .CallE,
    .BPReturnWrongD, .RASPCF, .PCLinkE);

  // Check the prediction
  // if it is a CFI then check if the next instruction address (PCD) matches the branch's target or fallthrough address.
  // if the class prediction is wrong a regular instruction may have been predicted as a taken branch
  // this will result in PCD not being equal to the fall through address PCLinkE (PCE+4).
  // The next instruction is always valid as no other flush would occur at the same time as the branch and not
  // also flush the branch.  This will change in a superscaler cpu. 
  // branch is wrong only if the PC does not match and both the Decode and Fetch stages have valid instructions.
  assign BPWrongE = (PCCorrectE != PCD) & InstrValidE & InstrValidD;
  flopenrc #(1) BPWrongMReg(clk, reset, FlushM, ~StallM, BPWrongE, BPWrongM);
  
  // Output the predicted PC or corrected PC on miss-predict.
  assign BPPCSrcF = (BPBranchF & BPDirPredF[1]) | BPJumpF;
  mux2 #(32) pcmuxbp(BPBTAF, RASPCF, BPReturnF, BPPCF);
  // Selects the BP or PC+2/4.
  mux2 #(32) pcmux0(PCPlus2or4F, BPPCF, BPPCSrcF, PC0NextF);
  // If the prediction is wrong select the correct address.
  mux2 #(32) pcmux1(PC0NextF, PCCorrectE, BPWrongE, PC1NextF);  
  // Correct branch/jump target.
  mux2 #(32) pccorrectemux(PCLinkE, IEUAdrE, BPWrongE, PCCorrectE);
  
  // If the fence/csrw was predicted as a taken branch then we select PCF, rather than PCE.
  // Effectively this is PCM+4 or the non-existant PCLinkM
  if(`INSTR_CLASS_PRED) mux2 #(32) pcmuxBPWrongInvalidateFlush(PCE, PCF, BPWrongM, NextValidPCE);
  else  assign NextValidPCE = PCE;

  // **** Fix me
  assign InstrClassM = {CallM, ReturnM, JumpM, BranchM};
  
endmodule
///////////////////////////////////////////
// gshare.sv
//
// Written: Ross Thompson
// Email: ross1728@gmail.com
// Created: 16 March 2021
// Adapted from ssanghai@hmc.edu (Shreya Sanghai)
// Modified: 20 February 2023
//
// Purpose: gshare and Global History Branch predictors
// 
// A component of the CORE-V-WALLY configurable RISC-V project.
// 
// Copyright (C) 2021-23 Harvey Mudd College & Oklahoma State University
//
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
//
// Licensed under the Solderpad Hardware License v 2.1 (the “License”); you may not use this file 
// except in compliance with the License, or, at your option, the Apache License version 2.0. You 
// may obtain a copy of the License at
//
// https://solderpad.org/licenses/SHL-2.1/
//
// Unless required by applicable law or agreed to in writing, any work distributed under the 
// License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, 
// either express or implied. See the License for the specific language governing permissions 
// and limitations under the License.
////////////////////////////////////////////////////////////////////////////////////////////////


module gshare #(parameter k = 10,
                parameter integer TYPE = 1) (
  input logic             clk,
  input logic             reset,
  input logic             StallF, StallD, StallE, StallM, StallW,
  input logic             FlushD, FlushE, FlushM, FlushW,
  output logic [1:0]      BPDirPredF, 
  output logic            BPDirPredWrongE,
  // update
  input logic [32-1:0] PCNextF, PCF, PCD, PCE, PCM,
  input logic             BPBranchF, BranchD, BranchE, BranchM, BranchW, BPWrongE
);

  logic                   MatchF, MatchD, MatchE, MatchM, MatchW;
  logic                   MatchX;

  logic [1:0]             TableBPDirPredF, BPDirPredD, BPDirPredE, FwdNewDirPredF;
  logic [1:0]             NewBPDirPredE, NewBPDirPredM, NewBPDirPredW;

  logic [k-1:0]           IndexNextF, IndexF, IndexD, IndexE, IndexM, IndexW;

  logic [k-1:0]           GHRF, GHRD, GHRE, GHRM;
  logic [k-1:0]           GHRNextM, GHRNextF;
  logic                   PCSrcM;

  if(TYPE == 1) begin
  assign IndexNextF = GHRNextF ^ {PCNextF[k+1] ^ PCNextF[1], PCNextF[k:2]};
  assign IndexF = GHRF ^ {PCF[k+1] ^ PCF[1], PCF[k:2]};
  assign IndexD = GHRD ^ {PCD[k+1] ^ PCD[1], PCD[k:2]};
  assign IndexE = GHRE ^ {PCE[k+1] ^ PCE[1], PCE[k:2]};
  assign IndexM = GHRM ^ {PCM[k+1] ^ PCM[1], PCM[k:2]};
  end else if(TYPE == 0) begin
  assign IndexNextF = GHRNextF;
  assign IndexF = GHRF;
  assign IndexD = GHRD;
  assign IndexE = GHRE;
  assign IndexM = GHRM;
  end

  flopenrc #(k) IndexWReg(clk, reset, FlushW, ~StallW, IndexM, IndexW);

  assign MatchD = BranchD & ~FlushE & (IndexF == IndexD);
  assign MatchE = BranchE & ~FlushM & (IndexF == IndexE);
  assign MatchM = BranchM & ~FlushW & (IndexF == IndexM);
  assign MatchW = BranchW & ~FlushW & (IndexF == IndexW);
  assign MatchX = MatchD | MatchE | MatchM | MatchW;

  assign FwdNewDirPredF = MatchD ? {2{BPDirPredD[1]}} :
                                   MatchE ? {NewBPDirPredE} :
                                   MatchM ? {NewBPDirPredM} :
                   NewBPDirPredW ;
  
  assign BPDirPredF = MatchX ? FwdNewDirPredF : TableBPDirPredF;

  ram2p1r1wbe #(2**k, 2) PHT(.clk(clk),
    .ce1(~StallF), .ce2(~StallW & ~FlushW),
    .ra1(IndexNextF),
    .rd1(TableBPDirPredF),
    .wa2(IndexM),
    .wd2(NewBPDirPredM),
    .we2(BranchM),
    .bwe2(1'b1));

  flopenrc #(2) PredictionRegD(clk, reset,  FlushD, ~StallD, BPDirPredF, BPDirPredD);
  flopenrc #(2) PredictionRegE(clk, reset,  FlushE, ~StallE, BPDirPredD, BPDirPredE);

  satCounter2 BPDirUpdateE(.BrDir(BPWrongE), .OldState(BPDirPredE), .NewState(NewBPDirPredE));
  flopenrc #(2) NewPredictionRegM(clk, reset,  FlushM, ~StallM, NewBPDirPredE, NewBPDirPredM);
  flopenrc #(2) NewPredictionRegW(clk, reset,  FlushW, ~StallW, NewBPDirPredM, NewBPDirPredW);

  assign BPDirPredWrongE = BPWrongE != BPDirPredE[1] & BranchE;

  assign GHRNextF = BPBranchF ? {BPDirPredF[1], GHRF[k-1:1]} : GHRF;
  assign GHRF = BranchD  ? {BPDirPredD[1], GHRD[k-1:1]} : GHRD;
  assign GHRD = BranchE ? {BPWrongE, GHRE[k-1:1]} : GHRE;
  assign GHRE = BranchM ? {PCSrcM, GHRM[k-1:1]} : GHRM;

  assign GHRNextM = {PCSrcM, GHRM[k-1:1]};

  flopenr #(k) GHRReg(clk, reset, ~StallW & ~FlushW & BranchM, GHRNextM, GHRM);
  flopenrc #(1) PCSrcMReg(clk, reset, FlushM, ~StallM, BPWrongE, PCSrcM);
    
endmodule
///////////////////////////////////////////
// icpred.sv
//
// Written: Ross Thomposn ross1728@gmail.com
// Created: February 26, 2023
// Modified: February 26, 2023
//
// Purpose: Partial decode of instructions into control flow instructions (cfi)P
//          Call, Return, Jump, and Branch
// 
// A component of the CORE-V-WALLY configurable RISC-V project.
// 
// Copyright (C) 2021-23 Harvey Mudd College & Oklahoma State University
//
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
//
// Licensed under the Solderpad Hardware License v 2.1 (the “License”); you may not use this file 
// except in compliance with the License, or, at your option, the Apache License version 2.0. You 
// may obtain a copy of the License at
//
// https://solderpad.org/licenses/SHL-2.1/
//
// Unless required by applicable law or agreed to in writing, any work distributed under the 
// License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, 
// either express or implied. See the License for the specific language governing permissions 
// and limitations under the License.
////////////////////////////////////////////////////////////////////////////////////////////////



module icpred #(parameter INSTR_CLASS_PRED = 1)(
  input  logic             clk, reset,
  input  logic             StallF, StallD, StallE, StallM, StallW,
  input  logic             FlushD, FlushE, FlushM, FlushW,
  input  logic [31:0]      PostSpillInstrRawF, InstrD,        // Instruction
  input  logic             BranchD, BranchE,
  input  logic             JumpD, JumpE,
  output logic             BranchM, BranchW,
  output logic             JumpM, JumpW,
  output logic             CallD, CallE, CallM, CallW,
  output logic             ReturnD, ReturnE, ReturnM, ReturnW,
  input  logic             BTBCallF, BTBReturnF, BTBJumpF, BTBBranchF,
  output logic             BPCallF, BPReturnF, BPJumpF, BPBranchF,
  output logic             IClassWrongM, BPReturnWrongD, IClassWrongE
);

  logic                    IClassWrongD;
  logic                    BPBranchD, BPJumpD, BPReturnD, BPCallD;

  if (!INSTR_CLASS_PRED) begin : DirectClassDecode
    // This section is mainly for testing, verification, and PPA comparison.
    // An alternative to using the BTB to store the instruction class is to partially decode
    // the instructions in the Fetch stage into, Call, Return, Jump, and Branch instructions.
    // This logic is not described in the text book as of 23 February 2023.
    logic     ccall, cj, cjr, ccallr, CJumpF, CBranchF;
    logic     NCJumpF, NCBranchF;

 //   if(`C_SUPPORTED) begin 
 //     logic [4:0] CompressedOpcF;
 //     assign CompressedOpcF = {PostSpillInstrRawF[1:0], PostSpillInstrRawF[15:13]};
 //     assign ccall = CompressedOpcF == 5'h09 & 32 == 32;
 //     assign cj = CompressedOpcF == 5'h0d;
 //     assign cjr = CompressedOpcF == 5'h14 & ~PostSpillInstrRawF[12] & PostSpillInstrRawF[6:2] == 5'b0 & PostSpillInstrRawF[11:7] != 5'b0;
 //     assign ccallr = CompressedOpcF == 5'h14 & PostSpillInstrRawF[12] & PostSpillInstrRawF[6:2] == 5'b0 & PostSpillInstrRawF[11:7] != 5'b0;
 //     assign CJumpF = ccall | cj | cjr | ccallr;
 //     assign CBranchF = CompressedOpcF[4:1] == 4'h7;
    //end else begin
      assign {ccall, cj, cjr, ccallr, CJumpF, CBranchF} = '0;
    //end
 
    assign NCJumpF = PostSpillInstrRawF[6:0] == 7'h67 | PostSpillInstrRawF[6:0] == 7'h6F;
    assign NCBranchF = PostSpillInstrRawF[6:0] == 7'h63;
 
    assign BPBranchF = NCBranchF; //| (`C_SUPPORTED & CBranchF);
    assign BPJumpF = NCJumpF ;//| (`C_SUPPORTED & (CJumpF));
    assign BPReturnF = (NCJumpF & (PostSpillInstrRawF[19:15] & 5'h1B) == 5'h01);// | // returnurn must returnurn to ra or r5
        //(`C_SUPPORTED & (ccallr | cjr) & ((PostSpillInstrRawF[11:7] & 5'h1B) == 5'h01));
    
    assign BPCallF = (NCJumpF & (PostSpillInstrRawF[11:07] & 5'h1B) == 5'h01);// | // call(r) must link to ra or x5
        //(`C_SUPPORTED & (ccall | (ccallr & (PostSpillInstrRawF[11:7] & 5'h1b) == 5'h01)));
    end else begin
    // This section connects the BTB's instruction class prediction.
    assign {BPCallF, BPReturnF, BPJumpF, BPBranchF} = {BTBCallF, BTBReturnF, BTBJumpF, BTBBranchF};
    end

  assign ReturnD = JumpD & (InstrD[19:15] & 5'h1B) == 5'h01; // returnurn must returnurn to ra or x5
  assign CallD = JumpD & (InstrD[11:7] & 5'h1B) == 5'h01; // call(r) must link to ra or x5

  flopenrc #(2) InstrClassRegE(clk, reset,  FlushE, ~StallE, {CallD, ReturnD}, {CallE, ReturnE});
  flopenrc #(4) InstrClassRegM(clk, reset,  FlushM, ~StallM, {CallE, ReturnE, JumpE, BranchE}, {CallM, ReturnM, JumpM, BranchM});
  flopenrc #(4) InstrClassRegW(clk, reset,  FlushM, ~StallW, {CallM, ReturnM, JumpM, BranchM}, {CallW, ReturnW, JumpW, BranchW});

  // branch predictor
  flopenrc #(1) BPClassWrongRegM(clk, reset, FlushM, ~StallM, IClassWrongE, IClassWrongM);
  flopenrc #(1) WrongInstrClassRegE(clk, reset, FlushE, ~StallE, IClassWrongD, IClassWrongE);

  // pipeline the predicted class
  flopenrc #(4) PredInstrClassRegD(clk, reset, FlushD, ~StallD, {BPCallF, BPReturnF, BPJumpF, BPBranchF}, {BPCallD, BPReturnD, BPJumpD, BPBranchD});

  // branch class prediction wrong.
  assign IClassWrongD = |({BPCallD, BPReturnD, BPJumpD, BPBranchD} ^ {CallD, ReturnD, JumpD, BranchD});
  assign BPReturnWrongD = BPReturnD ^ ReturnD;

endmodule
///////////////////////////////////////////
// RASPredictor.sv
//
// Written: Ross Thomposn ross1728@gmail.com
// Created: 15 February 2021
// Modified: 25 January 2023
//
// Purpose: 2 bit saturating counter predictor with parameterized table depth.
// 
// Documentation: RISC-V System on Chip Design Chapter 10 (Figure ***)
//
// A component of the CORE-V-WALLY configurable RISC-V project.
// 
// Copyright (C) 2021-23 Harvey Mudd College & Oklahoma State University
//
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
//
// Licensed under the Solderpad Hardware License v 2.1 (the “License”); you may not use this file 
// except in compliance with the License, or, at your option, the Apache License version 2.0. You 
// may obtain a copy of the License at
//
// https://solderpad.org/licenses/SHL-2.1/
//
// Unless required by applicable law or agreed to in writing, any work distributed under the 
// License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, 
// either express or implied. See the License for the specific language governing permissions 
// and limitations under the License.
////////////////////////////////////////////////////////////////////////////////////////////////


module RASPredictor #(parameter int StackSize = 16 )(
  input  logic             clk,
  input  logic             reset, 
  input  logic             StallF, StallD, StallE, StallM, FlushD, FlushE, FlushM,
  input  logic             BPReturnWrongD,                      // Prediction class is wrong
  input  logic             ReturnD,
  input  logic             ReturnE, CallE,                  // Instr class
  input  logic             BPReturnF,
  input  logic [32-1:0] PCLinkE,                                   // PC of instruction after a call
  output logic [32-1:0] RASPCF                                     // Top of the stack
   );

  logic                     CounterEn;
  localparam Depth = $clog2(StackSize);

  logic [Depth-1:0]         NextPtr, Ptr, P1, M1, IncDecPtr;
  logic [StackSize-1:0]     [32-1:0] memory;
  integer        index;

  logic      PopF;
  logic      PushE;
  logic      RepairD;
  logic      IncrRepairD, DecRepairD;
  
  logic      DecrementPtr;
  logic      FlushedReturnDE;
  logic      WrongPredReturnD;
  
  
  assign PopF = BPReturnF & ~StallD & ~FlushD;
  assign PushE = CallE & ~StallM & ~FlushM;

  assign WrongPredReturnD = (BPReturnWrongD) & ~StallE & ~FlushE;
  assign FlushedReturnDE = (~StallE & FlushE & ReturnD) | (FlushM & ReturnE); // flushed return

  assign RepairD = WrongPredReturnD | FlushedReturnDE ;

  assign IncrRepairD = FlushedReturnDE | (WrongPredReturnD & ~ReturnD); // Guessed it was a return, but its not

  assign DecRepairD =  WrongPredReturnD & ReturnD; // Guessed non return but is a return.
    
  assign CounterEn = PopF | PushE | RepairD;

  assign DecrementPtr = (PopF | DecRepairD) & ~IncrRepairD;

  assign P1 = 1;
  assign M1 = '1; // -1
  mux2 #(Depth) PtrMux(P1, M1, DecrementPtr, IncDecPtr);
  assign NextPtr = Ptr + IncDecPtr;

  flopenr #(Depth) PTR(clk, reset, CounterEn, NextPtr, Ptr);

  // RAS must be reset. 
  always_ff @ (posedge clk) begin
    if(reset) begin
      for(index=0; index<StackSize; index++)
    memory[index] <= {32{1'b0}};
    end else if(PushE) begin
      memory[NextPtr] <= #1 PCLinkE;
    end
  end

  assign RASPCF = memory[Ptr];
  
  
endmodule
///////////////////////////////////////////
// btb.sv
//
// Written: Ross Thompson ross1728@gmail.com
// Created: February 15, 2021
// Modified: 24 January 2023 
//
// Purpose: Branch Target Buffer (BTB). The BTB predicts the target address of all control flow instructions.
//          It also guesses the type of instrution; jalr(r), return, jump (jr), or branch.
//
// Documentation: RISC-V System on Chip Design Chapter 10 (Figure ***)
// 
// A component of the CORE-V-WALLY configurable RISC-V project.
// 
// Copyright (C) 2021-23 Harvey Mudd College & Oklahoma State University
//
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
//
// Licensed under the Solderpad Hardware License v 2.1 (the “License”); you may not use this file 
// except in compliance with the License, or, at your option, the Apache License version 2.0. You 
// may obtain a copy of the License at
//
// https://solderpad.org/licenses/SHL-2.1/
//
// Unless required by applicable law or agreed to in writing, any work distributed under the 
// License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, 
// either express or implied. See the License for the specific language governing permissions 
// and limitations under the License.
////////////////////////////////////////////////////////////////////////////////////////////////


module btb #(parameter Depth = 10 ) (
  input  logic             clk,
  input  logic             reset,
  input  logic             StallF, StallD, StallE, StallM, StallW, FlushD, FlushE, FlushM, FlushW,
  input  logic [32-1:0] PCNextF, PCF, PCD, PCE, PCM, // PC at various stages
  output logic [32-1:0] BPBTAF,                      // BTB's guess at PC
  output logic [32-1:0] BPBTAD,
  output logic [32-1:0] BPBTAE,
  output logic [3:0]       BTBIClassF,                  // BTB's guess at instruction class
  // update
  input  logic             IClassWrongM,                // BTB's instruction class guess was wrong
  input  logic             IClassWrongE,
  input  logic [32-1:0] IEUAdrE,                     // Branch/jump target address to insert into btb
  input  logic [32-1:0] IEUAdrM,                     // Branch/jump target address to insert into btb
  input  logic [3:0]       InstrClassD,                 // Instruction class to insert into btb
  input  logic [3:0]       InstrClassE,                 // Instruction class to insert into btb
  input  logic [3:0]       InstrClassM,                 // Instruction class to insert into btb
  input  logic [3:0]       InstrClassW
);

  logic [Depth-1:0]        PCNextFIndex, PCFIndex, PCDIndex, PCEIndex, PCMIndex, PCWIndex;
  logic [32-1:0]        ResetPC;
  logic                    MatchD, MatchE, MatchM, MatchW, MatchX;
  logic [32+3:0]        ForwardBTBPrediction, ForwardBTBPredictionF;
  logic [32+3:0]        TableBTBPredF;
  logic [32-1:0]        IEUAdrW;
  logic [32-1:0]        PCW;
  logic                    BTBWrongE, BPBTAWrongE;
  logic                    BTBWrongM, BPBTAWrongM;
  
  
  // hashing function for indexing the PC
  // We have Depth bits to index, but XLEN bits as the input.
  // bit 0 is always 0, bit 1 is 0 if using 4 byte instructions, but is not always 0 if
  // using compressed instructions.  XOR bit 1 with the MSB of index.
  assign PCFIndex = {PCF[Depth+1] ^ PCF[1], PCF[Depth:2]};
  assign PCDIndex = {PCD[Depth+1] ^ PCD[1], PCD[Depth:2]};
  assign PCEIndex = {PCE[Depth+1] ^ PCE[1], PCE[Depth:2]};
  assign PCMIndex = {PCM[Depth+1] ^ PCM[1], PCM[Depth:2]};
  assign PCWIndex = {PCW[Depth+1] ^ PCW[1], PCW[Depth:2]};

  // must output a valid PC and valid bit during reset.  Because only PCF, not PCNextF is reset, PCNextF is invalid
  // during reset.  The BTB must produce a non X PC1NextF to allow the simulation to run.
  // While the mux could be included in IFU it is not necessary for the IROM/I$/bus.
  // For now it is optimal to leave it here.
  assign ResetPC = 32'h80000000;
  assign PCNextFIndex = reset ? ResetPC[Depth+1:2] : {PCNextF[Depth+1] ^ PCNextF[1], PCNextF[Depth:2]}; 

  assign MatchD = PCFIndex == PCDIndex;
  assign MatchE = PCFIndex == PCEIndex;
  assign MatchM = PCFIndex == PCMIndex;
  assign MatchW = PCFIndex == PCWIndex;
  assign MatchX = MatchD | MatchE | MatchM | MatchW;

  assign ForwardBTBPredictionF = MatchD ? {InstrClassD, BPBTAD} :
                                 MatchE ? {InstrClassE, IEUAdrE} :
                                 MatchM ? {InstrClassM, IEUAdrM} :
                                 {InstrClassW, IEUAdrW} ;

  assign {BTBIClassF, BPBTAF} = MatchX ? ForwardBTBPredictionF : {TableBTBPredF};


  // An optimization may be using a PC relative address.
  ram2p1r1wbe #(2**Depth, 32+4) memory(
    .clk, .ce1(~StallF | reset), .ra1(PCNextFIndex), .rd1(TableBTBPredF),
     .ce2(~StallW & ~FlushW), .wa2(PCMIndex), .wd2({InstrClassM, IEUAdrM}), .we2(BTBWrongM), .bwe2('1));

  flopenrc #(32) BTBD(clk, reset, FlushD, ~StallD, BPBTAF, BPBTAD);

  // BPBTAE is not strickly necessary.  However it is used by two parts of wally.
  // 1. It gates updates to the BTB when the prediction does not change.  This save power.
  // 2. BPBTAWrongE is used by the performance counters to track when the BTB's BPBTA or instruction class is wrong.
  flopenrc #(32) BTBTargetEReg(clk, reset, FlushE, ~StallE, BPBTAD, BPBTAE);
  assign BPBTAWrongE = (BPBTAE != IEUAdrE) & (InstrClassE[0] | InstrClassE[1] & ~InstrClassE[2]);

  flopenrc #(1) BPBTAWrongMReg(clk, reset, FlushM, ~StallM, BPBTAWrongE, BPBTAWrongM);  
  assign BTBWrongM = BPBTAWrongM | IClassWrongM;
  
  flopenr #(32) PCWReg(clk, reset, ~StallW, PCM, PCW);
  flopenr #(32) IEUAdrWReg(clk, reset, ~StallW, IEUAdrM, IEUAdrW);

endmodule
///////////////////////////////////////////
// satCounter2.sv
//
// Written: Ross Thomposn
// Email: ross1728@gmail.com
// Created: February 13, 2021
// Modified: 
//
// Purpose: 2 bit starting counter
// 
// A component of the CORE-V-WALLY configurable RISC-V project.
// 
// Copyright (C) 2021-23 Harvey Mudd College & Oklahoma State University
//
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
//
// Licensed under the Solderpad Hardware License v 2.1 (the “License”); you may not use this file 
// except in compliance with the License, or, at your option, the Apache License version 2.0. You 
// may obtain a copy of the License at
//
// https://solderpad.org/licenses/SHL-2.1/
//
// Unless required by applicable law or agreed to in writing, any work distributed under the 
// License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, 
// either express or implied. See the License for the specific language governing permissions 
// and limitations under the License.
////////////////////////////////////////////////////////////////////////////////////////////////


module satCounter2
  (input logic        BrDir,
   input logic [1:0]  OldState,
   output logic [1:0] NewState
   );

  always_comb begin
    case(OldState)
      2'b00: begin
 if(BrDir) NewState = 2'b01;
 else NewState = 2'b00;
      end
      2'b01: begin
 if(BrDir) NewState = 2'b10;
 else NewState = 2'b00;
      end
      2'b10: begin
 if(BrDir) NewState = 2'b11;
 else NewState = 2'b01;
      end
      2'b11: begin
 if(BrDir) NewState = 2'b11;
 else NewState = 2'b10;
      end
    endcase
  end

endmodule
