/*
 *
 * Redistributions of any form whatsoever must retain and/or include the
 * following acknowledgment, notices and disclaimer:
 *
 * This product includes software developed by Carnegie Mellon University. 
 *
 * Copyright (c) 2004 by Babak Falsafi and James Hoe,
 * Computer Architecture Lab at Carnegie Mellon (CALCM), 
 * Carnegie Mellon University.
 *
 * This source file was written and maintained by Jared Smolens 
 * as part of the Two-Way In-Order Superscalar project for Carnegie Mellon's 
 * Introduction to Computer Architecture course, 18-447. The source file
 * is in part derived from code originally written by Herman Schmit and 
 * Diana Marculescu.
 *
 * You may not use the name "Carnegie Mellon University" or derivations 
 * thereof to endorse or promote products derived from this software.
 *
 * If you modify the software you must place a notice on or within any 
 * modified version provided or made available to any third party stating 
 * that you have modified the software.  The notice shall include at least 
 * your name, address, phone number, email address and the date and purpose 
 * of the modification.
 *
 * THE SOFTWARE IS PROVIDED "AS-IS" WITHOUT ANY WARRANTY OF ANY KIND, EITHER 
 * EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO ANYWARRANTY 
 * THAT THE SOFTWARE WILL CONFORM TO SPECIFICATIONS OR BE ERROR-FREE AND ANY 
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, 
 * TITLE, OR NON-INFRINGEMENT.  IN NO EVENT SHALL CARNEGIE MELLON UNIVERSITY 
 * BE LIABLE FOR ANY DAMAGES, INCLUDING BUT NOT LIMITED TO DIRECT, INDIRECT, 
 * SPECIAL OR CONSEQUENTIAL DAMAGES, ARISING OUT OF, RESULTING FROM, OR IN 
 * ANY WAY CONNECTED WITH THIS SOFTWARE (WHETHER OR NOT BASED UPON WARRANTY, 
 * CONTRACT, TORT OR OTHERWISE).
 *
 */

//////
////// MIPS 447: A single-cycle MIPS ISA simulator
//////

// Include the MIPS constants
`include "mips_defines.vh"
`include "internal_defines.vh"

////
//// The MIPS standalone processor module
////
////   clk          (input)  - The clock
////   inst_addr    (output) - Address of instruction to load
////   inst         (input)  - Instruction from memory
////   inst_excpt   (input)  - inst_addr not valid
////   mem_addr     (output) - Address of data to load
////   mem_data_in  (output) - Data for memory store
////   mem_data_out (input)  - Data from memory load
////   mem_write_en (output) - Memory write mask
////   mem_excpt    (input)  - mem_addr not valid
////   halted       (output) - Processor halted
////   reset        (input)  - Reset the processor
////   

module mips_core(/*AUTOARG*/
   // Outputs
   inst_addr, mem_addr, mem_data_in, mem_write_en, halted,
   // Inputs
   clk, inst_excpt, mem_excpt, inst, mem_data_out, rst_b
   );
   
   parameter text_start  = 32'h00400000; /* Initial value of $pc */

   // Core Interface
   input         clk, inst_excpt, mem_excpt;
   output [29:0] inst_addr;
   output [29:0] mem_addr;
   input  [31:0] inst, mem_data_out;
   output [31:0] mem_data_in;
   output [3:0]  mem_write_en;
   output        halted;
   input         rst_b;

   // // for now have every pipeline register be 128 bits long -- can always prune later 
   // wire [127:0] if_out, id_in, id_out, ex_in, ex_out, mem_in, mem_out, wb_in;  // bitstrings for the pipeline registers 
   // wire [127:0]   IFID;  // Pipeline register between instruction fetch & instruction decode 
   // wire [127:0]   IDEX;  // pipeline register between instruction decode & execute 
   // wire [127:0]   EXMEM; // pipeline register between execute & memory 
   // wire [127:0]   MEMWB; // pipeline register between memory access & writeback

   

   



   // Forced interface signals -- required for synthesis to work OK.
   
   // This is probably not what you want!
   

   // Internal signals
   wire [31:0] nextpc, branch_pc; 
   wire [31:0]   pc, nextnextpc, jump_target, branchtarget;
   wire [2:0] pc_src; 
   wire          exception_halt, syscall_halt, internal_halt;
   wire          load_epc, load_bva, load_bva_sel;
   wire [31:0]   rt_data, rs_data, rd_data, alu__out, r_v0;
   wire [31:0]   epc, cause, bad_v_addr;
   wire [4:0]    cause_code;
   wire [3:0]    alu_flags; 

   wire [31:0] if_controls, id_controls, ex_controls, mem_controls, wb_controls; 
   wire [31:0] if_controls_out, id_controls_out, ex_controls_out, mem_controls_out, wb_controls_out; 
   wire [255:0] id_in, if_out, id_out, ex_in, ex_out, mem_in, mem_out, wb_in; 

   // Decode signals
   wire [31:0]   dcd_se_imm, dcd_se_offset, dcd_e_imm, dcd_se_mem_offset;
   wire [5:0]    dcd_op, dcd_funct2;
   wire [4:0]    dcd_rs, dcd_funct1, dcd_rt, dcd_rd, dcd_shamt;
   wire [15:0]   dcd_offset, dcd_imm;
   wire [25:0]   dcd_target;
   wire [19:0]   dcd_code;
   wire          dcd_bczft, imm_sign, is_shift;
   wire [1:0]    ins_type; 
   wire [4:0]    regfile_write_addr; 
   wire [31:0]   regfile_write_data; 
   wire [31:0]   alu_input_2; 
   wire [31:0]   imm_extend; 
   wire [31:0]   alu_input_1; 
   wire [2:0]    mem_read_bytes, mem_write_bytes; // how many bytes to read from/write to memory -- don't care if not a load/store instruction 

   // synthesis translate_on

   // Let Verilog-Mode pipe wires through for us.  This is another example
   // of Verilog-Mode's power -- undeclared nets get AUTOWIREd up when we
   // run 'make auto'.
   
   /*AUTOWIRE*/
   // Beginning of automatic wires (for undeclared instantiated-module outputs)
   wire [3:0]		alu__sel;		// From Decoder of mips_decode.v
   wire			ctrl_RI;		// From Decoder of mips_decode.v
   wire			ctrl_Sys;		// From Decoder of mips_decode.v
   wire			ctrl_we;		// From Decoder of mips_decode.v
   wire mem_to_reg;     // From Decoder of mips_decode.v 
   
   
   // PC Management
   register #(32, text_start) PCReg(pc, nextpc, clk, ~internal_halt, rst_b);


   // Pipeline registers 


   // Nothing is in the instruction fetch cycle??? 

   // Going into IFID pipeline register -- current and instruction data 
   assign if_out = {32'b0, pc, 32'b0, inst}; 
   assign inst_addr = pc[31:2]; 
   register #(256, 0) IFIDReg(id_in, if_out, clk, ~internal_halt, rst_b); 


   
   // synthesis translate_off
   always @(posedge clk) begin
     // useful for debugging, you will want to comment this out for long programs
     if (rst_b) begin
       $display ( "\n=== Simulation Cycle %d ===", $time );
      //  $display ("[branch_result = %d, nextpc = %x, nextnextpc = %x, pc_src = %d, jumptarget = %x, branchtarget = %x]", 
      //             branch_result, nextpc, nextnextpc, pc_src, jump_target, branchtarget);
      $display("Fetch Stage: [pc=%x, inst=%x]",
                   pc, inst);
      $display("Decode Stage: [pc = %x, inst = %x] [op=%x, rs=%d, rt=%d, rd=%d, imm=%x, f2=%x] [reset=%d, halted=%d] [ctrl_we=%d, rs_data = %d, imm_extend = %d, rt_data = %d]  ", id_pc, id_inst, dcd_op, dcd_rs, dcd_rt, dcd_rd, dcd_imm, dcd_funct2, ~rst_b, halted, ctrl_we, rs_data, imm_extend, 
                   rt_data); 
      $display("Execute Stage: [alu_op1 = %d, alu_op2 = %d, alu_out = %d, alu__sel = %x, alu_src = %x, pc = %x] ", alu_input_1, alu_input_2, alu__out, ex_alu__sel, ex_alu__src, ex_pc); 
      $display("Memory Stage: [mem_addr = %x, mem_write_data = %x, mem_read_data = %x, mem_write_en = %x]", mem_addr, mem_data_out, mem_data_in, mem_write_en); 
      $display("Writeback Stage: [mem_to_reg = %x, reg_write_data = %d, reg_write_addr = %d]", mem_to_reg, regfile_write_data, regfile_write_addr); 

     end

   end
   

   // register #(32, text_start+4) PCReg2(nextpc, nextnextpc, clk,
   //                                     ~internal_halt, rst_b);
   wire [31:0] id_pc, id_inst; 
   assign id_pc = id_in[95:64]; 
   assign id_inst = id_in[31:0]; 

   // Instruction decoding
   assign        dcd_op = id_inst[31:26];    // Opcode
   assign        dcd_rs = id_inst[25:21];    // rs field
   assign        dcd_rt = id_inst[20:16];    // rt field
   assign        dcd_rd = id_inst[15:11];    // rd field
   assign        dcd_shamt = id_inst[10:6];  // Shift amount
   assign        dcd_bczft = id_inst[16];    // bczt or bczf?
   assign        dcd_funct1 = id_inst[4:0];  // Coprocessor 0 function field
   assign        dcd_funct2 = id_inst[5:0];  // funct field; secondary opcode
   assign        dcd_offset = id_inst[15:0]; // offset field
        // Sign-extended offset for branches
   assign        dcd_se_offset = { {14{dcd_offset[15]}}, dcd_offset, 2'b00 };
        // Sign-extended offset for load/store
   assign        dcd_se_mem_offset = { {16{dcd_offset[15]}}, dcd_offset };
   assign        dcd_imm = id_inst[15:0];        // immediate field
   assign        dcd_e_imm = { 16'h0, dcd_imm };  // zero-extended immediate
        // Sign-extended immediate
   assign        dcd_se_imm = { {16{dcd_imm[15]}}, dcd_imm };
   assign        dcd_target = id_inst[25:0];     // target field
   assign        dcd_code = id_inst[25:6];       // Breakpoint code


   // Generate control signals
   mips_decode Decoder(/*AUTOINST*/
		       // Outputs
		       .ctrl_we		(ctrl_we),
		       .ctrl_Sys	(ctrl_Sys),
		       .ctrl_RI		(ctrl_RI),
		       .alu__sel	(alu__sel[3:0]),
           .alu__src  (alu__src), 
           .mem_to_reg(mem_to_reg),
           .ins_type(ins_type),
           .imm_sign(imm_sign),
           .is_shift(is_shift),
           .mem_read_bytes(mem_read_bytes),
           .mem_write_en(mem_write_en), 
           .mem_write_bytes(mem_write_bytes),

		       // Inputs
		       .dcd_op		(dcd_op[5:0]),
		       .dcd_funct2	(dcd_funct2[5:0]));


   // following decode, all the control signals must flow into the IDEX register 

   
   // control signals should all go into instruction decode
   // Execute stage needs: 
   // Control signals: alu__sel, alu__src, mem_to_reg, ctrl_we, imm_sign, is_shift -- for now, don't worry about branch controls 

   assign id_controls = {'0, mem_write_bytes, mem_read_bytes, mem_write_en, alu__sel, alu__src, mem_to_reg, ctrl_we, imm_sign, is_shift};

   wire [4:0] id_regfile_write_addr; 
   assign id_regfile_write_addr = (ins_type == `J_TYPE && ctrl_we) ? 'hffffffff : (ins_type == `I_TYPE) ? dcd_rt : dcd_rd; 

   // Execute
   assign imm_extend = (mem_to_reg) ? dcd_se_mem_offset : 
                        (is_shift) ? dcd_shamt :
                         (imm_sign) ? dcd_se_imm : 
                         dcd_e_imm; 

   // Data: PC, rs_data1,  rt_data1, sign extended immediate, reg_write destination
   assign id_out = {id_regfile_write_addr, (is_shift) ? dcd_shamt : rs_data, rt_data, imm_extend, id_in[95:64]}; 
   // INSTRUCTION DECODE END -- pass data into IDEXReg and IDEXControlsReg 

   // Register File
   // Instantiate the register file from reg_file.v here.

   

   regfile RegisterFile(
    .rs_num(dcd_rs), 
    .rt_num(dcd_rt), 
    .rd_num(regfile_write_addr), 
    .rd_data(regfile_write_data), 
    .rd_we(ctrl_we), 
    .clk(clk), 
    .rst_b(rst_b),
    .halted(halted), 
    .rs_data(rs_data), 
    .rt_data(rt_data)

   ); 



   register #(256, 0) IDEXReg(ex_in, id_out, clk, ~internal_halt, rst_b); 
   register #(32, 0) IDEXControlsReg(ex_controls, id_controls, clk, ~internal_halt, rst_b); 

   wire ex_is_shift, ex_alu__src, ex_mem_to_reg, ex_mem_write_en; 
   wire [3:0] ex_alu__sel; 
   wire [2:0] ex_mem_read_bytes, ex_mem_write_bytes; 
   wire [4:0] ex_regfile_write_addr; 

   wire [31:0] ex_rs_data, ex_rt_data, ex_imm_extend, ex_pc; 

   // input controls: 
   // assign id_controls = {'0, mem_write_bytes, mem_read_bytes, mem_write_en, alu__sel, alu__src, mem_to_reg, ctrl_we, imm_sign, is_shift};
   assign ex_is_shift = ex_controls[0]; 
   assign ex_alu__sel = ex_controls[8:5]; 
   assign ex_alu__src = ex_controls[4];
   assign ex_mem_to_reg =  ex_controls[3]; 
   assign ex_mem_write_en = ex_controls[9]; 
   assign ex_mem_read_bytes = ex_controls[12:10]; 
   assign ex_mem_write_bytes = ex_controls[15:13]; 

   // input data: 
   // assign id_out = {rs_data, rt_data, imm_extend, pc}; 
   assign ex_regfile_write_addr = ex_in[132:128]; // carry over all the way to writeback stage 
   assign ex_rs_data = ex_in[127:96];     // assuming if the alu needs a shift value, rs_data will contain it 
   assign ex_rt_data =  ex_in[95:64]; 
   assign ex_imm_extend = ex_in[63:32]; 
   assign ex_pc = ex_in[31:0];      // leave PC here for fun 
   


   assign alu_input_1 = ex_rs_data; 
   assign alu_input_2 = (ex_alu__src) ? ex_imm_extend : ex_rt_data;

   mips_ALU ALU(.alu__out(alu__out), 
                .alu__op1(alu_input_1),
                .alu__op2(alu_input_2),
                .alu__sel(ex_alu__sel),
                .alu_flags(alu_flags));



   // output from execute stage -- the memory requires write_data = rt_data, read_address, 
   assign ex_out = {'0, ex_regfile_write_addr, alu_flags, rt_data, alu__out}; 
   assign ex_controls_out =  {'0, mem_write_bytes, mem_read_bytes, mem_write_en, mem_to_reg, ctrl_we}; 


   register #(256, 0) EXMEMReg(mem_in, ex_out, clk, ~internal_halt, rst_b);   
   register #(32, 0) EXMEMControlsReg(mem_controls, ex_controls_out, clk, ~internal_halt, rst_b); 

   // carry over control signals 

   wire mem_ctrl_we, mem_mem_to_reg, mem_write_en; 
   wire [2:0] mem_mem_read_bytes, mem_mem_write_bytes; 

   // wire [31:0] mem_addr, mem_data_in; 
   wire [4:0] mem_regfile_write_addr; 


   assign mem_ctrl_we = mem_controls[0]; 
   assign mem_mem_to_reg = mem_controls[1];
   assign mem_write_en = mem_controls[2];       // Assign the ACTUAL memory write enable signal that goes to mips_mem to the incoming control signal 
   assign mem_mem_read_bytes = mem_controls[5:3]; 
   assign mem_mem_write_bytes = mem_controls[8:6]; 

   assign mem_addr = mem_in[29:0] >> 2;
   assign mem_data_in = mem_in[63:32]; 
   assign mem_regfile_write_addr = mem_in[72:68]; 

   

   assign mem_out = {'0, mem_regfile_write_addr, mem_in[31:0], mem_data_out}; 


   assign mem_controls_out = {'0, mem_mem_to_reg, mem_ctrl_we}; 
   register #(256, 0) MEMWBReg(wb_in, mem_out, clk, ~internal_halt, rst_b);  
   register #(32, 0)  MEMWBControlsReg(wb_controls, mem_controls_out, clk, ~internal_halt, rst_b); 
   

   // // for now, the only control signals we need for writeback are mem_to_reg and ctrl_we; 
   // assign regfile_write_data = (ins_type == `J_TYPE && ctrl_we) ? (pc + 4) : (mem_to_reg) ? mem_data_out : alu__out; 

   // eventually build up to jump instructions lol

   wire wb_ctrl_we, wb_mem_to_reg; 
   wire [31:0] wb_alu_out, wb_regfile_write_addr, wb_mem_data_out;


   assign wb_ctrl_we = wb_controls[0]; 
   assign wb_mem_to_reg = wb_controls[1]; 

   assign wb_mem_data_out = wb_in[31:0]; 
   assign wb_alu_out = wb_in[63:32]; 
   assign wb_regfile_write_addr = wb_in[95:64]; 

   assign regfile_write_addr = wb_regfile_write_addr; 

   assign regfile_write_data = (wb_mem_to_reg) ? wb_mem_data_out : wb_alu_out; 


 

   // cursed branching code 
   

   assign branch_result = ((dcd_op == `OP_BEQ) && alu_flags[3] == 1) || ((dcd_op == `OP_BNE) && alu_flags[3] != 1); 
   

   // assign        mem_addr = (mem_to_reg) ? (alu__out[29:0] >> 2) : 0;
   // assign        mem_data_in = rt_data;



   assign jump_target = {pc[31:28], dcd_target, 2'b00}; 
   assign branchtarget = pc + 4 + dcd_se_offset; 
   // assign nextnextpc = nextpc + 4; 
   assign branch_pc = (branch_result) ? branchtarget : pc + 4; 



   assign nextpc = (pc_src == 3'b1) ? jump_target : 
                   (pc_src == 3'b11 && branch_result == 1'b1) ? branchtarget : 
                        pc + 4;
   assign nextnextpc = (pc_src == 3'b1) ? jump_target + 4 : 
                   (pc_src == 3'b11 && branch_result == 1'b1) ? branchtarget + 4: 
                        nextpc + 4; 

   assign pc_src = (ins_type == `J_TYPE) ? 3'b1 : (ins_type == `B_TYPE) ? 3'b11 : 3'b0; 


   // // this block is basically our pc_sel mux 
   // always @(*) begin 
   //    case (pc_src) 
   //       3'b1: // jump instruction 
   //          nextpc = jump_target; 
   //       3'b11:   // branch instruction 
   //          nextpc = (branch_result) ? branchtarget : nextpc; 
   //    endcase 
      
   // end


 

   
   // Miscellaneous stuff (Exceptions, syscalls, and halt)
   exception_unit EU(.exception_halt(exception_halt), .pc(pc), .rst_b(rst_b),
                     .clk(clk), .load_ex_regs(load_ex_regs),
                     .load_bva(load_bva), .load_bva_sel(load_bva_sel),
                     .cause(cause_code),
                     .IBE(inst_excpt),
                     .DBE(1'b0),
                     .RI(ctrl_RI),
                     .Ov(1'b0),
                     .BP(1'b0),
                     .AdEL_inst(pc[1:0]?1'b1:1'b0),
                     .AdEL_data(1'b0),
                     .AdES(1'b0),
                     .CpU(1'b0));

   assign r_v0 = 32'h0a; // Good enough for now. To support syscall for real,
                         // you should read the syscall
                         // argument from $v0 of the register file 

   syscall_unit SU(.syscall_halt(syscall_halt), .pc(pc), .clk(clk), .Sys(ctrl_Sys),
                   .r_v0(r_v0), .rst_b(rst_b));
   assign        internal_halt = exception_halt | syscall_halt;
   register #(1, 0) Halt(halted, internal_halt, clk, 1'b1, rst_b);
   register #(32, 0) EPCReg(epc, pc, clk, load_ex_regs, rst_b);
   register #(32, 0) CauseReg(cause,
                              {25'b0, cause_code, 2'b0}, 
                              clk, load_ex_regs, rst_b);
   register #(32, 0) BadVAddrReg(bad_v_addr, pc, clk, load_bva, rst_b);

endmodule // mips_core


////
//// mips_ALU: Performs all arithmetic and logical operations
////
//// out (output) - Final result
//// in1 (input)  - Operand modified by the operation
//// in2 (input)  - Operand used (in arithmetic ops) to modify in1
//// sel (input)  - Selects which operation is to be performed

module mips_ALU(alu__out, alu_flags, alu__op1, alu__op2, alu__sel);

   output reg [31:0] alu__out;
   output reg [3:0] alu_flags; // from MSB to LSB, is Z C N V 
   input [31:0]  alu__op1, alu__op2;
   input [3:0]   alu__sel;
   wire set_ovflow; 
   wire pre_sign; 
   wire keep_sign_on_ovflow; 
   reg carry, zero; 

   assign pre_sign = alu__op1[31]; 
   
   
   // weird flag creation for N -- https://zipcpu.com/zipcpu/2017/08/11/simple-alu.html 
   assign keep_sign_on_ovflow = ((alu__sel == `ALU_ADD) && (alu__op1[31] == alu__op2[31])) || ((alu__sel == `ALU_SUB && (alu__op1[31] != alu__op2[31]))); 
   always @(*) begin 
      zero = ~|{alu__out  ^ 32'h00000000};
      alu_flags = {zero, carry, alu__out[31] ^ ((keep_sign_on_ovflow) && (pre_sign != alu__out[31])), 1'b0};
   end 
   always @(*) begin 
      
      case (alu__sel)
         `ALU_ADD: 
            {carry, alu__out} = alu__op1 + alu__op2; 
         `ALU_ADDU: 
            alu__out = alu__op1 + alu__op2; // right now, don't care about distinctions between unsigned and signed 
         `ALU_SUB: 
            {carry, alu__out} = alu__op1 - alu__op2; 
         `ALU_SUBU: 
            alu__out = alu__op1 - alu__op2; 
         `ALU_AND: 
            alu__out = alu__op1 & alu__op2; 
         `ALU_OR: 
            alu__out = alu__op1 | alu__op2; 
         `ALU_XOR: 
            alu__out = alu__op1 ^ alu__op2; 
         `ALU_SRL: 
            alu__out = alu__op2 >> alu__op1; 
         `ALU_SRA: 
            alu__out = alu__op2 >>> alu__op1; 
         `ALU_SLL: 
            alu__out = alu__op2 << alu__op1; 
         default: 
            {carry, alu__out} = alu__op1 + alu__op2; 

      endcase 
   end 

endmodule


//// register: A register which may be reset to an arbirary value
////
//// q      (output) - Current value of register
//// d      (input)  - Next value of register
//// clk    (input)  - Clock (positive edge-sensitive)
//// enable (input)  - Load new value?
//// reset  (input)  - System reset
////
module register(q, d, clk, enable, rst_b);

   parameter
            width = 32,
            reset_value = 0;

   output [(width-1):0] q;
   reg [(width-1):0]    q;
   input [(width-1):0]  d;
   input                 clk, enable, rst_b;

   always @(posedge clk or negedge rst_b)
     if (~rst_b)
       q <= reset_value;
     else if (enable)
       q <= d;

endmodule // register


////
//// adder
////
//// out (output) - adder result
//// in1 (input)  - Operand1
//// in2 (input)  - Operand2
//// sub (input)  - Subtract?
////
module adder(out, in1, in2, sub);
   output [31:0] out;
   input [31:0]  in1, in2;
   input         sub;

   assign        out = sub?(in1 - in2):(in1 + in2);

endmodule // adder


////
//// add_const: An adder that adds a fixed constant value
////
//// out (output) - adder result
//// in  (input)  - Operand
////
module add_const(out, in);

   parameter add_value = 1;

   output   [31:0] out;
   input    [31:0] in;

   assign   out = in + add_value;

endmodule // adder

// Local Variables:
// verilog-library-directories:("." "../447rtl")
// End:
