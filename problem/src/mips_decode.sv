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

// Include the MIPS constants
`include "mips_defines.vh"
`include "internal_defines.vh"

////
//// mips_decode: Decode MIPS instructions
////
//// op      (input)  - Instruction opcode
//// funct2  (input)  - Instruction minor opcode
//// rt      (input)  - Instruction minor opcode
//// alu_sel (output) - Selects the ALU function
//// we      (output) - Write to the register file
//// Sys     (output) - System call exception
//// RI      (output) - Reserved instruction exception
////
module mips_decode(/*AUTOARG*/
   // Outputs
   ctrl_we, ctrl_Sys, ctrl_RI, alu__sel, alu__src, mem_to_reg, ins_type, imm_sign, is_shift, mem_read_bytes, mem_write_bytes, mem_write_en,
   // Inputs
   dcd_op, dcd_funct2
   );

   input       [5:0] dcd_op, dcd_funct2;
   output reg     [1:0] ins_type; 
   output reg        ctrl_we, ctrl_Sys, ctrl_RI, alu__src, mem_to_reg, imm_sign, is_shift; 
   output reg  [3:0] alu__sel, mem_write_en;
   output reg [2:0] mem_read_bytes, mem_write_bytes; // could be 1, 2, or 4 depending on b, h, or w
   

   wire        [2:0] op_major; 
   wire        [2:0] op_minor; 
   assign op_major = dcd_op[5:3]; 
   assign op_minor = dcd_op[2:0]; 
   

   always @(*) begin
     alu__sel = 4'hx;
     ctrl_we = 1'b0;
     ctrl_Sys = 1'b0;
     ctrl_RI = 1'b0;
     ins_type = `R_TYPE; 
     imm_sign = 1'b1; 
     is_shift = 1'b0; 
     mem_write_en = 1'b0; 

     case (op_major) 
        `OP_ITYPE: 
          begin 
            ins_type = `I_TYPE; 

            alu__src = 1'b1; 
            mem_to_reg = 1'b0; 
            ctrl_we = 1'b1; 
            

            case(op_minor) 
              'h0: 
                alu__sel = `ALU_ADD; 
              'h1: 
                alu__sel = `ALU_ADDU; 
              'h2: 
                alu__sel = `ALU_SUB; 
              'h3: 
                alu__sel = `ALU_SUBU; 
              'h4: 
                alu__sel = `ALU_AND; 
              'h5:
                alu__sel = `ALU_OR; 
              'h6: 
                alu__sel = `ALU_XOR; 
              'h7: 
                alu__sel = `ALU_ADD; 
            endcase 
          end 
        `OP_LOAD: 
          begin 
            ins_type = `I_TYPE; 
            alu__src = 1'b1; 
            mem_to_reg = 1'b1; 
            ctrl_we = 1'b1; 
            alu__sel = `ALU_ADD; 
            imm_sign = 1'b1; 
            mem_read_bytes = 1; 
            case (dcd_op) 
            `OP_LBU: 
            begin
              imm_sign = 1'b0; 
              mem_read_bytes = 1; 
            end
            `OP_LHU: 
            begin
              imm_sign = 1'b0; 
              mem_read_bytes = 2; 
            end
            `OP_LB: 
              mem_read_bytes = 1; 
            `OP_LH: 
              mem_read_bytes = 2; 
            `OP_LW: 
              mem_read_bytes = 4; 
            endcase 
            


          end 
        `OP_STORE: 
          begin 
            ins_type = `I_TYPE; 
            alu__src = 1'b1; 
            mem_to_reg = 1'b1; // note -- we keep this on so that the datapath interprets the instruction as a memory access/write
            // To prevent the register actually being written, just set reg_we = 0 as we do here -- cool trick :)  
            ctrl_we = 1'b0; 
            alu__sel = `ALU_ADD; 
            imm_sign = 1'b1; 
            mem_read_bytes = 1; 
            mem_write_en = 1'b1; 
            case (dcd_op) 
            `OP_SB: 
            begin
              mem_read_bytes = 1; 
              mem_write_en = 4'b1; 
            end
            `OP_SH: 
            begin
              mem_read_bytes = 2; 
              mem_write_en = 4'b11; 
            end
            `OP_SW: 
            begin
              mem_read_bytes = 4; 
              mem_write_en = 4'b1111; 
            end
            endcase 
            
          end
        `OP_RTYPE: 
          begin 
            alu__src = 1'b0; 
            mem_to_reg = 1'b0; 
            ctrl_we = 1'b1; 

            case (op_minor)
              3'b0: 
                begin 
                  case (dcd_funct2) 
              `OP0_ADD: 
                alu__sel = `ALU_ADD; 
              `OP0_ADDU: 
                alu__sel = `ALU_ADD; 
              `OP0_AND: 
                alu__sel = `ALU_AND; 
              `OP0_NOR: 
                alu__sel = `ALU_NOR; 
              `OP0_XOR: 
                alu__sel = `ALU_XOR; 
              `OP0_OR: 
                alu__sel = `ALU_OR; 
              `OP0_SLT: 
                alu__sel = `ALU_SLT; 
              `OP0_SUB: 
                alu__sel = `ALU_SUB; 
              `OP0_SLT: 
                alu__sel = `ALU_SLT; 
              `OP0_SRL: 
                begin
                  imm_sign = 1'b0; 
                  alu__sel = `ALU_SRL; 
                  is_shift = 1'b1; 
                end
              `OP0_SLL:
                begin 
                  imm_sign = 1'b0; 
                  alu__sel = `ALU_SLL; 
                  is_shift = 1'b1; 
                end
              `OP0_SRA: 
                alu__sel = `ALU_SRA; 
              endcase 
              
              end
              3'b10: 
                // j instruction
                begin 
                  ins_type = `J_TYPE; 
                  ctrl_we = 1'b0; 
                end

              3'b11: // jal instruction 
                ins_type = `J_TYPE; 



            endcase 

            
            
          end 



     endcase 

     case(dcd_op)
       `OP_OTHER0:
         case(dcd_funct2)
           `OP0_SYSCALL:
                ctrl_Sys = 1'b1;
           default:
                ctrl_RI = 1'b1;
         endcase
       `OP_ADDIU:
         begin
            alu__sel = `ALU_ADD;
            ctrl_we = 1'b1;
         end
        `OP_ADDI: 
          begin 
            alu__sel = `ALU_ADD; 
            ctrl_we = 1'b1; 

          end 
        //`OP_SLTI: 
        //`OP_SLTIU: 
        //`OP_ANDI: 
        //`OP_ORI: 

       default:
         begin
            ctrl_RI = 1'b1;
         end
     endcase // case(op)
   end

endmodule
