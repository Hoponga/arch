#include <stdio.h>
#include <stdbool.h>

#include "shell.h"

#define OPCODE_SPECIAL 0x0 
#define OPCODE_IMM 0x1

// modularize code based on pipeline stages 
// should have mem fetch & ins fetch as their own stages but 
// doesn't really matter rn 
void ins_decode(); 
void execute(); 
void writeback(); 
void printBits(uint32_t num); 


struct {
    uint32_t instruction; 
    bool is_immediate; 
    bool is_reg; 
    uint32_t imm_instr; 
    bool alu_signed; 
    bool reg_write; 
    bool mem_to_reg; 
    bool mem_write; 
    

} control_signals; 

enum ALU_control{ADD, SLTI, SUB, AND, OR, XOR, NOR}; 

typedef struct alu_flags {
    bool overflow; 
    bool negative; 
    bool zero; 

} flags; 

struct {
    uint32_t aluA; 
    uint32_t aluB; 
    enum ALU_control aluOp; 
    uint32_t aluControl; 
    uint32_t writeData; 
    uint32_t writeAddr; 
    uint32_t aluResult; 
    uint32_t memAddr; 
    uint32_t rd1; 
    uint32_t rd2; 
    uint32_t memData; 
    uint32_t memOut; 




} datapath; 




void process_instruction()
{
    /* execute one instruction here. You should use CURRENT_STATE and modify
     * values in NEXT_STATE. You can call mem_read_32() and mem_write_32() to
     * access memory. */
    NEXT_STATE = CURRENT_STATE;  // placeholder for now 
    uint32_t instruction = mem_read_32(CURRENT_STATE.PC); 

    printf("%x\n", instruction); 
    printBits(instruction); 


    control_signals.instruction = instruction; 
    
    // 3 stage pipeline 
    ins_decode(); 
    execute(); 
    writeback(); 


    NEXT_STATE.PC = CURRENT_STATE.PC + 4; 

}

void ins_decode() {
    uint32_t instruction = control_signals.instruction; 
    // First, read opcode 
    int opcode1 = (instruction >> 29); // Get 3 MSB of instruction 
    int opcode2 = (instruction >> 26) & 0x7; 

    if (opcode1 == OPCODE_SPECIAL) {
        // register instruction
        control_signals.is_reg = true; 
        // alu control must be chosen depending on funct field 

        
    } else if (opcode1 == OPCODE_IMM) {
        control_signals.mem_to_reg = false; 

        // immediate instruction 
        control_signals.is_immediate = true;  
        control_signals.alu_signed = true;  

        control_signals.imm_instr = opcode2; 

        // 0 for 0 extend, 1 for sign extend 
        bool sign_or_zero_extend = 0; 
        if (opcode2 == 0x0) {
            sign_or_zero_extend = 1; 
            datapath.aluOp = ADD; 

        } else if (opcode2 == 0x1) {
            sign_or_zero_extend = 1; 
            control_signals.alu_signed = false; 
            datapath.aluOp = ADD; 
            
        } else if (opcode2 == 0x2) {
            sign_or_zero_extend = 1; 
            
            datapath.aluOp = SLTI; // THIS IS NOT SUB, BUT INSTEAD SLTI 

        } else if (opcode2 == 0x3) {

            sign_or_zero_extend = 1; 
            control_signals.alu_signed = false; 
            datapath.aluOp = SLTI; 

        } else if (opcode2 == 0x4) {
            sign_or_zero_extend = 0; 

            datapath.aluOp = AND; 

        } else if (opcode2 == 0x5) {
            sign_or_zero_extend = 0; 
            datapath.aluOp = OR; 
        } else if (opcode2 == 0x6) {
            // dw about it 
            datapath.aluOp = XOR; 

        } else if (opcode2 == 0x7) {
            sign_or_zero_extend = 1; 
            datapath.aluOp = ADD; 

        }
        // rs = instr[25:21] 
        uint32_t rs = (instruction >> 21) & 0x1f; 
        printf("\nRs %d \n", rs); 
        // rt = instr[20:16]
        uint32_t rt = (instruction >> 16) & 0x1f;  

        if (opcode2 != 0x7) {

            datapath.aluA = CURRENT_STATE.REGS[rs]; 
        } else {
            datapath.aluA = 0; 

        }

        // imm is instr[15:0]
        // sign or zero extend depending on what instruction is being run 
        uint32_t imm = instruction & 0xffff; 
        if (sign_or_zero_extend == 1) {
            if ((imm >> 15) == 1) {
                imm = imm | 0xffff0000; 

            }

        }
    
        datapath.aluB = imm; 
        datapath.writeAddr = rt; 
        // all immediate instructions require reg_write 
        control_signals.reg_write = true; 

        

    }

}

void execute() {
    // dummy alu code 
    switch(datapath.aluOp) {
        case ADD: 
            datapath.aluResult = datapath.aluA + datapath.aluB; 
            break; 
        case SUB: 
            datapath.aluResult = datapath.aluA - datapath.aluB; 
            break;
        case SLTI: 
            datapath.aluResult = ((datapath.aluA - datapath.aluB) < 0) ? 1 : 0; 
            break; 
        case OR: 
            datapath.aluResult = datapath.aluA | datapath.aluB; 
            break; 
        case XOR: 
            // I dont think this is actually supported 
            datapath.aluResult = datapath.aluA ^ datapath.aluB; 
            break; 
        case NOR: 
            datapath.aluResult = ~(datapath.aluA | datapath.aluB); 
            break; 
        case AND: 
            datapath.aluResult = datapath.aluA & datapath.aluB; 
            break; 

    }
    datapath.memAddr = datapath.aluResult; 
    datapath.memData = datapath.rd2; 

    

}

void writeback() {
    datapath.memOut = mem_read_32(datapath.memAddr); 
    if (control_signals.mem_write) {
        mem_write_32(datapath.memAddr, datapath.memData); 

    }
    datapath.writeData = (control_signals.mem_to_reg) ? datapath.memOut : datapath.aluResult; 

    if (control_signals.reg_write) {

        NEXT_STATE.REGS[datapath.writeAddr] = datapath.writeData; 
    }


}

// Helper to visualize instructions by bits 
void printBits(uint32_t num)
{

   for(int bit= sizeof(num)*8; bit > 0; bit--)
   {
      printf("%i", num & 0x01);
      num = num >> 1;
   }
}
