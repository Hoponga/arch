module PatternHistoryTable(clk, rst_b, write_en, taken, branch_history, saturating_counter); 
    input clk, rst_b, write_en, taken; 
    input [11:0] branch_history; 
    output wire [1:0] saturating_counter; 

    parameter HISTORY_LEN = 12; // global history register has 12 bits for previous taken branches 
    parameter PHT_SIZE = 1 << HISTORY_LEN; 
    parameter SAT_WIDTH = 2; // two-bit saturating counter

    localparam STRONGLY_NOT_TAKEN = 2'b00; 
    localparam WEAKLY_NOT_TAKEN = 2'b01; 
    localparam WEAKLY_TAKEN = 2'b11; 
    localparam STRONGLY_TAKEN = 2'b10; 



    reg [SAT_WIDTH-1:0] ph_table [0:PHT_SIZE-1]; 
    always @(posedge clk) begin 
        if (write_en) begin 
            case (ph_table[branch_history]) 
            STRONGLY_NOT_TAKEN: 
                ph_table[branch_history] <= (taken) ? WEAKLY_NOT_TAKEN : STRONGLY_NOT_TAKEN; 
            WEAKLY_NOT_TAKEN: 
                ph_table[branch_history] <= (taken) ? WEAKLY_TAKEN : STRONGLY_NOT_TAKEN; 
            WEAKLY_TAKEN: 
                ph_table[branch_history] <= (taken) ? STRONGLY_TAKEN : WEAKLY_NOT_TAKEN; 
            STRONGLY_TAKEN: 
                ph_table[branch_history] <= (taken) ? STRONGLY_TAKEN : WEAKLY_TAKEN; 
            endcase 
        end 
    end 
    assign saturating_counter = ph_table[branch_history]; 



endmodule 