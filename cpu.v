/*
 * Copyright (c) 2018, Itzik Kotler
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

module cpu
(
    input clk,
    input rst,
    output reg [3:0] led,
    output reg [7:0] ja,
    output reg [7:0] jb,
    output reg [7:0] jc,
    output reg [7:0] jd
);
    // Program Counter (i.e., PC)
    reg [7:0] pc, pc_next;

    // Carry & Zero Flags
    reg carry_flag, carry_flag_next, zero_flag, zero_flag_next;

    // Instruction Register
    wire [7:0]   instr;

    // Instruction Decoding
    wire dst_reg = instr[4];
    wire src_reg = instr[3];
    wire [2:0]  branch_condition = instr[5:3];
    wire [2:0]  rel_address = instr[2:0];
    wire [3:0]  immediate = instr[3:0];
    wire [2:0]  alu_opcode = instr[2:0];
    wire [2:0]  mem_address = instr[2:0];

    // Register file
    reg [7:0]   rf[1:0];
    reg         reg_we;
    reg [3:0]   reg_data;
    reg         reg_addr;

    // ROM
    reg [7:0]   rom[100:0];

    // ROM Program (i.e., python asm2rom_v.py -i program.asm)
    initial begin
    rom[0] = 8'b01000000; // lda $0
    rom[1] = 8'b01010001; // ldb $1
    rom[2] = 8'b11110100; // out $1, $0
    // _sleep_led_off:
    rom[3] = 8'b01101100; // add a, b
    rom[4] = 8'b11000111; // cmp a, $7
    rom[5] = 8'b10001110; // jne _sleep_led_off
    rom[6] = 8'b11110101; // out $1, $1
    // _sleep_led_on:
    rom[7] = 8'b01101101; // sub a, b
    rom[8] = 8'b11000000; // cmp a, $0
    rom[9] = 8'b10001110; // jne _sleep_led_on
    rom[10] = 8'b10011000; // jmp $0
    end

    // RAM
    reg [7:0]   ram[7:0];
    reg [7:0]   ram_addr;
    reg [7:0]   ram_in;
    reg         ram_we;

    // Fetch
    assign instr = rom[pc];

    // Decode & Execute
    always @(*) begin
        // Default Values
        ram_we = 0;
        reg_we = 0;

        case (instr[7:5])
        // MOV (A|B|MEM), (A|B|MEM)
        3'b000: begin
            pc_next = pc + 1;
            case (instr[3])
                // MOV (A|B), MEM
                1'b1: begin
                    ram_we = 1'b1;
                    ram_in = rf[dst_reg];
                    ram_addr = mem_address;
                // MOV MEM, (A|B)
                end
                1'b0: begin
                    reg_we = 1'b1;
                    reg_addr = dst_reg;
                    reg_data = ram[mem_address];
                end
                default: begin
                    $display("<%b> UNKNOWN MOV OPCODE", instr);
                end
            endcase
        end
        // LD(A|B), IMM4
        3'b010: begin
            // $display("LD REG #%b WITH: %h", dst_reg, immediate);
            pc_next = pc + 1;
            reg_we = 1'b1;
            reg_addr = dst_reg;
            reg_data = immediate;
            ram_we = 1'b0;
        end
        // ALU
        3'b011: begin
            // $display("ALU");
            case (alu_opcode)
                // OR (A|B), (A|B)
                3'b010: begin
                    pc_next = pc + 1;
                    reg_we = 1'b1;
                    reg_addr = dst_reg;
                    {carry_flag_next, reg_data} = rf[dst_reg] | rf[src_reg];
                end
                // ADD (A|B), (A|B)
                3'b100: begin
                    pc_next = pc + 1;
                    reg_we = 1'b1;
                    reg_addr = dst_reg;
                    {carry_flag_next, reg_data} = rf[dst_reg] + rf[src_reg];
                end
                // SUB (A|B), (A|B)
                3'b101: begin
                    pc_next = pc + 1;
                    reg_we = 1'b1;
                    reg_addr = dst_reg;
                    {carry_flag_next, reg_data} = rf[dst_reg] - rf[src_reg];
                end
                default: begin
                    $display("<%b> UNKNOWN ALU OPCODE", instr);
                end
            endcase
        end
        // BRANCH
        3'b100: begin
            case (branch_condition)
                // If Not Carry
                2'b00: begin
                    // $display("BRANCH JNC");
                    if (carry_flag == 1'b0) begin
                        pc_next = $signed(pc) + $signed(rel_address);
                    end else begin
                        pc_next = pc + 1;
                    end
                end
                // If Not Zero
                2'b01: begin
                    if (zero_flag == 1'b0) begin
                        pc_next = $signed(pc) + $signed(rel_address);
                    end else begin
                        pc_next = pc + 1;
                    end
                end
                // Absolute
                2'b11: begin
                    pc_next = rel_address;
                end
                default: begin
                    $display("<%b> UNKNOWN BRANCH OPCODE", instr);
                end
            endcase
        end
        // Arithmetic Compare
        3'b110: begin
            // $display("REG [%b] == %b", dst_reg, immediate);
            if (rf[dst_reg] == immediate) begin
                zero_flag_next = 1;
            end else begin
                zero_flag_next = 0;
            end
            pc_next = pc + 1;
        end
        // I/O
        3'b111: begin
            // I/O Mode
            case (instr[4])
                // Output
                1'b1: begin
                    jd[instr[2]] = instr[0];
                    pc_next = pc + 1;
                end
            default: begin
                $display("<%b> UNKNOWN I/O OPCODE", instr);
            end
            endcase
        end
        default: begin
            $display("<%b> UNKNOWN OPCODE", instr);
        end
    endcase
end

// Reset (OR) Write-back
always @(posedge clk or negedge rst) begin
    // Async Active Low Reset
    if (~rst) begin
        pc <= 0;
        carry_flag <= 0;
        zero_flag <= 0;
        $display("*** RESET ***");
    end else begin
        ja <= rf[0];
        jb <= rf[1];
        jc <= instr;
        led[3:0] <= pc;
        $display("[%b] RF[0] = %h, RF[1] = %h, PC = %h (NEXT: %h), INSTR = %b, OP = %b, CF = %b, ZF = %b", instr, rf[0], rf[1], pc, pc_next, instr, instr[7:5], carry_flag, zero_flag);

        // Increase Program Counter
        pc <= pc_next;

        // Write to Register File
        if (reg_we) begin
            rf[reg_addr] <= reg_data;
            carry_flag <= carry_flag_next;
        end else begin
            zero_flag <= zero_flag_next;
        end

        // Write to RAM
        if (ram_we) begin
            ram[ram_addr] <= ram_in;
        end
    end
end

endmodule
