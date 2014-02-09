//=============================================================================
// EE108B Lab 2
// Authors: Nipun Agarwala, Charles Guan
// Decode module. Determines what to do with an instruction.
//=============================================================================

`include "mips_defines.v"

module decode (
    input [31:0] pc,
    input [31:0] instr,
    input [31:0] alu_result,
    input [31:0] rs_data,
    input [31:0] rt_data,

    output wire [4:0] reg_write_addr, // destination register number
    output reg branch_en,            // high when the instruction is a taken branch
    output wire jump_en,              // high when the instruction is j or jal
    output wire jump_reg_en,          // high when the instruction is jr or jalr
    output reg [3:0] alu_opcode,      // see mips_defines.v, chooses the function of the ALU
    output wire [31:0] alu_op_x,      // first operand for ALU
    output wire [31:0] alu_op_y,      // second operand for ALU
    output wire mem_write_en,         // high when the instruction is sw
    output wire mem_read_en,          // high when the instruction is lw
    output wire reg_write_en,         // high when the instruction writes to a register
    output wire [4:0] rs_addr,        // rs register number (already set below)
    output wire [4:0] rt_addr         // rt register number (already set below)
);

//******************************************************************************
// instruction fields
//******************************************************************************

    wire [5:0] op = instr[31:26];
    assign rs_addr = instr[25:21];
    assign rt_addr = instr[20:16];
    wire [4:0] rd_addr = instr[15:11];
    wire [4:0] shamt = isLUI ? 5'd16 : instr[10:6];
    wire [5:0] funct = instr[5:0];
    wire [15:0] immediate = instr[15:0];

//******************************************************************************
// branch instructions decode
//******************************************************************************

    // Decides whether the instruction is a taken branch
    // Remember that the ALU result is an input to the decode module, so you can
    // use the ALU to evaluate the branch condition.
    wire is_branch_instr = |{op == `BEQ, op == `BNE, op == `BLEZ, op == `BGTZ,
                             op == `BLTZ_GEZ};
    always @ (*) begin
        case(op)
            `BEQ:  branch_en = rs_data == rt_data;
            `BNE:  branch_en = rs_data != rt_data;
            `BLEZ: branch_en = rs_data <= 32'b0;
            `BGTZ: branch_en = rs_data >  32'b0;
            `BLTZ_GEZ:
                case(rt_addr)
                    `BLTZ:   branch_en =  rs_data[31];
                    `BGEZ:   branch_en = ~rs_data[31];
                    `BLTZAL: branch_en =  rs_data[31];
                    `BGEZAL: branch_en = ~rs_data[31];
                    default: branch_en = 1'b0;
                endcase
            default: branch_en = 1'b0;
        endcase
    end

//******************************************************************************
// jump instructions decode
//******************************************************************************

    // checks whether there is a jump or a jump to register and asserts
    // jump_en or jump_reg_en high if necessary.

    assign jump_en = (op == `J) | (op == `JAL);
    assign jump_reg_en = (op == `JR) | (op == `JALR);

    wire jump_no_link = (op == `J) | (op == `JR);
    wire jump_with_link = (op == `JAL) | (op == `JALR);

//******************************************************************************
// shift instruction decode
//******************************************************************************

    wire isSLL = (op == `SPECIAL) & (funct == `SLL);
    wire isSRA = (op == `SPECIAL) & (funct == `SRA);
    wire isSRL = (op == `SPECIAL) & (funct == `SRL);
    wire isSLLV = (op == `SPECIAL) & (funct == `SLLV);
    wire isSRAV = (op == `SPECIAL) & (funct == `SRAV);
    wire isSRLV = (op == `SPECIAL) & (funct == `SRLV);
    wire isLUI = (op == `LUI);
    
    wire isVarShift = |{isSLLV, isSRAV, isSRLV};
    wire isShift = |{isSLL, isSRA, isSRL, isLUI, isVarShift};

//******************************************************************************
// ALU instructions decode / control signal for ALU datapath
//******************************************************************************
    
    // Matches {op, funct} pairs and the corresponding alu operations
    // Refer to include/mips_defines.v
    // ADD, ADDU, ADDI, ADDIU, SUB, SUBU, SLT, SLTU, SLTI, SLTIU, AND, ANDI,
    // OR, ORI, XOR, XORI, NOR, SRL, SRA, SLL, SRLV, SRAV, SLLV, LUI, SW, LW

    always @* begin
        casex({op, funct})
            {`SPECIAL, `ADD}:  alu_opcode = `ALU_ADD;
            {`SPECIAL, `ADDU}: alu_opcode = `ALU_ADDU;
            {`ADDI,    `DC6}:  alu_opcode = `ALU_ADD;
            {`ADDIU,   `DC6}:  alu_opcode = `ALU_ADDU;
            {`SPECIAL, `SUB}:  alu_opcode = `ALU_SUB;
            {`SPECIAL, `SUBU}: alu_opcode = `ALU_SUBU;
            {`SPECIAL, `SLT}:  alu_opcode = `ALU_SLT;
            {`SPECIAL, `SLTU}: alu_opcode = `ALU_SLTU;
            {`SLTI,    `DC6}:  alu_opcode = `ALU_SLT;
            {`SLTIU,   `DC6}:  alu_opcode = `ALU_SLTU;
            {`SPECIAL, `AND}:  alu_opcode = `ALU_AND;
            {`ANDI,    `DC6}:  alu_opcode = `ALU_AND;
            {`SPECIAL, `OR}:   alu_opcode = `ALU_OR;
            {`ORI,     `DC6}:  alu_opcode = `ALU_OR;
            {`SPECIAL, `XOR}:  alu_opcode = `ALU_XOR;
            {`XORI,    `DC6}:  alu_opcode = `ALU_XOR;
            {`SPECIAL, `NOR}:  alu_opcode = `ALU_NOR;
            {`SPECIAL, `SRL}:  alu_opcode = `ALU_SRL;
            {`SPECIAL, `SRA}:  alu_opcode = `ALU_SRA;
            {`SPECIAL, `SLL}:  alu_opcode = `ALU_SLL;
            {`SPECIAL, `SRLV}: alu_opcode = `ALU_SRL;
            {`SPECIAL, `SRAV}: alu_opcode = `ALU_SRA;
            {`SPECIAL, `SLLV}: alu_opcode = `ALU_SLL;
            {`LUI,     `DC6}:  alu_opcode = `ALU_SLL;
            {`LW,      `DC6}:  alu_opcode = `ALU_ADD;
            {`SW,      `DC6}:  alu_opcode = `ALU_ADD;
            default:           alu_opcode = `ALU_PASSX;
    	endcase
        if (jump_with_link)
            alu_opcode = `ALU_ADD;
    end

//******************************************************************************
// Compute value for 32 bit immediate data
//******************************************************************************

    // zero-extend immediate for bitwise operations
    // for non-bitwise operations, sign-extend or don't care.
    // See MIPS green sheet for reference.

    wire [31:0] imm_sign_extend = {{16{immediate[15]}}, immediate};  
    wire [31:0] imm_zero_extend = {16'b0, immediate};	

    reg [31:0] imm_ext;
    always @(*) begin
        if (|{op == `ORI, op == `ANDI, op == `XORI})
            imm_ext = imm_zero_extend;
        else
            imm_ext = imm_sign_extend;
    end

//******************************************************************************
// Determine ALU inputs and register writeback address
//******************************************************************************

    // sets alu_op_x and alu_op_y based on the kind of operation that
    // must be performed
    wire [31:0] shift_amount = {27'b0, isVarShift ? rs_data[4:0] : shamt};
    assign alu_op_x = jump_with_link ? pc : (isShift ? shift_amount : rs_data);

    // use immediate operand for I-format instructions
    // don't use imm_operand for R-format (op == `SPECIAL)
    wire use_imm_operand = &{op != `SPECIAL, ~is_branch_instr};

    assign alu_op_y = jump_with_link ? 32'd4 : (use_imm_operand ? imm_ext : rt_data);
    assign reg_write_addr = jump_with_link ? 5'd31 : (use_imm_operand ? rt_addr : rd_addr); 
    
    // Asserts high when the instruction writes to a register
    assign reg_write_en = &{op != `SW, jump_no_link, ~is_branch_instr};
  
//******************************************************************************
// Memory control
//******************************************************************************
    assign mem_write_en = op == `SW;    // write to memory
    assign mem_read_en  = op == `LW;    // read from memory

endmodule
