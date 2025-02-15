module processor(
    input clk, reset,
    output [31:0] pc,
    input [31:0] instruction,
    output write_enable,
    output [31:0] address_to_mem, data_to_mem,
    input [31:0] data_from_mem
);

    // ? 
    wire branch_blt, branch_beq, branch_jal, branch_jalr, reg_write, mem_to_reg, mem_write, alu_src;
    wire [2:0] alu_control;
    wire [2:0] imm_control;



    wire [31:0] PCPLus4, ALUOut;
    wire BranchJalx = branch_jal | branch_jalr;

    //    PC
    wire [31:0] BranchTarget;
    // ALU
    wire lt, Zero;
    wire MuxPCSelect = (branch_blt & lt) | (branch_beq & Zero) | BranchJalx;
    // ? ?  ? ?
    wire [31:0] WD3;
    wire [31:0] PCNext;
    // ? 
    wire [31:0] SrcA, rd2;
    // ?  
    wire [31:0] ImmOp;
    // ? ?  ? ALU
    wire [31:0] SrcB;

    // ? Control Unit
    control_unit ctrl(
        instruction, 
        branch_blt, branch_beq, branch_jal, branch_jalr, 
        reg_write, mem_to_reg, mem_write, alu_src, 
        alu_control, imm_control
    );
    
    ALU AlUModule(alu_control, SrcA, SrcB, lt, Zero, ALUOut);

    registers Regs(
        instruction[19:15], instruction[24:20], instruction[11:7],
        WD3, reg_write, reset, clk, SrcA, rd2
    );

    ImmDecoder immdec(instruction[31:7], imm_control, ImmOp);

    //  PC
    PC MyPC(PCNext, pc, clk, reset); 

    Plus4 alu4(pc, PCPLus4);


    assign SrcB = (alu_src) ? ImmOp : rd2;
        //  
    assign address_to_mem = ALUOut;
    assign data_to_mem = rd2;
    assign write_enable = mem_write;
    assign WD3 = (mem_to_reg) ? data_from_mem 
                            : (BranchJalx ? PCPLus4 : ALUOut);
    assign BranchTarget = (branch_jalr) ? ALUOut : (pc + ImmOp);
    assign PCNext = (MuxPCSelect) ? BranchTarget : reset ? 0 : PCPLus4;

endmodule


module Plus4(input [31:0] pcin, output [31:0] out);
    assign out = pcin + 4;
endmodule    


module control_unit(
        input [31:0] inst,
        output reg branch_blt, branch_beq, branch_jal, branch_jalr,
        output reg reg_write, mem_to_reg, mem_write, alu_src,
        output reg [2:0] alu_control,
        output reg [2:0] imm_control
    );
    // ?  ? opcodes
    localparam OPCODE_RTYPE   = 7'b0110011;
    localparam OPCODE_ITYPE   = 7'b0010011;
    localparam OPCODE_LOAD    = 7'b0000011;
    localparam OPCODE_STORE   = 7'b0100011;
    localparam OPCODE_BRANCH  = 7'b1100011;
    localparam OPCODE_LUI     = 7'b0110111;
    localparam OPCODE_JAL     = 7'b1101111;
    localparam OPCODE_JALR    = 7'b1100111;
    localparam OPCODE_EXP     = 7'b0001011; //  

    // ?  ? alu_control
    localparam ALU_AND       = 3'b000;
    localparam ALU_SRL       = 3'b001;
    localparam ALU_ADD       = 3'b010;
    localparam ALU_SUB       = 3'b011;
    localparam ALU_EXPONENT  = 3'b100;
    localparam ALU_LUI       = 3'b101;
    localparam ALU_ETD       = 3'b111;

    always @(*) begin
        // ?  
        branch_blt   = 0;
        branch_beq   = 0;
        branch_jal   = 0;
        branch_jalr  = 0;
        reg_write    = 0;
        mem_to_reg   = 0;
        mem_write    = 0;
        alu_src      = 0;
        alu_control  = ALU_ADD; //  ?
        imm_control  = 3'b000;  //  ?

        case (inst[6:0])
            OPCODE_RTYPE: begin // R-type 
                reg_write = 1;
                case ({inst[31:25], inst[14:12]})
                    {7'b0000000, 3'b000}: alu_control = ALU_ADD; // add
                    {7'b0100000, 3'b000}: alu_control = ALU_SUB; // sub
                    {7'b0000000, 3'b111}: alu_control = ALU_AND; // and
                    {7'b0000000, 3'b101}: alu_control = ALU_SRL; // srl
                    {7'b0000000, 3'b110}: alu_control = ALU_ETD; // etd

                endcase
            end
            OPCODE_ITYPE: begin // I-type  
                reg_write   = 1;
                alu_src     = 1;
                imm_control = 3'b001; // I-type immediate
                case (inst[14:12])
                    3'b000: alu_control = ALU_ADD; // addi
                endcase
            end
            OPCODE_LOAD: begin // Load 
                if (inst[14:12] == 3'b010) begin // lw
                    reg_write   = 1;
                    mem_to_reg  = 1;
                    alu_src     = 1;
                    imm_control = 3'b001; // I-type immediate
                end
            end
            OPCODE_STORE: begin // Store 
                if (inst[14:12] == 3'b010) begin // sw
                    mem_write   = 1;
                    alu_src     = 1;
                    imm_control = 3'b010; // S-type immediate
                end
            end
            OPCODE_BRANCH: begin // Branch 
                imm_control = 3'b011; // B-type immediate
                alu_control = ALU_SUB; // ? ?
                case (inst[14:12])
                    3'b000: branch_beq = 1; // beq
                    3'b100: branch_blt = 1; // blt
                    default: ;
                endcase
            end
            
            OPCODE_LUI: begin // LUI
                reg_write   = 1;
                alu_src     = 1;
                imm_control = 3'b100; // U-type immediate
                alu_control = ALU_LUI; //  ALU ?  
            end
            OPCODE_JAL: begin // JAL
                branch_jal  = 1;
                reg_write   = 1;
                imm_control = 3'b101; // J-type immediate
            end
            OPCODE_JALR: begin // JALR
                branch_jalr = 1;
                reg_write   = 1;
                alu_src     = 1;
                imm_control = 3'b001; // I-type immediate
            end
            OPCODE_EXP: begin //   - ? 
                if (inst[14:12] == 3'b000 && inst[31:25] == 7'b0000000) begin
                    reg_write   = 1;
                    alu_control = ALU_EXPONENT; // ? ?   
                end
            end

        endcase
    end
endmodule


module registers(
        input [4:0] a1, a2, a3,
        input [31:0] wd3,
        input we, reset, clk,
        output reg [31:0] rd1, rd2
    );

    integer i;
    reg [31:0] data [31:0];

    //  ? (? )
    always @(*) begin
        rd1 = data[a1];
        rd2 = data[a2];
    end

    //  ? ? ?
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for(i = 0; i < 32; i = i + 1)
                data[i] <= 0;
        end else begin
            if (we == 1 && a3 != 0) begin
                data[a3] <= wd3;
            end
        end
    end
endmodule


module ALU (
        input [2:0] ALUControl, // 3-  
        input [31:0] SrcA, SrcB,
        output reg lt, Zero,
        output reg [31:0] ALUOut
    );
    integer i;
    always @(*) begin
        Zero = 0;
        lt = 0;
        case (ALUControl)
            3'b000: ALUOut = SrcA & SrcB;            // AND
            3'b001: ALUOut = SrcA >> SrcB;           // SRL
            3'b010: ALUOut = SrcA + SrcB;            // ADD
            3'b011: ALUOut = SrcA - SrcB;            // SUB beq blt
            3'b100: ALUOut = $signed (SrcA[30:23]) - 127;
            default: ALUOut = SrcB; // LUI
        endcase

        Zero = (ALUOut == 0);
        lt = ($signed(SrcA) < $signed(SrcB));
    end
endmodule


module ImmDecoder(
        input [31:7] inst,
        input [2:0] imm_control,
        output reg [31:0] ImmOp
    );
    always @(*) begin
        ImmOp = 0;
        case (imm_control)
            //
            3'b000: ImmOp = 0; // 

            3'b001: begin // I-type (addi, lw, jalr)
                ImmOp[11:0] = inst[31:20];
                ImmOp[31:12] = {20{inst[31]}};
            end

            3'b010: begin // S-type (sw)
                // Imm[11:0] = {inst[31:25], inst[11:7]}
                ImmOp[11:5] = inst[31:25];
                ImmOp[4:0] = inst[11:7];
                ImmOp[31:12] = {20{inst[31]}};
            end

            3'b011: begin // B-type (beq, blt)
                ImmOp[31:13] = {19{inst[31]}};
                ImmOp[12]    = inst[31];
                ImmOp[11]    = inst[7];
                ImmOp[10:5]  = inst[30:25];
                ImmOp[4:1]   = inst[11:8];
                ImmOp[0]     = 0; 
            end

            3'b100: begin // U-type (lui)
                ImmOp[31:12] = inst[31:12];
                ImmOp[11:0] = 12'b0;
            end

            3'b101: begin // J-type (jal)
                ImmOp[31:21] = {11{inst[31]}};
                ImmOp[20]    = inst[31];
                ImmOp[19:12] = inst[19:12];
                ImmOp[11]    = inst[20];
                ImmOp[10:1]  = inst[30:21];
                ImmOp[0]     = 0; 
            end
            default: ImmOp = 0;
        endcase
    end
endmodule

module PC(input [31:0] PCnext, output [31:0] out, input clk, reset );
    reg [31:0] data;
    assign out = data;
    always @(posedge clk or posedge reset) begin
        if (reset)
            data <= 0;
        else
            data <= PCnext;
    end
endmodule