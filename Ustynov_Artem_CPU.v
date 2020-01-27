module ALU (input [31:0] SrcA, SrcB, input[3:0] ALUControl, output zero, output reg [31:0] ALUResult);
      
    function [7:0] satsum;
    input [7:0] a, b;
    reg [8:0] tmp;
    begin
      tmp = a + b;
      satsum = (tmp [8] == 1) ? 8'b11111111 : tmp [7:0];
    end
  endfunction

    assign zero = (ALUResult==0);
    // always @(ALUControl) $display( "ALU, %b",ALUControl,SrcA , SrcB);
    always@(*)
	case (ALUControl)
    // 4'b 0000:  ALUResult = SrcA + 0;
	4'b 0010:  ALUResult = SrcA + SrcB;
	4'b 0110:  ALUResult = SrcA - SrcB; 
	4'b 0000:  ALUResult = SrcA & SrcB ; 
	4'b 0001:  ALUResult = SrcA | SrcB; 
    4'b 0011:  ALUResult = SrcA ^ SrcB;
    4'b 0111:  ALUResult = SrcA + 8'h ffffffff < SrcB + 8'h ffffffff;
    4'b 1000: begin
                    ALUResult[7:0] = SrcA[7:0] + SrcB[7:0];
                    ALUResult[15:8] = SrcA[15:8] + SrcB[15:8];
                    ALUResult[23:16] = SrcA[23:16] + SrcB[23:16];
                    ALUResult[31:24] = SrcA[31:24] + SrcB[31:24];
                end
    4'b 1001: begin
             ALUResult[7:0] = satsum(SrcA[7:0] , SrcB[7:0]);
             ALUResult[15:8] = satsum(SrcA[15:8] , SrcB[15:8]);
             ALUResult[23:16] = satsum(SrcA[23:16] , SrcB[23:16]);
             ALUResult[31:24] = satsum(SrcA[31:24] , SrcB[31:24]);
         end

    default: ALUResult = -1;
	endcase
endmodule
// -------------------------------------------------------------------------------------------------

module RegisterFile (input [4:0] A1, output [31:0] RD1,
                     input [4:0] A2, output [31:0] RD2,
                     input [4:0] A3, input [31:0] WD3,
                     input clk, WE3);

    reg [31:0] rf [31:0];

    assign RD1 = A1? rf[A1]:0;
    assign RD2 = A2? rf[A2]:0;

always@(posedge clk)
    if(WE3) rf[A3] = WD3;

endmodule

module mult_2_1 (input [31:0] d0 , d1, input select , output reg [31:0] out );	
     always@(*)	
	if (select) out = d1 ;
	else out = d0 ;
endmodule

module mult_4_1 (input [31:0] d0 , d1, d2, d3, input [1:0] select , output reg [31:0] out );	
     always@(*)
	case (select)
	0: out = d0;
	1: out = d1; 
	2: out = d2; 
	3: out = d3; 
    default out = -1;
	endcase
endmodule

module bit_adder_32(input [31:0] a,b, input carry_in, output [31:0] sum, output carry_out);
    assign {carry_out,sum} = a + b + carry_in;
endmodule  

module comp_32 (input [31:0] a, b, output out);
    assign out = (a==b);
endmodule

module multiply_by_4 (input [31:0] a, output[31:0] out) ;
    assign out = a << 2; 
endmodule

module extend_16_to_32 (input [15:0] in,output [31:0] out ) ;
    assign out = {{ 16{in[15]}}, in }; 
endmodule

module register_32 ( input [31:0] a, input clk, output reg [31:0] out) ;
always@(posedge clk)
     out = a;
endmodule

module resetable_reg_32 ( input [31:0] a, input clk, reset, output reg [31:0] out) ;
always@(posedge clk)
     out = reset? 0 : a ;
endmodule

module control_unit(input [5:0] Opcode, Funct, input [4:0] Shamt , 
                    output pcSrcJal, PCSrcJr, regWrite, memToReg, memWrite, aluSrc, RegDst, branch,
                    output [3:0] ALUControl);
    wire [1:0] ALUop;
    main_decoder main_decoder (Opcode, ALUop, pcSrcJal, PCSrcJr, regWrite, memToReg, memWrite, aluSrc, RegDst, branch ); 
    alu_op_decoder alu_op_decoder (Funct, Shamt , ALUop, ALUControl) ; 

endmodule   

module main_decoder (input [5:0] Opcode, output reg [1:0] ALUOp,
                     output reg PCSrcJal, PCSrcJr, RegWrite, MemToReg, MemWrite, ALUSrc, RegDst, Branch ); 
    always @(*)
    case(Opcode)
        6'b 000000: begin // R TYP
            RegWrite=1;RegDst=1; ALUSrc=0; ALUOp = 2'b 10 ;
            Branch =0;MemWrite=0;MemToReg=0; PCSrcJal=0; PCSrcJr=0;  
            end
        6'b 100011: begin // lw
            RegWrite=1;RegDst=0; ALUSrc=1; ALUOp = 2'b 00 ;
            Branch =0;MemWrite=0;MemToReg=1; PCSrcJal=0; PCSrcJr=0;  
            end
        6'b 101011: begin //sw
            RegWrite=0; ALUSrc=1; ALUOp = 2'b 00 ;
            Branch =0; MemWrite=1; PCSrcJal=0; PCSrcJr=0;  
            end
        6'b 000100: begin //beq
            RegWrite=0; ALUSrc=0; ALUOp = 2'b 01 ;
            Branch =1; MemWrite=0; PCSrcJal=0; PCSrcJr=0;  
            end
        6'b 001000: begin // addi
            RegWrite=1;RegDst=0; ALUSrc=1; ALUOp = 2'b 00 ;
            Branch =0;MemWrite=0;MemToReg=0; PCSrcJal=0; PCSrcJr=0;  
            end
        6'b 000011: begin // jal
            RegWrite=1;MemWrite=0; PCSrcJal=1; PCSrcJr=0;  
            end
        6'b 000111: begin // jr
            RegWrite=0;MemWrite=0; PCSrcJal=0; PCSrcJr=1;  
            end
        6'b 011111: begin // addi
            RegWrite=1; RegDst=1; ALUSrc=0; ALUOp = 2'b 11 ;
            Branch =0;MemWrite=0;MemToReg=0; PCSrcJal=0; PCSrcJr=0;  
            end
    endcase
endmodule

module alu_op_decoder (input [5:0] Funct, input [4:0] Shamt, input [1:0] ALUOp, output reg [3:0] ALUControl);
    always @ (*)
        case(ALUOp)
            2'b 00: ALUControl = 4'b 0010;
            2'b 01: ALUControl = 4'b 0110;
            2'b 10: begin 
                case (Funct)
                    6'b 100000: ALUControl = 4'b 0010;
                    6'b 100010: ALUControl = 4'b 0110;
                    6'b 100100: ALUControl = 4'b 0000;
                    6'b 100101: ALUControl = 4'b 0001;
                    6'b 101010: ALUControl = 4'b 0111;
                endcase
            end
            2'b 11: begin 
                case (Funct)
                    6'b 010000: 
                        case(Shamt)
                            5'b 00000: ALUControl = 4'b 1000;
                            5'b 00100: ALUControl = 4'b 1001;
                        endcase
                endcase
            end
        endcase
    
    endmodule

module processor( input         clk, reset,
                  output [31:0] PC,
                  input  [31:0] instruction,
                  output        WE,
                  output [31:0] address_to_mem,
                  output [31:0] data_to_mem,
                  input  [31:0] data_from_mem
                );
    // add_4 next_inst(PC,clk,reset, PC);
    // ins 
    wire [31:0] SrcA, SrcB, Signlmm;
    wire [3:0 ] ALUControl;
    wire [3:0 ] wire_jal1;
    wire [25:0 ] wire_jal2;
    // outs
    wire [31:0]  ALUOut, WD3, ReadData;
    wire PCSrcJal,PCSrcJr, RegWrite, MemToReg,WE, ALUSrc,RegDst,Branch, PCSrcBeq ;
    wire [5:0] Opcode, Funct;
    wire [4:0] Shamt ;
    wire [4:0] A1, A2, A3;
    wire [4:0] WriteReg ;
    wire [4:0] Rt,Rd ;
    wire [31:0] WriteData;
    wire Zero;
    wire [31:0] wire_mult_add;
    wire [31:0] wire_pccont_cunt, wire_cunt_a;
    wire [31:0] PCPlus4, Result, PCBranch;
    wire [31:0] PCJal;
    // const 
    wire [4:0] thirty_one;
    assign thirty_one = 31;
    wire [31:0] four;
    assign four = 4;
    //else
    assign Opcode = instruction[31:26];
    assign Funct = instruction[5:0];
    assign Shamt = instruction[10:6];
    assign A1 = instruction[25:21];
    assign A2  = instruction[20:16];
    // assign A3 =  instruction[20:16];
    assign Rt = instruction[20:16];
    assign Rd = instruction[15:11];
    assign wire_jal1 = PCPlus4 [31:28];
    assign wire_jal2 = instruction [25:0];

    assign PCSrcBeq = Branch & Zero;
    // assign data_from_mem = ReadData ;
    // assign data_to_mem=WriteData  ;
    // assign address_to_mem = ALUOut ;
    // assign PC = wire_cunt_a ;

    PCControl pccont(PCSrcBeq,PCSrcJr, PCSrcJal, 
                     SrcA, PCJal, PCBranch, PCPlus4, 
                     wire_pccont_cunt); //out

    resetable_reg_32 prog_cunt( wire_pccont_cunt, clk, reset, PC);

    control_unit control_unit(Opcode, Funct, Shamt , 
                    PCSrcJal, PCSrcJr, RegWrite, MemToReg, WE, ALUSrc, RegDst, Branch,
                    ALUControl);    
    always @(WE)  $display( "WE %d ",WE); 
    mult_2_1_5b_5o mul1( WriteReg , thirty_one,  PCSrcJal , A3 );	
    mult_2_1       mul2( Result, PCPlus4, PCSrcJal, WD3 );	
    mult_2_1_5b_5o mul3( Rt, Rd, RegDst, WriteReg );
    mult_2_1       mul4( data_to_mem, Signlmm, ALUSrc, SrcB );	
    mult_2_1       mul5( address_to_mem, data_from_mem, MemToReg, Result );	
    
    multiply_by_4  times_4( Signlmm, wire_mult_add) ;
    summer sum1 ( PC, four,   PCPlus4) ;
    summer sum2 ( wire_mult_add, PCPlus4,  PCBranch);
    jal jaller (wire_jal1, wire_jal2, PCJal);

    RegisterFile register ( A1,   SrcA,       // in - out
                            A2,   data_to_mem,  // in - out
                            A3,   WD3,       // intputs
                            clk,  RegWrite); // outs

    extend_16_to_32 extender (instruction[15:0], Signlmm ) ;
    ALU ALU ( SrcA, SrcB, ALUControl, 
              Zero,address_to_mem); //outs
 
// always @(instruction) $display( "CPU PC %b, inst %b %b %b",PC[7:2],instruction,instruction[25:21],address_to_mem);
endmodule

module summer ( input [31:0] a, b, output reg [31:0] out) ;
     always @(*)
     out = a + b;
endmodule

module PCControl(input PCSrcBeq,PCSrcJr, PCSrcJal, 
                 input [31:0] JR, JAL, BEQ, ZERO, 
                 output reg [31:0] PC );
    always @(*) 
        if (PCSrcBeq)
        PC = BEQ ;
        else if (PCSrcJr)
        PC = JR ;
        else if (PCSrcJal)
        PC = JAL ;
        else PC = ZERO;
endmodule

module mult_2_1_5b (input [4:0] d0 , d1, input select , output reg [31:0] out );	
     always@(*)	
	if (select) out = d1 ;
	else out = d0 ;
endmodule
module mult_2_1_5b_5o (input [4:0] d0 , d1, input select , output reg [4:0] out );	
    always@(*)	
	if (select) out = d1 ;
	else out = d0 ;
endmodule

module jal (input [3:0] a,input [25:0] b, output[31:0] out);
    assign out = {a,b, 2'b 00 };
endmodule

module tst ( input  [7:0] a , output [2:0] b);
assign b = a [7:5];
endmodule

