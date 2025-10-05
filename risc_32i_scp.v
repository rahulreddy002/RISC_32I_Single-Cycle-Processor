// Code your design here



//==================================PC========================================
module program_counter (clk,rst,pc_in,pc_out);
  input clk,rst;
  input [31:0]pc_in;
  output reg [31:0]pc_out;
  
  
  always@(posedge clk or posedge rst)
    begin
      if(rst)
          pc_out <= 32'b00;
          else
            pc_out <= pc_in;
        end
endmodule

//============================================================================




//=================================ADDER+4====================================

module addplus4 (pc_to_adder,adder_out);
  input [31:0] pc_to_adder;
  output reg [31:0] adder_out;
  
  always @(*) begin
  adder_out <= pc_to_adder + 4;
  end
endmodule

//============================================================================

//=================================IM=========================================

module instruction_memory (clk,rst,read_address, instruction_out);
  input clk,rst;
  input [31:0] read_address;
  output [31:0]instruction_out;
  integer i;
  reg [31:0] I_M [63:0];
  
  assign instruction_out =I_M[read_address];
  
  always@(posedge clk or posedge rst )
    begin
      if(rst)
        begin
          for(i=0; i<63; i=i+1)
            begin
            I_M[i]<= 32'b00;
            end
        end
      else
        begin
       // R type
        I_M[0]=32'h00000000;
		  I_M[4]=32'b0000000_11001_10000_000_01101_0110011;  // add x13, x16, x25
		  I_M[8]=32'b0100000_00011_01000_000_00101_0110011;  // sub x5, x8, x3
		  I_M[12]=32'b0000000_00011_00010_111_00001_0110011; // and x1, x2, x3
		  I_M[16]=32'b0000000_00101_00011_110_00100_0110011; // or x4, x3, x5
		  
		  // I Type
		  I_M[20]=32'b000000000011_10101_000_10110_0010011; // addi x22, x21, 3
		  I_M[24]=32'b000000000001_01000_110_01001_0010011; // ori x9, x8, 1
		  
		  // L type
		  I_M[28]=32'b000000001111_00101_010_01000_0000011; // lw x8, 15(x5)
		  I_M[32]=32'b000000000011_00011_010_01001_0000011; // lw x9, 3(x3) 
		  
		  // S type
		  I_M[36]=32'b0000000_01111_00101_010_01100_0100011; // sw x15, 12(x5)
		  I_M[40]=32'b0000000_01110_00110_010_01010_0100011; // sw x14, 10(x6)
		  
		  // SB type
		  I_M[44]=32'h00738663; // beq x7,x7,12

        
        end
    end
endmodule

//============================================================================



//==============================REGISTER======================================

module Register (clk,rst,Rs1,Rs2,RegWrite,rd,write_data,Read_data1,Read_data2);
  input clk,rst,RegWrite;
  input [4:0]Rs1,Rs2,rd;
  input [31:0] write_data;
  output [31:0]Read_data1,Read_data2;
  
  integer i;
  reg [31:0]Register[31:0];
  
  initial begin
Register[0] = 0;
Register[1] = 4;
Register[2] = 2;
Register[3] = 24;
Register[4] = 4;
Register[5] = 1;
Register[6] = 44;
Register[7] = 4;
Register[8] = 2;
Register[9] = 1;
Register[10] = 23;
Register[11] = 4;
Register[12] = 90;
Register[13] = 10;
Register[14] = 20;
Register[15] = 30;
Register[16] = 40;
Register[17] = 50;
Register[18] = 60;
Register[19] = 70;
Register[20] = 80;
Register[21] = 80;
Register[22] = 90;
Register[23] = 70;
Register[24] = 60;
Register[25] = 65;
Register[26] = 4;
Register[27] = 32;
Register[28] = 12;
Register[29] = 34;
Register[30] = 5;
Register[31] = 10;
  end
  
  always@(posedge clk or posedge rst)
    begin
      if (rst)
        begin
          for (i=0;i<32;i=i+1)
            begin
              Register[i]<=32'b00;
            end
        end
      else if (RegWrite) begin
        Register[rd]<= write_data;
      end
    end
  
  assign Read_data1 = Register[Rs1];
  assign Read_data2 = Register[Rs2];
        
  
  
endmodule

//=====================================Control_unit==========================================

module control_unit (opcode,branch,MemRead,MemtoReg,MemWrite,ALUSrc,RegWrite,Aluop);
  input [6:0]opcode;
  output reg branch,MemRead,MemtoReg,MemWrite,ALUSrc,RegWrite;
  output reg [1:0]Aluop;
  
  always@(*)
    begin
      case(opcode)
        //I-type
        7'b0010011 : {ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,branch,Aluop}<= 8'b10100010;
        //Load
        7'b0000011 : {ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,branch,Aluop}<= 8'b11110000;
        //Store
        7'b0100011 : {ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,branch,Aluop}<= 8'b10001000;
        //BEQ
        7'b1100011 : {ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,branch,Aluop}<= 8'b00000111;
        
        default : {ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,branch,Aluop}<= 8'b00000000;
      endcase
    end
endmodule

//===========================================================================================


//============================================IMM_GEN==============================================

module IMM_GEN (opcode,instructions,imm_out);
  input [31:0]instructions;
  input [6:0]opcode;
  output reg [31:0] imm_out;
  
  always@(*)
    begin
      case (opcode[6:0])
        7'b0000011 :imm_out = {{20{instructions[31]}},instructions[31:20]}; //Load-type
        7'b0100011 : imm_out ={{20{instructions[31]}},instructions[31:25],instructions[11:7]}; //store
        7'b1100011 : imm_out ={{20{instructions[31]}},instructions[31],instructions[30:25],instructions[11:8],1'b0}; //Branch
        default :imm_out = 32'b00;
      endcase
    end
endmodule


//================================================================================================



//================================ALU_CONTROL============================================

module ALU_control (fun7,fun3,aluop,alu_control_out);
  input fun7;
  input [2:0]fun3;
  input [1:0]aluop;
  output reg[3:0]alu_control_out;
  
  
  always@(*)
    begin
      case({aluop,fun7,fun3})
        6'b00_0_000 : alu_control_out <= 0010;
        6'b01_0_000 : alu_control_out <= 0110;
        6'b10_0_000 : alu_control_out <= 0010;
        6'b10_1_000 : alu_control_out <= 0110;
        6'b10_0_111 : alu_control_out <= 0000;
        6'b10_0_110 : alu_control_out <= 0001;
      endcase
    end
endmodule

//=======================================================================================


//=======================================ALU=============================================

module ALU (ain,bin,alu_control,alu_result,zero);
  input [31:0]ain,bin;
  input [3:0]alu_control;
  output reg[31:0]alu_result;
  output reg zero;
  
  always@(ain or bin or alu_control)
    begin
      case(alu_control)
        4'b0000 : begin zero <=0; alu_result <= ain & bin;end 
        4'b0001 : begin zero <=0; alu_result <= ain | bin;end 
        4'b0010 : begin zero <=0; alu_result <= ain + bin;end 
        4'b0011 : begin if (ain==bin) zero <=1; else zero<=0; alu_result <= ain - bin;end 
        default : alu_result <= 32'b00;
      endcase
    end
endmodule

//=======================================================================================


//===================================DATA_MEM=========================================
module data_mem (clk,rst,address, write_data,MemWrite,MemRead,read_data);
  input clk,rst,MemWrite,MemRead;
  input [31:0]address, write_data;
  output reg [31:0]read_data;
  
  integer i;
  reg [31:0] Data_Mem [63:0];
  
  initial begin
   // Data_Mem[17]=56;
    //Data_Mem[15]=65;
  end
  
  always@(posedge clk or posedge rst)
    begin
      if (rst)
        begin
          for (i=0;i<64;i=i+1)
          Data_Mem[i]<=32'b00;
        end
      else if(MemWrite)
        begin
          Data_Mem[address]<= write_data;
        end
    end
  assign read_data = (MemRead)? Data_Mem[address]: 32'b00;      
      
endmodule
//====================================================================================

//====================================MUX1========================================
module ALU_Mux1 (A1,B1,sel1,out1);
  input [31:0]A1,B1;
  input sel1;
  output [31:0]out1;
  
  assign out1 = (sel1==0)? A1: B1;
  
endmodule

//================================================================================


//====================================MUX2========================================
module ADDER_Mux2 (A2,B2,sel2,out2);
  input [31:0]A2,B2;
  input sel2;
  output[31:0] out2;
  
  assign out2 = (sel2==0)? A2: B2;
  
endmodule

//================================================================================

//====================================MUX3========================================
module DATA_MEM_Mux3 (A3,B3,sel3,out3);
  input [31:0]A3,B3;
  input sel3;
  output[31:0]out3;
  
  assign out3 = (sel3==0)? A3: B3;
  
endmodule

//================================================================================

//====================================ADDER1========================================
module Adder (Ain,Bin,adder_out);
  input [31:0]Ain,Bin;
  output [31:0]adder_out;
  
  assign adder_out= Ain + Bin;
  
endmodule

//================================================================================

module andd (a,b,out);
  
  input a,b;
  output out;
  
  assign out = a & b;
  
endmodule


//=================================TOP=========================================================================================================================================================

module risc_top (clk,rst);
  input clk,rst;
  wire [31:0] pc_in_wire,pc_out_wire, adder_out_wire,Adder_out_wire,instruction_out_wire, write_data_wire,Read_data1_wire,Read_data2_wire,imm_out_wire,out1_wire,alu_result_out,r_data_wire_;
  
  wire RegWrite_wire,branch_wire,zero_wire,MemRead_wire,MemtoReg_top,MemWrite_wire,ALUSrc_wire,out_wire;
  wire [1:0]Aluop_wire;
  wire [3:0] alu_control_out_wire;

  program_counter PC (.clk(clk),.rst(rst),.pc_in(pc_in_wire),.pc_out(pc_out_wire));

  addplus4  AP4 (.pc_to_adder(pc_out_wire),.adder_out(Adder_out_wire));

  instruction_memory  IM (.clk(clk),.rst(rst),.read_address(pc_out_wire),.instruction_out(instruction_out_wire));

  Register RG (.clk(clk),.rst(rst),.Rs1(instruction_out_wire[19:15]),.Rs2(instruction_out_wire[24:20]),.RegWrite(RegWrite_wire),.rd(instruction_out_wire[11:7]),
               .write_data(write_data_wire),.Read_data1(Read_data1_wire),.Read_data2(Read_data2_wire));

   control_unit  CU (.opcode(instruction_out_wire[6:0]),.branch(branch_wire),.MemRead(MemRead_wire),.MemtoReg(MemtoReg_top),.MemWrite(MemWrite_wire),
                    .ALUSrc(ALUSrc_wire),.RegWrite(RegWrite_wire),.Aluop(Aluop_wire));

  IMM_GEN IG (.opcode(instruction_out_wire[6:0]),.instructions(instruction_out_wire),.imm_out(imm_out_wire));

  ALU_control  ALU_CTRL(.fun7(instruction_out_wire[30]),.fun3(instruction_out_wire[14:12]),.aluop(Aluop_wire),.alu_control_out(alu_control_out_wire));

  ALU  ALU_U (.ain(Read_data1_wire),.bin(out1_wire),.alu_control(alu_control_out_wire),.alu_result(alu_result_out),.zero(zero_wire));

  
  data_mem DM (.clk(clk),.rst(rst),.address(alu_result_out),.write_data(Read_data2_wire),.MemWrite(MemWrite_wire),.MemRead(MemRead_wire),.read_data(r_data_wire_));
  
  
  ALU_Mux1  M1 (.A1(Read_data2_wire),.B1(imm_out_wire),.sel1(ALUSrc_wire),.out1(out1_wire));

  ADDER_Mux2  M2 (.A2(Adder_out_wire),.B2(adder_out_wire),.sel2(out_wire),.out2(pc_in_wire));

  DATA_MEM_Mux3 M3(.A3(r_data_wire_),.B3(alu_result_out),.sel3(MemtoReg_top),.out3(write_data_wire));
  
  Adder ADDER (.Ain(pc_out_wire),.Bin(imm_out_wire),.adder_out(adder_out_wire));
  
  andd AND (.a(branch_wire),.b(zero_wire),.out(out_wire));

endmodule
