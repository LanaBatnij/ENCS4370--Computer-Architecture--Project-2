`timescale 1ns/1ns

module instruction_memory (input [31:0] address, output reg [31:0] instruction);
	reg [31:0] mem [511:0];
	
	always@(*) begin
		instruction = mem[address]; 
	end
	
endmodule		

module instruction_memory_tb;
    reg [31:0] address;
    wire [31:0] instruction;
    
    instruction_memory im(address, instruction);
    
    initial begin
        address = 0;
        #5;
        address = 2;
        #20;
        $finish;
    end
endmodule


module adder(input [31:0] PC, input [31:0] immediate, output reg [31:0]	out);
	always@(*)
		begin
			 out = PC + immediate;
		end
endmodule

module mux2x1_32(input S, input [31:0] D0, input [31:0] D1, output reg [31:0] Y);
  
  always@(*)
	  begin
		if (S == 0)
		  Y = D0;
		else
		  Y = D1;
 	  end
endmodule

module mux2x1_5(input S, input [4:0] D0, input [4:0] D1, output reg [4:0] Y);
  
  always@(*)
	  begin
		if (S == 0)
		  Y = D0;
		else
		  Y = D1;
 	  end
endmodule

module mux3x1_32(input [1:0] S, input [31:0] D0, input [31:0] D1, input [31:0] D2, output reg [31:0] Y);
  
  always@(*) begin
    case (S)
      2'b00: Y = D0; 
      2'b01: Y = D1; 
      2'b10: Y = D2;
    endcase
  end
endmodule	 

module extender14to32(input ExtOpI, input [13:0] input_14bits, output reg [31:0] output_32bits);
	always @(*) begin
		if(ExtOpI == 1'b0) begin
			output_32bits[31:14] = 18'b000000000000000000;
			output_32bits[13:0] = input_14bits;
		end
		else begin
			output_32bits[31:14] = {18{input_14bits[13]}};
			output_32bits[13:0] = input_14bits;
		end
	end
endmodule	

module extender24to32(input ExtOpJ, input [23:0] input_24bits, output reg [31:0] output_32bits);
	always @(*) begin
		if(ExtOpJ == 1'b0) begin
			output_32bits[31:24] = 8'b00000000;
			output_32bits[23:0] = input_24bits;
		end
		else begin
			output_32bits[31:24] = {8{input_24bits[23]}};
			output_32bits[23:0] = input_24bits;
		end
	end
endmodule 

module extender5to32(input ExtOpSA, input [4:0] input_5bits, output reg [31:0] output_32bits);
	always @(*) begin
		if(ExtOpSA == 1'b0) begin
			output_32bits[31:5] = 27'b0;
			output_32bits[4:0] = input_5bits;
		end
		else begin
			output_32bits[31:5] = {27{input_5bits[4]}};
			output_32bits[4:0] = input_5bits;
		end
	end
endmodule

module register_file(input clk, input regWr, input [4:0] RA, input [4:0] RB, input [4:0] RW,
	input [31:0] BusW, output reg [31:0] BusA, output reg [31:0] BusB);	
	
	reg [31:0] registers[31:0];
	
	always @(posedge clk) 
		begin
			if(regWr && RW != 0) 
				begin
					registers[RW] = BusW;
				end
			BusA = registers[RA];
			BusB = registers[RB];  
		end
endmodule	   

module ALU(input clk, input [2:0] ALUOp, input [31:0] A, input [31:0] B, output reg zero, output reg [31:0] out);
	parameter AND = 3'b000, ADD = 3'b001, SUB = 3'b010, SLL = 3'b011, SLR = 3'b100;
	assign zero = 0;
	
	always@(posedge clk)
		begin
			case (ALUOp)
				AND: out = A & B;
				ADD: out = A + B;
				SUB: begin
					out = A - B;
					if(out == 0)
						begin
							assign zero = 1;
						end
					end
				SLL: out = A << B;
				SLR: out = A >> B;
			endcase		
		end	
endmodule

module Stack (input EnStack, input push, input pop, input [31:0] data_in, output reg [31:0] data_out, output reg is_empty, output reg is_full);
	
  parameter STACK_SIZE = 16; // Define the size of the stack

  reg [31:0] stack[STACK_SIZE-1:0];
  reg [3:0] top;

  always@(*) 
	  begin
		if (push && EnStack && !is_full) 
			begin
			  // Push operation
			  stack[top] <= data_in; // Store the data_in value in the stack
			  top <= top + 1; // Increment top
			  is_empty <= 1'b0; // Stack is not empty
			  if (top == STACK_SIZE - 1) // Check if stack is full
			  	is_full <= 1'b1; // Stack is full
			end
		else if (pop && EnStack && !is_empty) 
			begin
			  // Pop operation
			  top <= top - 1; // Decrement top
			  data_out <= stack[top]; // Read the data from the stack
			  is_full <= 1'b0; // Stack is not full
			  if (top == 0) // Check if stack is empty
			  	is_empty <= 1'b1; // Stack is empty
			end
  	end
endmodule  

module data_memory (input clk, input [31:0] address, input [31:0] data_in, input MemRd, input MemWr, output reg [31:0] data_out);
	reg [31:0] mem[511:0];
	//assign mem = '{512{32'd0}};
	
	always @(posedge clk) begin	
		if(MemRd == 1) begin
			data_out = mem[address];
		end							
		else if(MemWr == 1) begin
			mem[address] = data_in;
		end
	end
endmodule

module data_memory_testbench();
	reg clk, mem_rd, mem_wr; 
	reg [31:0] address, data_in, data_out; 
	
 	data_memory dm(clk, address, data_in, mem_rd, mem_wr, data_out);	  
	
	initial begin
		clk = 0; mem_rd = 0; mem_wr = 0;
		address = 0; data_in = 0;
		
		#5 
		clk = ~clk;
		mem_wr = 1;
		mem_rd = 0;
		data_in = 12;
		address = 2;
		
		#20 
		clk = ~clk;
		
		#5 
		clk = ~clk;
		mem_wr = 0;
		mem_rd = 1;
		address = 2;
		
		#20
		
		$finish;
	end	 
endmodule

module split_instruction(input clk, input [31:0] inst, output reg [4:0] Function, output reg [4:0] rs1, output reg [4:0] rd,
	output reg [4:0] rs2, output reg [13:0] immediate, output reg [23:0] immediateJ, output reg [4:0] sa, output reg [1:0] Type,
	output reg stop);			  
	
	always@(posedge clk) 
		begin
			Function = inst[31:27];
			Type = inst[2:1];
			stop = inst[0];
			
			if(Type == 2'b00)
				begin
					rs1 = inst[26:22];
					rd = inst[21:17];
					rs2 = inst[16:12];
				end
			else if(Type == 2'b01)
				immediateJ = inst[26:3];  
			else if(Type == 2'b10)
				begin
					rs1 = inst[26:22];
					rd = inst[21:17];
					immediate = inst[16:3];
				end	 
			else if(Type == 2'b11)
				begin
					rs1 = inst[26:22];
					rd = inst[21:17];
					rs2 = inst[16:12];
					sa = inst[11:7];
				end
		end
endmodule	

module control_unit(input stop, input [4:0] Function, input [1:0] Type, input zero, output reg regRead, output reg regWr,
	output reg ExtOpJ, output reg ExtOpI, output reg ExtOpSA, output reg [1:0] ALUSrc, output reg PCAdderSrc,
	output reg [2:0] ALUOp, output reg MemRd, output reg MemWr, output reg WBData, output reg EnStack, output reg [1:0] PCSrc);
	
	always@(*)
		begin
			case({Type, Function}) 
				7'b0000000: begin					// R-Type
								regRead = 1'b0;
							   	regWr = 1'b1;
								ExtOpJ = 1'bX; //don't care
								ExtOpI = 1'bX; //don't care
								ExtOpSA = 1'bX; //don't care	 
								ALUSrc = 2'b00;
								PCAdderSrc = 1'bX; //don't care
								ALUOp = 3'b000;
								MemRd = 1'b0;
								MemWr = 1'b0;
								WBData = 1'b1;
								EnStack = 1'b0;
								if(stop == 1'b1)
									PCSrc = 2'b10;
								else
									PCSrc = 2'b00;
							end	 
							
				7'b0000001: begin
								regRead = 1'b0;
							   	regWr = 1'b1;
								ExtOpJ = 1'bX; //don't care
								ExtOpI = 1'bX; //don't care
								ExtOpSA = 1'bX; //don't care	 
								ALUSrc = 2'b00;
								PCAdderSrc = 1'bX; //don't care
								ALUOp = 3'b001;
								MemRd = 1'b0;
								MemWr = 1'b0;
								WBData = 1'b1;
								EnStack = 1'b0;
								if(stop == 1'b1)
									PCSrc = 2'b10;
								else
									PCSrc = 2'b00;
							end
							
							
				7'b0000010: begin
								regRead = 1'b0;
							   	regWr = 1'b1;
								ExtOpJ = 1'bX; //don't care
								ExtOpI = 1'bX; //don't care
								ExtOpSA = 1'bX; //don't care	 
								ALUSrc = 2'b00;
								PCAdderSrc = 1'bX; //don't care
								ALUOp = 3'b010;
								MemRd = 1'b0;
								MemWr = 1'b0;
								WBData = 1'b1;
								EnStack = 1'b0;
								if(stop == 1'b1)
									PCSrc = 2'b10;
								else
									PCSrc = 2'b00;
							end
							
							
				7'b0000011: begin
								regRead = 1'b0;
							   	regWr = 1'b0;
								ExtOpJ = 1'bX; //don't care
								ExtOpI = 1'bX; //don't care
								ExtOpSA = 1'bX; //don't care	 
								ALUSrc = 2'b00;
								PCAdderSrc = 1'bX; //don't care
								ALUOp = 3'b010;
								MemRd = 1'b0;
								MemWr = 1'b0;
								WBData = 1'b1;
								EnStack = 1'b0;
								if(stop == 1'b1)
									PCSrc = 2'b10;
								else
									PCSrc = 2'b00;
							end	
							
				7'b1000000: begin					// I-Type
								regRead = 1'bX; //don't care
							   	regWr = 1'b1;
								ExtOpJ = 1'bX; //don't care
								ExtOpI = 1'b0;
								ExtOpSA = 1'bX; //don't care	 
								ALUSrc = 2'b01;
								PCAdderSrc = 1'bX; //don't care
								ALUOp = 3'b000;
								MemRd = 1'b0;
								MemWr = 1'b0;
								WBData = 1'b1;
								EnStack = 1'b0;
								if(stop == 1'b1)
									PCSrc = 2'b10;
								else
									PCSrc = 2'b00;
							end
							
				7'b1000001: begin					
								regRead = 1'bX; //don't care
							   	regWr = 1'b1;
								ExtOpJ = 1'bX; //don't care
								ExtOpI = 1'b1;
								ExtOpSA = 1'bX; //don't care	 
								ALUSrc = 2'b01;
								PCAdderSrc = 1'bX; //don't care
								ALUOp = 3'b001;
								MemRd = 1'b0;
								MemWr = 1'b0;
								WBData = 1'b1;
								EnStack = 1'b0;
								if(stop == 1'b1)
									PCSrc = 2'b10;
								else
									PCSrc = 2'b00;
							end
							
				 7'b1000010: begin					
								regRead = 1'bX; //don't care
							   	regWr = 1'b1;
								ExtOpJ = 1'bX; //don't care
								ExtOpI = 1'b1;
								ExtOpSA = 1'bX; //don't care	 
								ALUSrc = 2'b01;
								PCAdderSrc = 1'bX; //don't care
								ALUOp = 3'b001;
								MemRd = 1'b1;
								MemWr = 1'b0;
								WBData = 1'b0;
								EnStack = 1'b0;
								if(stop == 1'b1)
									PCSrc = 2'b10;
								else
									PCSrc = 2'b00;
							end
							
				7'b1000011: begin					
								regRead = 1'b1;
							   	regWr = 1'b0;
								ExtOpJ = 1'bX; //don't care
								ExtOpI = 1'b1;
								ExtOpSA = 1'bX; //don't care	 
								ALUSrc = 2'b01;
								PCAdderSrc = 1'bX; //don't care
								ALUOp = 3'b001;
								MemRd = 1'b0;
								MemWr = 1'b1;
								WBData = 1'bX; //don't care
								EnStack = 1'b0;
								if(stop == 1'b1)
									PCSrc = 2'b10;
								else
									PCSrc = 2'b00;
							end
							
				7'b1000101: begin					
								regRead = 1'b1;
							   	regWr = 1'b0;
								ExtOpJ = 1'bX; //don't care
								ExtOpI = 1'b1;
								ExtOpSA = 1'bX; //don't care	 
								ALUSrc = 2'b00;
								PCAdderSrc = 1'b1; //if zero then 1 else don't care	 (1 in general)
								ALUOp = 3'b010;
								MemRd = 1'b0;
								MemWr = 1'b0;
								WBData = 1'bX; //don't care
								EnStack = 1'b0;
								if(zero == 1'b1)
									PCSrc = 2'b01;
								else
									begin
										if(stop == 1'b1)
											PCSrc = 2'b10;
										else
											PCSrc = 2'b00;
									end
							end	  
							
				7'b0100000: begin					// J-Type
								regRead = 1'bX; //don't care
							   	regWr = 1'b0;
								ExtOpJ = 1'b1;
								ExtOpI = 1'bX; //don't care
								ExtOpSA = 1'bX; //don't care	 
								ALUSrc = 2'bXX;	//don't care
								PCAdderSrc = 1'b0;
								ALUOp = 3'bXXX;	//don't care
								MemRd = 1'b0;
								MemWr = 1'b0;
								WBData = 1'bX;	//don't care
								EnStack = 1'b0;
								if(stop == 1'b1)
									PCSrc = 2'b10;
								else
									PCSrc = 2'b01;
							end	
							
				 7'b0100001: begin					
								regRead = 1'bX; //don't care
							   	regWr = 1'b0;
								ExtOpJ = 1'b1;
								ExtOpI = 1'bX; //don't care
								ExtOpSA = 1'bX; //don't care	 
								ALUSrc = 2'bXX;	//don't care
								PCAdderSrc = 1'b0;
								ALUOp = 3'bXXX;	//don't care
								MemRd = 1'b0;
								MemWr = 1'b0;
								WBData = 1'bX;	//don't care
								EnStack = 1'b1;
								if(stop == 1'b1)
									PCSrc = 2'b10;
								else
									PCSrc = 2'b01;
							end	
					   
				7'b1100000: begin					// S-Type
								regRead = 1'bX; //don't care
							   	regWr = 1'b1;
								ExtOpJ = 1'bX; //don't care
								ExtOpI = 1'bX; //don't care
								ExtOpSA = 1'b0;  
								ALUSrc = 2'b10;
								PCAdderSrc = 1'bX; //don't care
								ALUOp = 3'b011;	
								MemRd = 1'b0;
								MemWr = 1'b0;
								WBData = 1'b1;
								EnStack = 1'b0;
								if(stop == 1'b1)
									PCSrc = 2'b10;
								else
									PCSrc = 2'b00;
							end
							
				7'b1100001: begin					
								regRead = 1'bX; //don't care
							   	regWr = 1'b1;
								ExtOpJ = 1'bX; //don't care
								ExtOpI = 1'bX; //don't care
								ExtOpSA = 1'b0;  
								ALUSrc = 2'b10;
								PCAdderSrc = 1'bX; //don't care
								ALUOp = 3'b100;	
								MemRd = 1'b0;
								MemWr = 1'b0;
								WBData = 1'b1;
								EnStack = 1'b0;
								if(stop == 1'b1)
									PCSrc = 2'b10;
								else
									PCSrc = 2'b00;
							end	
							
				7'b1100010: begin					
								regRead = 1'b0; 
							   	regWr = 1'b1;
								ExtOpJ = 1'bX; //don't care
								ExtOpI = 1'bX; //don't care
								ExtOpSA = 1'bX; //don't care  
								ALUSrc = 2'b00;
								PCAdderSrc = 1'bX; //don't care
								ALUOp = 3'b011;	
								MemRd = 1'b0;
								MemWr = 1'b0;
								WBData = 1'b1;
								EnStack = 1'b0;
								if(stop == 1'b1)
									PCSrc = 2'b10;
								else
									PCSrc = 2'b00;
							end	
							
				7'b1100011: begin					
								regRead = 1'b0; 
							   	regWr = 1'b1;
								ExtOpJ = 1'bX; //don't care
								ExtOpI = 1'bX; //don't care
								ExtOpSA = 1'bX; //don't care  
								ALUSrc = 2'b00;
								PCAdderSrc = 1'bX; //don't care
								ALUOp = 3'b100;	
								MemRd = 1'b0;
								MemWr = 1'b0;
								WBData = 1'b1;
								EnStack = 1'b0;
								if(stop == 1'b1)
									PCSrc = 2'b10;
								else
									PCSrc = 2'b00;
							end
			endcase
		end
endmodule	


module IFetch(input clk, input [31:0] PC_in, output reg [31:0] inst);  	  	
  	instruction_memory im(PC_in, inst);

endmodule	

module decode(input clk, input PC_in, input [31:0] inst, input zero, input [31:0] BusW, output reg [4:0] Function, output reg [13:0] immediate,
	output reg [23:0] immediateJ, output reg [4:0] sa, output reg [1:0] Type, output reg stop, 
	output reg regWr, output reg ExtOpJ, output reg ExtOpI, output reg ExtOpSA, output reg [1:0] ALUSrc, output reg PCAdderSrc,
	output reg [2:0] ALUOp, output reg MemRd, output reg MemWr, output reg WBData, output reg EnStack, output reg [1:0] PCSrc,
	output reg [31:0] BusA, output reg [31:0] BusB, output reg [4:0] RA, output reg [4:0] RB, output reg [4:0] RW); 
	
	reg [4:0] rs1, rd, rs2;
	reg regRead, is_empty, is_full;
	reg [31:0] imm24to32, pc_in_stack, st_out;
	
	generate
		split_instruction split(clk, inst, Function, rs1, rd, rs2, immediate, immediateJ, sa, Type, stop);
		control_unit cu(stop, Function, Type, zero, regRead, regWr, ExtOpJ, ExtOpI, ExtOpSA, ALUSrc, PCAdderSrc, ALUOp, MemRd, MemWr, WBData, EnStack, PCSrc);
		assign pc_in_stack = PC_in + imm24to32 + 4;
		Stack stack(EnStack, 1'b1, 1'b0, pc_in_stack, st_out, is_empty, is_full);
		assign RA = rs1;
		mux2x1_5 mux (regRead, rs2, rd, RB);
		assign RW = rd;
		register_file rf(clk, regWr, RA, RB, RW, BusW, BusA, BusB);
	endgenerate
	
endmodule

module execute(input clk, input [31:0] PC_in, input [31:0] BusA, input [31:0] BusB , input [13:0] immediate, input [23:0] immediateJ,
	input [4:0] sa,input ExtOpJ, input ExtOpI, input ExtOpSA, input [1:0] ALUSrc, input [2:0] ALUOp, output reg zero,
	output reg [31:0] ALU_out, output reg [31:0] imm24to32, output reg [1:0] PCSrc, output reg PCAdderSrc);
	
	generate
		reg [31:0] imm14to32, imm5to32, A, B;
		assign A = BusA;
		extender14to32 ext14(ExtOpI, immediate, imm14to32);
		extender24to32 ext24(ExtOpJ, immediateJ, imm24to32);
		extender5to32 ext5(ExtOpSA, sa, imm5to32);
		
		mux3x1_32 mux(ALUSrc, BusB, imm14to32, imm5to32, B);
		
		ALU alu(clk, ALUOp, A, B, zero, ALU_out);
		
		always@(posedge clk)
			begin 
				if(zero == 1'b1)
					begin
						PCSrc = 2'b01;
						PCAdderSrc = 1'b1;
					end
				else
					begin
						PCSrc = 2'b00;
						PCAdderSrc = 1'bX;
					end
			end
	endgenerate
endmodule

module memory_stage(input clk, input [31:0] address, input [31:0] data_in, input MemRd, input MemWr, output reg [31:0] data);
	data_memory dm(clk, address, data_in, MemRd, MemWr, data);
endmodule 

module WB_stage(input clk, input WBData, input [4:0] RW, input [4:0] RA, input[4:0] RB,
	input [31:0] imm14to32, input [31:0] imm24to32, input [31:0] ALU_out, input [31:0] mem_data, input [1:0] PCSrc, input PCAdderSrc,
	input [31:0] PC_in, input EnStack,output reg [31:0] Pc_out);
	
	reg [31:0] BusA, BusB, BusW, imm, add, st;
	reg is_full, is_empty;
	
	mux2x1_32 mux(WBData, ALU_out, mem_data, BusW);
	register_file rf(clk, RA, RB, RW, BusW, BusA, BusB);
	
	mux2x1_32 mux2(PCAdderSrc, imm24to32, imm14to32, imm);
	adder a(PC_in, imm, add);
	Stack stack(EnStack, 1'b0, 1'b1, 0, st, is_empty, is_full);
	mux3x1_32 mux3(PCSrc, PC_in + 4, imm, st, Pc_out);
endmodule

module dataPath(input clk);
	reg [31:0] PC_in, inst, BusW, BusA, BusB, ALU_out, imm24to32, address, data_out_mem, imm14to32, Pc_out_wb;
	reg zero, stop, regWr, ExtOpJ, ExtOpI, ExtOpSA, PCAdderSrc, PCAdderSrc_exec, MemRd, MemWr, WBData, EnStack;
	reg [4:0] Function, sa, RA, RB, RW;
	reg [13:0] immediate;
	reg [23:0] immediateJ;
	reg [1:0] Type, ALUSrc, PCSrc, PCSrc_exec;
	reg [2:0] ALUOp;  
	

	generate
		IFetch iF(clk, PC_in, inst);
		decode d(clk, PC_in, inst, zero, BusW, Function, immediate, immediateJ, sa, Type, stop, regWr, ExtOpJ, ExtOpI, ExtOpSA, ALUSrc, 
		PCAdderSrc, ALUOp, MemRd, MemWr, WBData, EnStack, PCSrc, BusA, BusB, RA, RB, RW);
		execute e(clk, PC_in, BusA, BusB, immediate, immediateJ, sa, ExtOpJ, ExtOpI, ExtOpSA, ALUSrc, ALUOp, zero, ALU_out, imm24to32, PCSrc_exec, PCAdderSrc_exec);
		memory_stage ms(clk, ALU_out, BusB, MemRd, MemWr, data_out_mem);
		WB_stage wb(clk, WBData, RW, RA, RB, imm14to32, imm24to32, ALU_out, data_out_mem, PCSrc_exec, PCAdderSrc_exec, PC_in, EnStack, Pc_out_wb);
		assign PC_in = Pc_out_wb;
	endgenerate	
endmodule