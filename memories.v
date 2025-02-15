module testbench();
	reg         clk;
	reg         reset;
	wire [31:0] data_to_mem, address_to_mem;
	wire        write_enable;

	top simulated_system (clk, reset, data_to_mem, address_to_mem, write_enable);

	initial	begin
		$dumpfile("test");
		$dumpvars;
		reset<=1; # 2; reset<=0;
		#100;
		$writememh ("memfile_data_after_simulation.hex",simulated_system.dmem.RAM,0,63);
		$finish;
	end

	// generate clock
	always	begin
		clk<=1; # 1; clk<=0; # 1;
	end
endmodule


module top (	input         clk, reset,
		output [31:0] data_to_mem, address_to_mem,
		output        write_enable);

	wire [31:0] pc, instruction, data_from_mem;

	inst_mem  imem(pc[7:2], instruction);
	data_mem  dmem(clk, write_enable, address_to_mem, data_to_mem, data_from_mem);
	processor CPU(clk, reset, pc, instruction, write_enable, address_to_mem, data_to_mem, data_from_mem);
endmodule

//-------------------------------------------------------------------
module data_mem (input clk, we,
		 input  [31:0] address, wd,
		 output [31:0] rd);

	reg [31:0] RAM[1023:0];

	initial begin
		$readmemh ("memfile_data.hex",RAM,0,1023);
	end

	assign rd=RAM[address[31:2]]; // word aligned

	always @ (posedge clk)
		if (we)
			RAM[address[31:2]]<=wd;
endmodule

//-------------------------------------------------------------------
module inst_mem (input  [5:0]  address,
		 output [31:0] rd);

	reg [31:0] RAM[63:0];
	initial begin
		$readmemh ("memfile_inst.hex",RAM,0,11);
	end
	assign rd=RAM[address]; // word aligned
endmodule
