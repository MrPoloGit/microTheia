// Stubs for cells used by the synthesised netlist but absent from the
// ciel-managed gf180mcu_as_sc_mcu7t3v3 PDK Verilog model at commit
// f6bfbd4d3d23c4236ff1f36126489ee59aa35cbd.

`timescale 1ns/1ps

// dfxtp_4: positive-edge D flip-flop, drive strength 4.
// Identical behaviour to dfxtp_2; only drive strength differs.
module gf180mcu_as_sc_mcu7t3v3__dfxtp_4(
	input VPW,
	input VNW,
	input VDD,
	input VSS,

	input CLK,
	input D,
	output Q
);

reg state;
always @(posedge CLK) state <= D;
assign Q = state;

`ifndef FUNCTIONAL
specify
	(posedge CLK => (Q:D)) = (0:0:0, 0:0:0);
	$setup(posedge D, posedge CLK, 0:0:0);
	$setup(negedge D, posedge CLK, 0:0:0);
	$hold(posedge CLK, posedge D, 0:0:0);
	$hold(posedge CLK, negedge D, 0:0:0);
endspecify
`endif

endmodule
