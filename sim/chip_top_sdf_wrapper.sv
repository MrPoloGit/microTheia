`timescale 1ns/1ps

module chip_top_sdf_wrapper (
    input        clk_PAD,
    input        rst_n_PAD,
    inout        VSS,
    inout        VDD,
    inout  [1:0] analog_PAD,
    inout [39:0] bidir_PAD,
    input [11:0] input_PAD
);

    chip_top u_chip_top (
        .clk_PAD(clk_PAD),
        .rst_n_PAD(rst_n_PAD),
        .VSS(VSS),
        .VDD(VDD),
        .analog_PAD(analog_PAD),
        .bidir_PAD(bidir_PAD),
        .input_PAD(input_PAD)
    );

    initial begin
`ifdef SDF_FILE
        $display("[SDF] Annotating SDF file: %s", `SDF_FILE);
        $sdf_annotate(`SDF_FILE, u_chip_top);
`else
        $display("[SDF] ERROR: SDF_FILE macro was not defined.");
        $finish;
`endif
    end

endmodule
