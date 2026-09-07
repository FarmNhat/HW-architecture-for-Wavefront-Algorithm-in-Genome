// =============================================================================
// collector_top.sv
// WFA Accelerator — Collector Top (Packer × N + Scheduler)
// =============================================================================

`default_nettype none
`include "pkg_wfa_params.sv"

module collector_top
  import wfa_pkg::*;
#(
  parameter int NUM_ALIGNERS_P = NUM_ALIGNERS
) (
  input  logic         clk,
  input  logic         rst_n,

  // From N Aligners
  input  logic [RESULT_WIDTH-1:0] aligner_result [0:NUM_ALIGNERS_P-1],
  input  logic [NUM_ALIGNERS_P-1:0] aligner_result_valid,

  // Output stream
  output logic [RESULT_WIDTH-1:0] data_out,
  output logic                    data_out_valid
);

  // ---------------------------------------------------------------------------
  // Packer instances (one per aligner)
  // ---------------------------------------------------------------------------
  logic [RESULT_WIDTH-1:0] pack_results [0:NUM_ALIGNERS_P-1][0:7];
  logic [NUM_ALIGNERS_P-1:0] pack_valid;
  logic [NUM_ALIGNERS_P-1:0] pack_ack;

  genvar gi;
  generate
    for (gi = 0; gi < NUM_ALIGNERS_P; gi++) begin : gen_packer
      collector_packer u_packer (
        .clk          (clk),
        .rst_n        (rst_n),
        .result_in    (aligner_result[gi]),
        .result_valid (aligner_result_valid[gi]),
        .pack_results (pack_results[gi]),
        .pack_valid   (pack_valid[gi]),
        .pack_ack     (pack_ack[gi])
      );
    end
  endgenerate

  // ---------------------------------------------------------------------------
  // Scheduler
  // ---------------------------------------------------------------------------
  collector_scheduler #(.NUM_ALIGNERS_P(NUM_ALIGNERS_P)) u_sched (
    .clk            (clk),
    .rst_n          (rst_n),
    .packer_results (pack_results),
    .packer_valid   (pack_valid),
    .packer_ack     (pack_ack),
    .data_out       (data_out),
    .data_out_valid (data_out_valid)
  );

endmodule

`default_nettype wire
