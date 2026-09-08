// =============================================================================
// extractor_top.sv
// WFA Accelerator — Extractor Top
// =============================================================================
// Wrapper kết nối extractor_assign (bao gồm cả Extract FSM và Assign FSM).
// Trong thiết kế này, extractor_assign đã tích hợp cả hai FSM.

//
`include "pkg_wfa_params.sv"

module extractor_top
  import wfa_pkg::*;
#(
  parameter int NUM_ALIGNERS_P = NUM_ALIGNERS,
  parameter int MAX_GROUPS_P   = MAX_GROUPS,
  parameter int ID_WIDTH_P     = ID_WIDTH,
  parameter int LEN_WIDTH_P    = LEN_WIDTH
) (
  input  logic                       clk,
  input  logic                       rst_n,

  // Byte stream input
  input  logic [7:0]                 data_in,
  input  logic                       data_in_valid,
  output logic                       data_in_ready,

  // Aligner status bus
  input  logic [NUM_ALIGNERS_P-1:0]  aligner_status, // 1 = aligner idle

  // Job outputs (broadcast)
  output logic [NUM_ALIGNERS_P-1:0]  aligner_start,
  output logic [ID_WIDTH_P-1:0]      job_id,
  output logic [LEN_WIDTH_P-1:0]     job_len1,
  output logic [LEN_WIDTH_P-1:0]     job_len2,
  output logic [15:0] job_seq1_groups [0:MAX_GROUPS_P-1],
  output logic [15:0] job_seq2_groups [0:MAX_GROUPS_P-1]
);

  extractor_assign #(
    .NUM_ALIGNERS_P (NUM_ALIGNERS_P),
    .MAX_GROUPS_P   (MAX_GROUPS_P),
    .ID_WIDTH_P     (ID_WIDTH_P),
    .LEN_WIDTH_P    (LEN_WIDTH_P)
  ) u_assign (
    .clk             (clk),
    .rst_n           (rst_n),
    .data_in         (data_in),
    .data_in_valid   (data_in_valid),
    .data_in_ready   (data_in_ready),
    .aligner_status  (aligner_status),
    .aligner_start   (aligner_start),
    .job_id          (job_id),
    .job_len1        (job_len1),
    .job_len2        (job_len2),
    .job_seq1_groups (job_seq1_groups),
    .job_seq2_groups (job_seq2_groups)
  );

endmodule

`default_nettype wire
