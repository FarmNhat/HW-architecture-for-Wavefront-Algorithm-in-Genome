// =============================================================================
// wfa_accelerator_top.sv
// WFA Accelerator — Top-Level Module
// =============================================================================
// Kết nối:
//   Data In → Extractor → N × aligner_top (generate block) → Collector → Data Out
//
// Tham số hóa đầy đủ, override bất kỳ giá trị nào từ bên ngoài.
// =============================================================================

//
`include "pkg_wfa_params.sv"

module wfa_accelerator_top
  import wfa_pkg::*;
#(
  parameter int X_PEN_P       = X_PEN,
  parameter int O_PEN_P       = O_PEN,
  parameter int E_PEN_P       = E_PEN,
  parameter int K_MAX_P       = K_MAX,
  parameter int SEQ_LEN_MAX_P = SEQ_LEN_MAX,
  parameter int NUM_ALIGNERS_P = NUM_ALIGNERS
) (
  input  logic          clk,
  input  logic          rst_n,

  // Input byte stream (packed DNA jobs)
  input  logic [7:0]    data_in,
  input  logic          data_in_valid,
  output logic          data_in_ready,

  // Output result stream
  output logic [RESULT_WIDTH-1:0] data_out,
  output logic                    data_out_valid
);

  // ---------------------------------------------------------------------------
  // Internal signals
  // ---------------------------------------------------------------------------

  // Extractor → Aligners
  logic [NUM_ALIGNERS_P-1:0]  aligner_start;
  logic [ID_WIDTH-1:0]         job_id;
  logic [LEN_WIDTH-1:0]        job_len1, job_len2;
  logic [15:0] job_seq1_groups [0:MAX_GROUPS-1];
  logic [15:0] job_seq2_groups [0:MAX_GROUPS-1];

  // Aligners → Extractor (status)
  logic [NUM_ALIGNERS_P-1:0]  aligner_status_idle;

  // Aligners → Collector
  logic [RESULT_WIDTH-1:0]    aligner_result [0:NUM_ALIGNERS_P-1];
  logic [NUM_ALIGNERS_P-1:0]  aligner_result_valid;

  // ---------------------------------------------------------------------------
  // Extractor
  // ---------------------------------------------------------------------------
  extractor_top #(
    .NUM_ALIGNERS_P (NUM_ALIGNERS_P),
    .MAX_GROUPS_P   (MAX_GROUPS),
    .ID_WIDTH_P     (ID_WIDTH),
    .LEN_WIDTH_P    (LEN_WIDTH)
  ) u_extractor (
    .clk             (clk),
    .rst_n           (rst_n),
    .data_in         (data_in),
    .data_in_valid   (data_in_valid),
    .data_in_ready   (data_in_ready),
    .aligner_status  (aligner_status_idle),
    .aligner_start   (aligner_start),
    .job_id          (job_id),
    .job_len1        (job_len1),
    .job_len2        (job_len2),
    .job_seq1_groups (job_seq1_groups),
    .job_seq2_groups (job_seq2_groups)
  );

  // ---------------------------------------------------------------------------
  // Aligners: N copies via generate
  // ---------------------------------------------------------------------------
  genvar gi;
  generate
    for (gi = 0; gi < NUM_ALIGNERS_P; gi++) begin : gen_aligner
      logic aligner_needs_fallback; // (unused at top level for now)

      aligner_top #(
        .K_MAX_P       (K_MAX_P),
        .SEQ_LEN_MAX_P (SEQ_LEN_MAX_P),
        .X_PEN_P       (X_PEN_P),
        .O_PEN_P       (O_PEN_P),
        .E_PEN_P       (E_PEN_P),
        .MAX_GROUPS_P  (MAX_GROUPS)
      ) u_aligner (
        .clk                (clk),
        .rst_n              (rst_n),
        .start              (aligner_start[gi]),
        .job_id             (job_id),
        .job_len1           (job_len1),
        .job_len2           (job_len2),
        .job_seq1_groups    (job_seq1_groups),
        .job_seq2_groups    (job_seq2_groups),
        .status_idle        (aligner_status_idle[gi]),
        .result_word        (aligner_result[gi]),
        .result_valid       (aligner_result_valid[gi]),
        .needs_cpu_fallback (aligner_needs_fallback)
      );
    end
  endgenerate

  // ---------------------------------------------------------------------------
  // Collector
  // ---------------------------------------------------------------------------
  collector_top #(
    .NUM_ALIGNERS_P(NUM_ALIGNERS_P)
  ) u_collector (
    .clk                  (clk),
    .rst_n                (rst_n),
    .aligner_result       (aligner_result),
    .aligner_result_valid (aligner_result_valid),
    .data_out             (data_out),
    .data_out_valid       (data_out_valid)
  );

endmodule

`default_nettype wire
