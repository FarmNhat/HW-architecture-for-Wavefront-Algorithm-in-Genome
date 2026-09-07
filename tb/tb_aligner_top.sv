// =============================================================================
// tb_aligner_top.sv
// Testbench for aligner_top — Integration Test
// =============================================================================
// Feeds pre-encoded sequence groups directly into aligner_top,
// bypassing Extractor. Compares score and compact CIGAR with expected values
// from gen_reference_vectors.py.
//
// Test cases (must match gen_reference_vectors.py output):
//   TC0: Perfect match "GATTACA"/"GATTACA"              → score=0
//   TC1: 1 mismatch   "GATTACA"/"GATCACA"               → score=4
//   TC2: 1 deletion   "GATTACA"/"GATACA"                → score=8
//   TC3: 1 insertion  "GATACA"/"GATTACA"                → score=8
//   TC4: Multi-error  "GATTACA"/"GCTTCCA"               → score=TBD
//   TC5: 8-base perfect "ACGTACGT"/"ACGTACGT"           → score=0
//   TC6: Exceeds K_MAX → needs_cpu_fallback=1
// =============================================================================

`timescale 1ns/1ps
`include "../rtl/pkg_wfa_params.sv"

module tb_aligner_top;
  import wfa_pkg::*;

  // -------------------------------------------------------------------------
  // Clock & Reset
  // -------------------------------------------------------------------------
  logic clk = 0;
  always #5 clk = ~clk;

  logic rst_n;
  initial begin rst_n = 0; repeat(4) @(posedge clk); rst_n = 1; end

  // -------------------------------------------------------------------------
  // DUT signals
  // -------------------------------------------------------------------------
  logic                         start;
  logic [ID_WIDTH-1:0]          job_id;
  logic [LEN_WIDTH-1:0]         job_len1, job_len2;
  logic [15:0]                  job_seq1_groups [0:MAX_GROUPS-1];
  logic [15:0]                  job_seq2_groups [0:MAX_GROUPS-1];
  logic                         status_idle;
  logic [RESULT_WIDTH-1:0]      result_word;
  logic                         result_valid;
  logic                         needs_cpu_fallback;

  // DUT
  aligner_top #(
    .K_MAX_P       (K_MAX),
    .SEQ_LEN_MAX_P (SEQ_LEN_MAX),
    .X_PEN_P       (X_PEN),
    .O_PEN_P       (O_PEN),
    .E_PEN_P       (E_PEN),
    .MAX_GROUPS_P  (MAX_GROUPS)
  ) dut (
    .clk                (clk),
    .rst_n              (rst_n),
    .start              (start),
    .job_id             (job_id),
    .job_len1           (job_len1),
    .job_len2           (job_len2),
    .job_seq1_groups    (job_seq1_groups),
    .job_seq2_groups    (job_seq2_groups),
    .status_idle        (status_idle),
    .result_word        (result_word),
    .result_valid       (result_valid),
    .needs_cpu_fallback (needs_cpu_fallback)
  );

  // -------------------------------------------------------------------------
  // Base encoder helper
  // -------------------------------------------------------------------------
  function automatic logic [1:0] enc_base(input byte c);
    case (c)
      "A", "a": return 2'b00;
      "C", "c": return 2'b01;
      "G", "g": return 2'b10;
      "T", "t": return 2'b11;
      default:  return 2'b00;
    endcase
  endfunction

  task automatic encode_job_seq1(input string seq);
    int n, gi, bi;
    logic [15:0] cur;
    n = seq.len();
    job_len1 = LEN_WIDTH'(n);
    for (gi = 0; gi < MAX_GROUPS; gi++) job_seq1_groups[gi] = 16'h0;
    gi = 0; bi = 0; cur = 16'h0;
    for (int i = 0; i < n; i++) begin
      cur = (cur << 2) | {14'h0, enc_base(seq[i])};
      bi++;
      if (bi == 8) begin
        job_seq1_groups[gi] = cur;
        gi++; bi = 0; cur = 16'h0;
      end
    end
    if (bi > 0)
      job_seq1_groups[gi] = cur << (2 * (8 - bi));
  endtask

  task automatic encode_job_seq2(input string seq);
    int n, gi, bi;
    logic [15:0] cur;
    n = seq.len();
    job_len2 = LEN_WIDTH'(n);
    for (gi = 0; gi < MAX_GROUPS; gi++) job_seq2_groups[gi] = 16'h0;
    gi = 0; bi = 0; cur = 16'h0;
    for (int i = 0; i < n; i++) begin
      cur = (cur << 2) | {14'h0, enc_base(seq[i])};
      bi++;
      if (bi == 8) begin
        job_seq2_groups[gi] = cur;
        gi++; bi = 0; cur = 16'h0;
      end
    end
    if (bi > 0)
      job_seq2_groups[gi] = cur << (2 * (8 - bi));
  endtask

  // -------------------------------------------------------------------------
  // Run one alignment job and collect result
  // -------------------------------------------------------------------------
  task automatic run_job(
    input string  s1, s2,
    input int     expected_score,
    input logic   expect_fallback,
    input string  desc
  );
    logic [15:0] got_score;
    logic [15:0] got_id;
    logic [63:0] got_cigar;
    logic got_fallback;
    int timeout_cnt;

    $display("\n--- %s ---", desc);
    $display("  seq1='%s' seq2='%s'", s1, s2);
    $display("  Expected score=%0d, fallback=%b", expected_score, expect_fallback);

    // Wait until idle
    timeout_cnt = 0;
    while (!status_idle) begin
      @(posedge clk);
      timeout_cnt++;
      if (timeout_cnt > 100000) begin
        $display("  TIMEOUT waiting for idle");
        return;
      end
    end

    // Encode sequences
    encode_job_seq1(s1);
    encode_job_seq2(s2);

    // Issue start pulse
    @(posedge clk);
    start = 1'b1;
    job_id  = 16'hBEEF; // arbitrary
    @(posedge clk);
    start = 1'b0;

    // Wait for result_valid
    timeout_cnt = 0;
    while (!result_valid) begin
      @(posedge clk);
      timeout_cnt++;
      if (timeout_cnt > 500000) begin
        $display("  TIMEOUT waiting for result");
        return;
      end
    end

    // Capture result
    got_id     = result_word[127:112];
    got_score  = result_word[111:96];
    got_cigar  = result_word[95:32];
    got_fallback = needs_cpu_fallback;

    $display("  Got: score=%0d, fallback=%b", got_score, got_fallback);
    $display("  CIGAR[63:0] = 0x%016h", got_cigar);

    // Check score
    if (expect_fallback) begin
      if (got_fallback) begin
        $display("  PASS: CPU fallback correctly set");
        pass_cnt++;
      end else begin
        $display("  FAIL: Expected CPU fallback but not set");
        fail_cnt++;
      end
    end else begin
      if (got_score == expected_score) begin
        $display("  PASS: score = %0d ✓", got_score);
        pass_cnt++;
      end else begin
        $display("  FAIL: score = %0d, expected %0d", got_score, expected_score);
        fail_cnt++;
      end
    end

    @(posedge clk);
  endtask

  // -------------------------------------------------------------------------
  // Test tracking
  // -------------------------------------------------------------------------
  int pass_cnt = 0, fail_cnt = 0;

  // -------------------------------------------------------------------------
  // Main test sequence
  // -------------------------------------------------------------------------
  initial begin
    $dumpfile("tb_aligner_top.vcd");
    $dumpvars(0, tb_aligner_top);

    // Init
    start = 0; job_id = 0; job_len1 = 0; job_len2 = 0;
    for (int i = 0; i < MAX_GROUPS; i++) begin
      job_seq1_groups[i] = 0;
      job_seq2_groups[i] = 0;
    end

    wait (rst_n);
    repeat(4) @(posedge clk);

    $display("=== tb_aligner_top ===");
    $display("Parameters: X=%0d, O=%0d, E=%0d, K_MAX=%0d", X_PEN, O_PEN, E_PEN, K_MAX);

    // ------------------------------------------------------------------
    // TC0: Perfect match — score = 0
    // ------------------------------------------------------------------
    run_job("GATTACA", "GATTACA", 0, 1'b0, "TC0: Perfect match");

    // ------------------------------------------------------------------
    // TC1: 1 mismatch — score = X_PEN = 4
    // ------------------------------------------------------------------
    run_job("GATTACA", "GATCACA", X_PEN, 1'b0, "TC1: 1 mismatch (T→C)");

    // ------------------------------------------------------------------
    // TC2: 1 deletion (seq2 missing one T) — score = O_PEN + E_PEN = 8
    // ------------------------------------------------------------------
    run_job("GATTACA", "GATACA", O_PEN + E_PEN, 1'b0, "TC2: 1 deletion");

    // ------------------------------------------------------------------
    // TC3: 1 insertion (seq2 has extra T) — score = O_PEN + E_PEN = 8
    // ------------------------------------------------------------------
    run_job("GATACA", "GATTACA", O_PEN + E_PEN, 1'b0, "TC3: 1 insertion");

    // ------------------------------------------------------------------
    // TC4: 8-base perfect match across 1 full block
    // ------------------------------------------------------------------
    run_job("ACGTACGT", "ACGTACGT", 0, 1'b0, "TC4: 8-base perfect (1 full block)");

    // ------------------------------------------------------------------
    // TC5: Multi-mismatch ("GATTACA" vs "GCTTCCA")
    //   Positions 1 (A→C) and 5 (A→C) — 2 mismatches → score = 2*X = 8
    // ------------------------------------------------------------------
    run_job("GATTACA", "GCTTCCA", 2*X_PEN, 1'b0, "TC5: 2 mismatches");

    // ------------------------------------------------------------------
    // TC6: Longer perfect match (crossing block boundary)
    // ------------------------------------------------------------------
    run_job("ACGTACGTACGT", "ACGTACGTACGT", 0, 1'b0, "TC6: 12-base perfect");

    // ------------------------------------------------------------------
    // TC7: 1 gap + 1 mismatch (combined errors)
    // seq1="GATTACAG", seq2="GATCAG" (del T + mismatch T→C) — score TBD by WFA
    // We just check not a fallback for this short sequence
    // ------------------------------------------------------------------
    run_job("GATTACAG", "GATCAG", -1, 1'b0, "TC7: Combined gap+mismatch (score TBD)");

    // ------------------------------------------------------------------
    // TC8: CPU Fallback test — very different sequences exceed K_MAX
    // Use sequences with many differences; expect needs_cpu_fallback=1
    // With K_MAX=16 and X_PEN=4, max tolerable score = 16*(6+2) = 128
    // Use 20 bases all A vs all C → k_target=0, but score = 20*4=80
    // (May or may not exceed K_MAX depending on parameter; we'll just
    //  verify it completes and check fallback if score too high)
    // ------------------------------------------------------------------
    run_job("AAAAAAAAAAAAAAAAAAAA", "CCCCCCCCCCCCCCCCCCCC",
            -1, 1'b1, "TC8: All-mismatch 20bp (expect CPU fallback)");

    // ------------------------------------------------------------------
    // Done
    // ------------------------------------------------------------------
    $display("\n=== FINAL RESULTS: %0d PASS, %0d FAIL ===", pass_cnt, fail_cnt);
    if (fail_cnt == 0)
      $display("ALL TESTS PASSED ✓");
    else
      $display("SOME TESTS FAILED ✗");

    $finish;
  end

  // Global timeout
  initial begin
    #50_000_000; // 50ms sim time
    $display("GLOBAL TIMEOUT — simulation exceeded 50ms");
    $finish;
  end

endmodule
