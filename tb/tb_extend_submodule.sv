// =============================================================================
// tb_extend_submodule.sv
// Testbench for extend_submodule
// =============================================================================
// Test cases:
//   1. Perfect match: seq1 = seq2 = "GATTACA" → extend(k=0, h=0) should return h=7
//   2. 1 mismatch at position 3: seq1="GATTACA", seq2="GATCACA"
//      extend(k=0, h=0) → h=3 (khớp G,A,T rồi dừng ở T vs C)
//   3. Multi-block: seq1 = seq2 = 16 bases → extend(k=0, h=0) → h=16
//   4. Offset in middle: seq1=seq2="ACGTACGT", extend(k=0, h=4) → h=8
//   5. k≠0: seq1="GATTACA"(7), seq2="ATTACA"(6), k=1
//      extend(k=1, h=1) → h at position after extending with v=h-k=0
// =============================================================================

`timescale 1ns/1ps
`include "../rtl/pkg_wfa_params.sv"

module tb_extend_submodule;
  import wfa_pkg::*;

  // -------------------------------------------------------------------------
  // Clock & Reset
  // -------------------------------------------------------------------------
  logic clk = 0;
  always #5 clk = ~clk; // 100 MHz

  logic rst_n;
  initial begin
    rst_n = 0;
    repeat(4) @(posedge clk);
    rst_n = 1;
  end

  // -------------------------------------------------------------------------
  // DUT signals
  // -------------------------------------------------------------------------
  logic                        start;
  logic                        done;
  logic signed [K_WIDTH-1:0]   k_in;
  logic [OFF_WIDTH-1:0]        offset_in;
  logic [LEN_WIDTH-1:0]        seq_len1, seq_len2;
  logic [15:0]                 seq1_groups [0:MAX_GROUPS-1];
  logic [15:0]                 seq2_groups [0:MAX_GROUPS-1];
  logic [OFF_WIDTH-1:0]        offset_out;

  // DUT
  extend_submodule #(
    .MAX_GROUPS_P (MAX_GROUPS),
    .OFF_WIDTH_P  (OFF_WIDTH),
    .K_WIDTH_P    (K_WIDTH),
    .LEN_WIDTH_P  (LEN_WIDTH)
  ) dut (
    .clk         (clk),
    .rst_n       (rst_n),
    .start       (start),
    .done        (done),
    .k_in        (k_in),
    .offset_in   (offset_in),
    .seq_len1    (seq_len1),
    .seq_len2    (seq_len2),
    .seq1_groups (seq1_groups),
    .seq2_groups (seq2_groups),
    .offset_out  (offset_out)
  );

  // -------------------------------------------------------------------------
  // Helpers
  // -------------------------------------------------------------------------
  // Base encoder: ASCII char → 2-bit
  function automatic logic [1:0] enc_base(input byte c);
    case (c)
      "A", "a": return 2'b00;
      "C", "c": return 2'b01;
      "G", "g": return 2'b10;
      "T", "t": return 2'b11;
      default:  return 2'b00;
    endcase
  endfunction

  // Encode string seq1 directly into seq1_groups and seq_len1
  task automatic encode_seq1(input string seq);
    int n, gi, bi;
    logic [15:0] cur;
    n = seq.len();
    seq_len1 = LEN_WIDTH'(n);
    for (gi = 0; gi < MAX_GROUPS; gi++) seq1_groups[gi] = 16'h0;
    gi = 0; bi = 0;
    cur = 16'h0;
    for (int i = 0; i < n; i++) begin
      cur = (cur << 2) | {14'h0, enc_base(seq[i])};
      bi++;
      if (bi == 8) begin
        seq1_groups[gi] = cur;
        gi++; bi = 0; cur = 16'h0;
      end
    end
    if (bi > 0) begin
      seq1_groups[gi] = cur << (2 * (8 - bi));
    end
  endtask

  // Encode string seq2 directly into seq2_groups and seq_len2
  task automatic encode_seq2(input string seq);
    int n, gi, bi;
    logic [15:0] cur;
    n = seq.len();
    seq_len2 = LEN_WIDTH'(n);
    for (gi = 0; gi < MAX_GROUPS; gi++) seq2_groups[gi] = 16'h0;
    gi = 0; bi = 0;
    cur = 16'h0;
    for (int i = 0; i < n; i++) begin
      cur = (cur << 2) | {14'h0, enc_base(seq[i])};
      bi++;
      if (bi == 8) begin
        seq2_groups[gi] = cur;
        gi++; bi = 0; cur = 16'h0;
      end
    end
    if (bi > 0) begin
      seq2_groups[gi] = cur << (2 * (8 - bi));
    end
  endtask

  // Run extend and wait for done
  task automatic run_extend(
    input signed [K_WIDTH-1:0] k,
    input [OFF_WIDTH-1:0] h_in,
    output [OFF_WIDTH-1:0] h_out
  );
    @(posedge clk);
    start      = 1'b1;
    k_in       = k;
    offset_in  = h_in;
    @(posedge clk);
    start = 1'b0;
    // Wait for done (timeout after 100 cycles)
    begin : wait_done_loop
      int timeout_cnt = 0;
      while (!done && timeout_cnt < 100) begin
        @(posedge clk);
        timeout_cnt++;
      end
      if (!done) begin
        $display("ERROR: extend timeout!");
        $finish;
      end
      h_out = offset_out;
    end
    @(posedge clk); // settle
  endtask

  // -------------------------------------------------------------------------
  // Test result tracking
  // -------------------------------------------------------------------------
  int pass_cnt = 0, fail_cnt = 0;

  task automatic check(
    input string test_name,
    input [OFF_WIDTH-1:0] got,
    input [OFF_WIDTH-1:0] expected
  );
    if (got === expected) begin
      $display("  PASS: %s → offset_out = %0d (expected %0d)", test_name, got, expected);
      pass_cnt++;
    end else begin
      $display("  FAIL: %s → offset_out = %0d, expected %0d", test_name, got, expected);
      fail_cnt++;
    end
  endtask

  // -------------------------------------------------------------------------
  logic [OFF_WIDTH-1:0] h_result;

  // -------------------------------------------------------------------------
  // Main test
  // -------------------------------------------------------------------------
  initial begin
    // Dump waveforms
    $dumpfile("tb_extend_submodule.vcd");
    $dumpvars(0, tb_extend_submodule);

    start = 0;
    for (int i = 0; i < MAX_GROUPS; i++) begin
      seq1_groups[i] = 0;
      seq2_groups[i] = 0;
    end
    seq_len1 = 0; seq_len2 = 0;
    k_in = 0; offset_in = 0;

    // Wait for reset
    wait (rst_n);
    repeat(2) @(posedge clk);

    $display("=== tb_extend_submodule ===");

    // ------------------------------------------------------------------
    // TC 1: Perfect match, k=0, h=0: GATTACA == GATTACA → h=7
    // ------------------------------------------------------------------
    $display("\nTC1: Perfect match 'GATTACA' == 'GATTACA'");
    encode_seq1("GATTACA");
    encode_seq2("GATTACA");
    run_extend(0, 0, h_result);
    check("TC1 perfect_match", h_result, OFF_WIDTH'(7));

    // ------------------------------------------------------------------
    // TC 2: 1 mismatch at position 3
    // seq1="GATTACA", seq2="GATCACA"
    // extend(k=0, h=0) → matches G,A,T then stops at T vs C → h=3
    // ------------------------------------------------------------------
    $display("\nTC2: 1 mismatch — seq1='GATTACA', seq2='GATCACA'");
    encode_seq1("GATTACA");
    encode_seq2("GATCACA");
    run_extend(0, 0, h_result);
    check("TC2 mismatch_at_3", h_result, OFF_WIDTH'(3));

    // ------------------------------------------------------------------
    // TC 3: Longer match crossing block boundary (16 bases)
    // seq1 = seq2 = "ACGTACGTACGTACGT" (16 bases, 2 full groups)
    // extend(k=0, h=0) → h=16
    // ------------------------------------------------------------------
    $display("\nTC3: 16-base perfect match (crossing block boundary)");
    encode_seq1("ACGTACGTACGTACGT");
    encode_seq2("ACGTACGTACGTACGT");
    run_extend(0, 0, h_result);
    check("TC3 16base_match", h_result, OFF_WIDTH'(16));

    // ------------------------------------------------------------------
    // TC 4: Match from middle offset
    // seq1 = seq2 = "ACGTACGT", extend(k=0, h=4) → h=8
    // ------------------------------------------------------------------
    $display("\nTC4: Match from offset 4 in 8-base seq");
    encode_seq1("ACGTACGT");
    encode_seq2("ACGTACGT");
    run_extend(0, 4, h_result);
    check("TC4 mid_offset", h_result, OFF_WIDTH'(8));

    // ------------------------------------------------------------------
    // TC 5: No match at all (AAAA vs CCCC)
    // extend(k=0, h=0) → h=0 (no matches)
    // ------------------------------------------------------------------
    $display("\nTC5: No match at all — 'AAAA' vs 'CCCC'");
    encode_seq1("AAAA");
    encode_seq2("CCCC");
    run_extend(0, 0, h_result);
    check("TC5 no_match", h_result, OFF_WIDTH'(0));

    // ------------------------------------------------------------------
    // TC 6: k≠0 — diagonal k=1
    // seq1="GATTACA"(7), seq2="ATTACA"(6), k=1 means v=h-1
    // At h=1: seq1[1]='A', seq2[0]='A' → match; h=2: seq1[2]='T', seq2[1]='T' → match; ...
    // All 6 of seq2 match → extend(k=1, h=1) → h=7
    // ------------------------------------------------------------------
    $display("\nTC6: k=1, seq1='GATTACA' seq2='ATTACA'");
    encode_seq1("GATTACA");
    encode_seq2("ATTACA");
    run_extend(1, 1, h_result);
    check("TC6 k=1_diagonal", h_result, OFF_WIDTH'(7));

    // ------------------------------------------------------------------
    // Done
    // ------------------------------------------------------------------
    $display("\n=== RESULTS: %0d PASS, %0d FAIL ===", pass_cnt, fail_cnt);
    if (fail_cnt == 0)
      $display("ALL TESTS PASSED");
    else
      $display("SOME TESTS FAILED");

    $finish;
  end

  // Timeout watchdog
  initial begin
    #10000;
    $display("TIMEOUT: simulation exceeded 10us");
    $finish;
  end

endmodule
