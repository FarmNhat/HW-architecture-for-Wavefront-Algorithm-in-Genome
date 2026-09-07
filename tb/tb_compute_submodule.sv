// =============================================================================
// tb_compute_submodule.sv
// Testbench for compute_submodule
// =============================================================================
// compute_submodule là pure combinational — testbench kiểm tra các giá trị
// đầu ra tĩnh khi thay đổi inputs.
//
// Test cases:
//   1. Tất cả inputs invalid → null_out=1, m_pre.valid=0
//   2. Chỉ from_M_x_k valid (mismatch case) → m_pre từ MM
//   3. from_M_oe_km1 valid (gap open I) → I[s,k] computed
//   4. from_I_e_km1 valid → I từ I (gap extend)
//   5. I wins over MM → m_origin = I
//   6. D computation: from_M_oe_kp1 valid
//   7. All three valid: argmax check
// =============================================================================

`timescale 1ns/1ps
`include "../rtl/pkg_wfa_params.sv"

module tb_compute_submodule;
  import wfa_pkg::*;

  // -------------------------------------------------------------------------
  // DUT signals
  // -------------------------------------------------------------------------
  wf_entry_t from_M_oe_km1;
  wf_entry_t from_I_e_km1;
  wf_entry_t from_M_oe_kp1;
  wf_entry_t from_D_e_kp1;
  wf_entry_t from_M_x_k;

  wf_entry_t i_val_out;
  wf_entry_t d_val_out;
  wf_entry_t m_pre_out;
  origin_t   origin_out;
  logic      null_out;

  // DUT
  compute_submodule #(
    .OFF_WIDTH_P(OFF_WIDTH),
    .K_WIDTH_P  (K_WIDTH)
  ) dut (
    .from_M_oe_km1 (from_M_oe_km1),
    .from_I_e_km1  (from_I_e_km1),
    .from_M_oe_kp1 (from_M_oe_kp1),
    .from_D_e_kp1  (from_D_e_kp1),
    .from_M_x_k    (from_M_x_k),
    .i_val         (i_val_out),
    .d_val         (d_val_out),
    .m_pre         (m_pre_out),
    .origin_out    (origin_out),
    .null_out      (null_out)
  );

  // -------------------------------------------------------------------------
  // Helpers
  // -------------------------------------------------------------------------
  function automatic wf_entry_t mk(input logic v, input int h);
    mk.valid  = v;
    mk.offset = OFF_WIDTH'(h);
  endfunction

  function automatic wf_entry_t invalid();
    invalid.valid  = 1'b0;
    invalid.offset = '0;
  endfunction

  // -------------------------------------------------------------------------
  // Test tracking
  // -------------------------------------------------------------------------
  int pass_cnt = 0, fail_cnt = 0;

  task automatic check_val(
    input string name,
    input logic got_valid, input int got_off,
    input logic exp_valid, input int exp_off
  );
    if (got_valid === exp_valid && (got_off === exp_off || !exp_valid)) begin
      $display("  PASS: %s → valid=%b, offset=%0d", name, got_valid, got_off);
      pass_cnt++;
    end else begin
      $display("  FAIL: %s → valid=%b,offset=%0d expected valid=%b,offset=%0d",
               name, got_valid, got_off, exp_valid, exp_off);
      fail_cnt++;
    end
  endtask

  task automatic check_bit(
    input string name,
    input logic got, input logic expected
  );
    if (got === expected) begin
      $display("  PASS: %s = %b", name, got);
      pass_cnt++;
    end else begin
      $display("  FAIL: %s = %b, expected %b", name, got, expected);
      fail_cnt++;
    end
  endtask

  // -------------------------------------------------------------------------
  // Main test
  // -------------------------------------------------------------------------
  initial begin
    $dumpfile("tb_compute_submodule.vcd");
    $dumpvars(0, tb_compute_submodule);

    // Default all inputs to invalid
    from_M_oe_km1 = invalid();
    from_I_e_km1  = invalid();
    from_M_oe_kp1 = invalid();
    from_D_e_kp1  = invalid();
    from_M_x_k    = invalid();

    #1; // settle

    $display("=== tb_compute_submodule ===");

    // ==================================================================
    // TC 1: All invalid → null
    // ==================================================================
    $display("\nTC1: All inputs invalid → null_out=1");
    #5;
    check_bit("TC1 null_out", null_out, 1'b1);
    check_bit("TC1 m_pre.valid", m_pre_out.valid, 1'b0);

    // ==================================================================
    // TC 2: Only from_M_x_k valid (mismatch case, h=5)
    //   m_pre = 5+1 = 6, from MM
    // ==================================================================
    $display("\nTC2: Only from_M_x_k=5 → m_pre=6, origin=MM");
    from_M_x_k = mk(1, 5);
    #5;
    check_bit("TC2 null_out", null_out, 1'b0);
    check_val("TC2 m_pre",    m_pre_out.valid, int'(m_pre_out.offset), 1, 6);
    // origin m_origin should be M_ORIG_MM = 3'b000
    check_bit("TC2 m_origin[0]", origin_out[4], M_ORIG_MM[2]);
    check_bit("TC2 m_origin[1]", origin_out[3], M_ORIG_MM[1]);
    check_bit("TC2 m_origin[2]", origin_out[2], M_ORIG_MM[0]);

    // ==================================================================
    // TC 3: from_M_oe_km1 valid (gap open I), h=4
    //   i_val = 4+1 = 5, i_origin = ORIG_FROM_M
    // ==================================================================
    $display("\nTC3: from_M_oe_km1=4, from_M_x_k=3 → I opens gap (i_val=5), MM=4, I wins M");
    from_M_x_k    = mk(1, 3); // m_pre from MM = 4
    from_M_oe_km1 = mk(1, 4); // i_val = 5
    from_I_e_km1  = invalid();
    #5;
    check_val("TC3 i_val", i_val_out.valid, int'(i_val_out.offset), 1, 5);
    check_bit("TC3 i_origin", origin_out[0], ORIG_FROM_M); // i came from M
    // m_pre should be max(MM=4, I=5) = 5, origin=I
    check_val("TC3 m_pre",   m_pre_out.valid, int'(m_pre_out.offset), 1, 5);
    check_bit("TC3 m_origin=I", origin_out[4], M_ORIG_I[2]);

    // ==================================================================
    // TC 4: from_I_e_km1 wins over from_M_oe_km1
    //   from_M_oe_km1=2, from_I_e_km1=5 → i_val=6, i_origin=FROM_I
    // ==================================================================
    $display("\nTC4: from_I_e_km1=5 wins over from_M_oe_km1=2 → i_val=6");
    from_M_oe_km1 = mk(1, 2);
    from_I_e_km1  = mk(1, 5);
    from_M_x_k    = invalid();
    #5;
    check_val("TC4 i_val",    i_val_out.valid, int'(i_val_out.offset), 1, 6);
    check_bit("TC4 i_origin", origin_out[0], ORIG_FROM_I);

    // ==================================================================
    // TC 5: D computation
    //   from_M_oe_kp1=7, from_D_e_kp1=invalid → d_val=7, d_origin=FROM_M
    // ==================================================================
    $display("\nTC5: from_M_oe_kp1=7 → d_val=7, d_origin=FROM_M");
    from_M_oe_km1 = invalid(); from_I_e_km1  = invalid();
    from_M_oe_kp1 = mk(1, 7); from_D_e_kp1  = invalid();
    from_M_x_k    = invalid();
    #5;
    check_val("TC5 d_val",    d_val_out.valid, int'(d_val_out.offset), 1, 7);
    check_bit("TC5 d_origin", origin_out[1], ORIG_FROM_M);

    // ==================================================================
    // TC 6: D wins over M (from_D > from_M)
    //   from_M_oe_kp1=3, from_D_e_kp1=9 → d_val=9, d_origin=FROM_D
    // ==================================================================
    $display("\nTC6: from_D_e_kp1=9 > from_M_oe_kp1=3 → d_val=9, d_origin=FROM_D");
    from_M_oe_kp1 = mk(1, 3);
    from_D_e_kp1  = mk(1, 9);
    #5;
    check_val("TC6 d_val",    d_val_out.valid, int'(d_val_out.offset), 1, 9);
    check_bit("TC6 d_origin", origin_out[1], ORIG_FROM_D);

    // ==================================================================
    // TC 7: Three-way max: MM=10, I=8, D=12 → m_pre=12 from D
    //   from_M_x_k=9 → MM=10
    //   from_M_oe_km1=7 → i_val=8
    //   from_D_e_kp1=12 → d_val=12
    // ==================================================================
    $display("\nTC7: Three-way max: MM=10, I=8, D=12 → m_pre=12 from D");
    from_M_x_k    = mk(1, 9);  // MM = 9+1 = 10
    from_M_oe_km1 = mk(1, 7);  // I  = 7+1 = 8
    from_I_e_km1  = invalid();
    from_M_oe_kp1 = invalid();
    from_D_e_kp1  = mk(1, 12); // D  = 12
    #5;
    check_val("TC7 m_pre",      m_pre_out.valid, int'(m_pre_out.offset), 1, 12);
    // origin m_origin should be M_ORIG_D
    check_bit("TC7 m_origin_D", origin_out[4], M_ORIG_D[2]);

    // ==================================================================
    // TC 8: Both I and MM equal → MM has priority (tie-breaking)
    //   from_M_x_k=5 → MM=6, from_M_oe_km1=5 → I=6
    //   Expected: m_pre=6, winner could be either (impl-defined, but valid)
    // ==================================================================
    $display("\nTC8: Tie MM=6 vs I=6 (just check valid and value)");
    from_M_x_k    = mk(1, 5);
    from_M_oe_km1 = mk(1, 5);
    from_I_e_km1  = invalid();
    from_M_oe_kp1 = invalid();
    from_D_e_kp1  = invalid();
    #5;
    check_val("TC8 m_pre_value", m_pre_out.valid, int'(m_pre_out.offset), 1, 6);
    // Just verify valid and value = 6; don't check origin (tie-dependent)

    // ==================================================================
    $display("\n=== RESULTS: %0d PASS, %0d FAIL ===", pass_cnt, fail_cnt);
    if (fail_cnt == 0)
      $display("ALL TESTS PASSED");
    else
      $display("SOME TESTS FAILED");

    $finish;
  end

  // Timeout
  initial begin
    #1000;
    $display("TIMEOUT");
    $finish;
  end

endmodule
