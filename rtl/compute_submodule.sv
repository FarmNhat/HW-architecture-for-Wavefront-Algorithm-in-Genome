// =============================================================================
// compute_submodule.sv
// WFA Accelerator — Compute Submodule
// =============================================================================
// Triển khai Equation 1 (spec Section 3) cho 1 đường chéo k tại score s:
//
//   I[s,k] = max(M[s-o-e, k-1], I[s-e, k-1]) + 1
//   D[s,k] = max(M[s-o-e, k+1], D[s-e, k+1])
//   M[s,k] = max(M[s-x, k]+1, I[s,k], D[s,k])   ← trước extend
//
// Rồi gọi Extend để hoàn thành M[s,k].
//
// Module này tính cho 1 đường chéo k mỗi lần (P_SUBMODULES=8 instances
// được dùng song song trong aligner_top).
//
// Inputs: giá trị đọc ra từ Wavefront Window (wavefront_window.sv)
//   - from_M_oe_km1 : M[s-o-e, k-1]
//   - from_I_e_km1  : I[s-e, k-1]
//   - from_M_oe_kp1 : M[s-o-e, k+1]
//   - from_D_e_kp1  : D[s-e, k+1]
//   - from_M_x_k    : M[s-x, k]
//
// Output:
//   - i_val    : I[s,k] (= max(from_M_oe_km1, from_I_e_km1) + 1)
//   - d_val    : D[s,k] (= max(from_M_oe_kp1, from_D_e_kp1))
//   - m_pre    : M[s,k] trước extend (= max(from_M_x_k+1, i_val, d_val))
//   - origin   : 5-bit origin code = {m_origin[2:0], d_origin, i_origin}
//   - null_out : 1 = tất cả input đều invalid → bỏ qua đường chéo này
// =============================================================================

//
`include "pkg_wfa_params.sv"

module compute_submodule
  import wfa_pkg::*;
#(
  parameter int OFF_WIDTH_P = OFF_WIDTH,
  parameter int K_WIDTH_P   = K_WIDTH
) (
  // Inputs from Wavefront Window (already read with correct lag and k offset)
  input  wf_entry_t  from_M_oe_km1,  // M[s-o-e, k-1] : for computing I
  input  wf_entry_t  from_I_e_km1,   // I[s-e,   k-1] : for computing I
  input  wf_entry_t  from_M_oe_kp1,  // M[s-o-e, k+1] : for computing D
  input  wf_entry_t  from_D_e_kp1,   // D[s-e,   k+1] : for computing D
  input  wf_entry_t  from_M_x_k,     // M[s-x,   k]   : for computing M (mismatch)

  // Outputs: pre-extend values and origin
  output wf_entry_t  i_val,      // I[s,k] = max(from_M_oe_km1, from_I_e_km1) + 1
  output wf_entry_t  d_val,      // D[s,k] = max(from_M_oe_kp1, from_D_e_kp1)
  output wf_entry_t  m_pre,      // M[s,k] pre-extend = max(M_x+1, I, D)
  output origin_t    origin_out, // 5-bit: {m_origin[2:0], d_origin, i_origin}
  output logic       null_out    // 1 = no valid inputs, this diagonal is null
);

  // ---------------------------------------------------------------------------
  // I computation
  //   from_M_I = M[s-o-e, k-1]    (opens a new gap in seq1)
  //   from_I   = I[s-e,   k-1]    (extends existing gap in seq1)
  //   I[s,k]   = max(from_M_I, from_I) + 1
  //   i_origin = (from_M_I >= from_I) ? ORIG_FROM_M : ORIG_FROM_I
  // ---------------------------------------------------------------------------
  logic i_valid;
  logic i_origin;
  logic [OFF_WIDTH_P-1:0] i_offset;

  always_comb begin
    if (!from_M_oe_km1.valid && !from_I_e_km1.valid) begin
      // Both invalid: I not computable
      i_valid  = 1'b0;
      i_offset = '0;
      i_origin = ORIG_FROM_M; // don't care
    end else if (!from_I_e_km1.valid ||
                 (from_M_oe_km1.valid && from_M_oe_km1.offset >= from_I_e_km1.offset)) begin
      // from_M wins (or from_I invalid)
      i_valid  = 1'b1;
      i_offset = from_M_oe_km1.offset + 1'b1;
      i_origin = ORIG_FROM_M;
    end else begin
      // from_I wins
      i_valid  = 1'b1;
      i_offset = from_I_e_km1.offset + 1'b1;
      i_origin = ORIG_FROM_I;
    end
    i_val.valid  = i_valid;
    i_val.offset = i_offset;
  end

  // ---------------------------------------------------------------------------
  // D computation
  //   from_M_D = M[s-o-e, k+1]    (opens a new gap in seq2)
  //   from_D   = D[s-e,   k+1]    (extends existing gap in seq2)
  //   D[s,k]   = max(from_M_D, from_D)    ← NOTE: NO +1 (k shifts, not h)
  //   d_origin = (from_M_D >= from_D) ? ORIG_FROM_M : ORIG_FROM_D
  // ---------------------------------------------------------------------------
  logic d_valid;
  logic d_origin;
  logic [OFF_WIDTH_P-1:0] d_offset;

  always_comb begin
    if (!from_M_oe_kp1.valid && !from_D_e_kp1.valid) begin
      d_valid  = 1'b0;
      d_offset = '0;
      d_origin = ORIG_FROM_M;
    end else if (!from_D_e_kp1.valid ||
                 (from_M_oe_kp1.valid && from_M_oe_kp1.offset >= from_D_e_kp1.offset)) begin
      d_valid  = 1'b1;
      d_offset = from_M_oe_kp1.offset;
      d_origin = ORIG_FROM_M;
    end else begin
      d_valid  = 1'b1;
      d_offset = from_D_e_kp1.offset;
      d_origin = ORIG_FROM_D;
    end
    d_val.valid  = d_valid;
    d_val.offset = d_offset;
  end

  // ---------------------------------------------------------------------------
  // M pre-extend computation
  //   from_MM = M[s-x, k] + 1    (mismatch)
  //   m_pre   = max(from_MM, I[s,k], D[s,k])
  //   m_origin = argmax(from_MM, I, D)
  //             2'b00 = from mismatch (MM)
  //             2'b01 = from I
  //             2'b10 = from D
  // ---------------------------------------------------------------------------
  logic m_valid;
  logic [2:0] m_origin; // 3-bit as per spec (redundancy bit for safety)
  logic [OFF_WIDTH_P-1:0] m_offset;

  // from_M_x_k + 1 (only if valid)
  wf_entry_t from_MM;
  always_comb begin
    if (from_M_x_k.valid) begin
      from_MM.valid  = 1'b1;
      from_MM.offset = from_M_x_k.offset + 1'b1;
    end else begin
      from_MM.valid  = 1'b0;
      from_MM.offset = '0;
    end
  end

  // 3-way max with argmax
  always_comb begin
    // Default: all invalid
    m_valid  = 1'b0;
    m_offset = '0;
    m_origin = M_ORIG_MM;

    // Start with from_MM
    if (from_MM.valid) begin
      m_valid  = 1'b1;
      m_offset = from_MM.offset;
      m_origin = M_ORIG_MM;
    end

    // Compare with I (i_val)
    if (i_val.valid && (!m_valid || i_val.offset > m_offset)) begin
      m_valid  = 1'b1;
      m_offset = i_val.offset;
      m_origin = M_ORIG_I;
    end

    // Compare with D (d_val)
    if (d_val.valid && (!m_valid || d_val.offset > m_offset)) begin
      m_valid  = 1'b1;
      m_offset = d_val.offset;
      m_origin = M_ORIG_D;
    end

    m_pre.valid  = m_valid;
    m_pre.offset = m_offset;
  end

  // ---------------------------------------------------------------------------
  // Null flag: set if no valid input exists
  // ---------------------------------------------------------------------------
  always_comb begin
    null_out = !(from_M_oe_km1.valid || from_I_e_km1.valid ||
                 from_M_oe_kp1.valid || from_D_e_kp1.valid ||
                 from_M_x_k.valid);
  end

  // ---------------------------------------------------------------------------
  // Origin packing: {m_origin[2:0], d_origin[0], i_origin[0]}
  // ---------------------------------------------------------------------------
  always_comb begin
    origin_out = {m_origin[2:0], d_origin, i_origin};
  end

endmodule

`default_nettype wire
