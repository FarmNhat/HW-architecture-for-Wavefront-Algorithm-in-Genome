// =============================================================================
// pkg_wfa_params.sv
// WFA Accelerator — Shared Parameters & Typedefs Package
// Target: Xilinx UltraScale+, SystemVerilog 2012
// Reference: Haghi et al., FPL 2021
// =============================================================================
// Mọi module trong dự án đều `import wfa_pkg::*;` để dùng các tham số này.
// Giá trị mặc định cho simulation ngắn (K_MAX=16, SEQ_LEN_MAX=100).
// Để scale lên, thay đổi tại đây hoặc override qua -D flag của compiler.
// =============================================================================

`ifndef PKG_WFA_PARAMS_SV
`define PKG_WFA_PARAMS_SV

package wfa_pkg;

  // ---------------------------------------------------------------------------
  // 1. PENALTY PARAMETERS (gap-affine WFA)
  // ---------------------------------------------------------------------------
  parameter int X_PEN       = 4;   // mismatch penalty
  parameter int O_PEN       = 6;   // gap-open penalty
  parameter int E_PEN       = 2;   // gap-extend penalty

  // ---------------------------------------------------------------------------
  // 2. SEQUENCE & DIAGONAL PARAMETERS
  // ---------------------------------------------------------------------------
  parameter int K_MAX         = 16;  // max diagonal index (range -K_MAX .. +K_MAX)
  parameter int SEQ_LEN_MAX   = 100; // max sequence length supported
  parameter int BLOCK_SIZE    = 8;   // bases per group (must be 8, hardware-fixed)
  parameter int P_SUBMODULES  = 8;   // parallel Extend/Compute submodule copies

  // ---------------------------------------------------------------------------
  // 3. SYSTEM PARAMETERS
  // ---------------------------------------------------------------------------
  parameter int NUM_ALIGNERS  = 4;   // number of parallel Aligner instances
  parameter int ID_WIDTH      = 16;  // job ID bit width
  parameter int LEN_WIDTH     = 8;   // sequence length field bit width (≥ log2(SEQ_LEN_MAX))

  // ---------------------------------------------------------------------------
  // 4. DERIVED PARAMETERS (localparam style — computed from above)
  // ---------------------------------------------------------------------------
  // Number of groups (16-bit words) needed to encode one sequence
  parameter int MAX_GROUPS    = (SEQ_LEN_MAX + BLOCK_SIZE - 1) / BLOCK_SIZE; // ceil(SEQ_LEN_MAX/8)

  // Offset (h) bit width: needs to hold values 0 .. SEQ_LEN_MAX
  parameter int OFF_WIDTH     = $clog2(SEQ_LEN_MAX + 1) + 1; // +1 for safety

  // Diagonal (k) bit width (signed): range -K_MAX .. +K_MAX
  parameter int K_WIDTH       = $clog2(K_MAX + 1) + 1; // signed, e.g. K_MAX=16 → 6 bits

  // Score bit width: max score bounded by SEQ_LEN_MAX * max(X,O+E)
  parameter int S_WIDTH       = $clog2(SEQ_LEN_MAX * (O_PEN + E_PEN) + 1) + 1;

  // Total number of diagonals in the active band
  parameter int NUM_DIAGS     = 2 * K_MAX + 1;

  // ---------------------------------------------------------------------------
  // 5. WAVEFRONT WINDOW DEPTHS (Section III.B.1, spec Eq 1)
  // window_I = E_PEN, window_D = E_PEN, window_M = max(X_PEN, O_PEN + E_PEN)
  // ---------------------------------------------------------------------------
  parameter int WIN_DEPTH_I   = E_PEN;                                    // = 2
  parameter int WIN_DEPTH_D   = E_PEN;                                    // = 2
  parameter int WIN_DEPTH_M   = (X_PEN > (O_PEN + E_PEN)) ? X_PEN : (O_PEN + E_PEN); // = 8

  // ---------------------------------------------------------------------------
  // 6. BACKTRACE RAM PARAMETERS
  // ---------------------------------------------------------------------------
  parameter int BT_CELL_WIDTH  = 5;                     // bits per cell (origin)
  parameter int BT_CELLS_PER_WORD = P_SUBMODULES;       // 8 cells written per RAM word
  parameter int BT_RAM_WIDTH   = BT_CELL_WIDTH * BT_CELLS_PER_WORD; // 40 bits
  parameter int BT_RAM_DEPTH   = SEQ_LEN_MAX * 2;       // safe upper bound for score steps

  // ---------------------------------------------------------------------------
  // 7. COMPACT CIGAR FORMAT (RLE: 8 runs × 8 bits each = 64 bits)
  // Each run: [7:4] = op-type (4-bit), [3:0] = count-1 (4-bit, count = 1..16)
  // op-type encoding:
  //   4'b0000 = MATCH (M)
  //   4'b0001 = MISMATCH (X)
  //   4'b0010 = INSERTION (I)
  //   4'b0011 = DELETION (D)
  //   4'b1111 = EMPTY (run slot unused)
  // ---------------------------------------------------------------------------
  parameter int CIGAR_RUNS     = 8;   // max number of RLE runs in compact CIGAR
  parameter int CIGAR_WIDTH    = 64;  // total compact CIGAR bits (8 runs × 8 bits)

  // CIGAR op-type encoding constants
  parameter logic [3:0] CIGAR_MATCH = 4'b0000;
  parameter logic [3:0] CIGAR_MISMATCH = 4'b0001;
  parameter logic [3:0] CIGAR_INS   = 4'b0010;
  parameter logic [3:0] CIGAR_DEL   = 4'b0011;
  parameter logic [3:0] CIGAR_EMPTY = 4'b1111;

  // ---------------------------------------------------------------------------
  // 8. RESULT WORD LAYOUT (128-bit, sent from Aligner → Collector → Data Out)
  // [127:112] = job_id  (16 bit)
  // [111:96]  = score   (16 bit)
  // [95:32]   = compact_cigar (64 bit, RLE format)
  // [31:0]    = reserved / padding
  // ---------------------------------------------------------------------------
  parameter int RESULT_WIDTH   = 128;

  // ---------------------------------------------------------------------------
  // 9. ORIGIN ENCODING (5-bit packed per cell, stored in Backtrace RAM)
  // Layout: {m_origin[2:0], d_origin[0], i_origin[0]}
  //   i_origin: 0=from_M, 1=from_I
  //   d_origin: 0=from_M, 1=from_D
  //   m_origin: 2'b00=from_MM(mismatch+1), 2'b01=from_I, 2'b10=from_D
  //   (spec uses 3-bit for m_origin for redundancy)
  // ---------------------------------------------------------------------------
  parameter logic ORIG_FROM_M  = 1'b0;  // for I/D origin: came from M matrix
  parameter logic ORIG_FROM_I  = 1'b1;  // for I origin: came from I matrix
  parameter logic ORIG_FROM_D  = 1'b1;  // for D origin: came from D matrix

  parameter logic [2:0] M_ORIG_MM  = 3'b000; // M came from M[s-x,k]+1 (mismatch)
  parameter logic [2:0] M_ORIG_I   = 3'b001; // M came from I[s,k]
  parameter logic [2:0] M_ORIG_D   = 3'b010; // M came from D[s,k]

  // ---------------------------------------------------------------------------
  // 10. BASE ENCODING (2 bits per base)
  // ---------------------------------------------------------------------------
  parameter logic [1:0] BASE_A = 2'b00;
  parameter logic [1:0] BASE_C = 2'b01;
  parameter logic [1:0] BASE_G = 2'b10;
  parameter logic [1:0] BASE_T = 2'b11;

  // ---------------------------------------------------------------------------
  // 11. TYPEDEFS FOR CLEAN INTERFACES
  // ---------------------------------------------------------------------------
  typedef logic [OFF_WIDTH-1:0]           offset_t;   // h offset (unsigned)
  typedef logic signed [K_WIDTH-1:0]      diag_t;     // diagonal k (signed)
  typedef logic [S_WIDTH-1:0]             score_t;    // WFA score s (unsigned)
  typedef logic [ID_WIDTH-1:0]            job_id_t;   // job identifier
  typedef logic [LEN_WIDTH-1:0]           seq_len_t;  // sequence length
  typedef logic [15:0]                    seq_group_t; // 8-base encoded group (2bit×8)
  typedef logic [BT_RAM_WIDTH-1:0]        bt_word_t;  // backtrace RAM word (40-bit)
  typedef logic [CIGAR_WIDTH-1:0]         cigar_t;    // compact CIGAR (64-bit RLE)
  typedef logic [RESULT_WIDTH-1:0]        result_t;   // full result word (128-bit)

  // A wavefront entry: offset + valid flag
  typedef struct packed {
    logic             valid;      // 1 = real value, 0 = NEG_INF
    offset_t          offset;     // h value
  } wf_entry_t;

  // One 5-bit origin cell
  typedef logic [4:0] origin_t;

  // Utility: pack a wf_entry from (valid, offset)
  function automatic wf_entry_t make_entry(input logic v, input offset_t h);
    make_entry.valid  = v;
    make_entry.offset = h;
  endfunction

  // Utility: invalid (NEG_INF) entry
  function automatic wf_entry_t invalid_entry();
    invalid_entry.valid  = 1'b0;
    invalid_entry.offset = '0;
  endfunction

endpackage

`endif // PKG_WFA_PARAMS_SV
