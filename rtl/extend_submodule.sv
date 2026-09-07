// =============================================================================
// extend_submodule.sv
// WFA Accelerator — Extend Submodule
// =============================================================================
// Chức năng: cho đường chéo k và offset đầu vào h, tìm offset mới h' sao cho
// seq1[h'..h'-1] == seq2[h'-k..h'-k-1] (trượt khi ký tự khớp).
//
// Datapath (theo Hình 5 của bài báo):
//   1. MUX chọn 2 group 8-base liên tiếp của seq1 chứa vị trí h hiện tại
//      và 2 group tương ứng của seq2 chứa vị trí v = h - k.
//   2. Concatenate (32-bit) → Shift → lấy 8 base đầu vào Comparator.
//   3. 8-wide parallel comparator (XOR + priority encoder → matches_num).
//   4. Controller (FSM): cộng dồn matches_num vào h; nếu matches_num==8 và
//      chưa đến cuối chuỗi → lặp; ngược lại → xuất offset_out.
//
// Latency: variable, phụ thuộc số vòng lặp. Controller sinh tín hiệu done.
//
// Synthesizable constraints:
//   - Không dùng dynamic array hay real.
//   - Vòng lặp for trong always_comb có bound là BLOCK_SIZE=8 (unrollable).
//   - Mọi array có kích thước cố định từ parameter.
// =============================================================================

`default_nettype none
`include "pkg_wfa_params.sv"

module extend_submodule
  import wfa_pkg::*;
#(
  parameter int BLOCK_SIZE_P  = BLOCK_SIZE,     // 8 bases per group
  parameter int MAX_GROUPS_P  = MAX_GROUPS,     // ceil(SEQ_LEN_MAX/8)
  parameter int OFF_WIDTH_P   = OFF_WIDTH,      // offset bit width
  parameter int K_WIDTH_P     = K_WIDTH,        // diagonal bit width
  parameter int LEN_WIDTH_P   = LEN_WIDTH       // length bit width
) (
  input  logic                         clk,
  input  logic                         rst_n,

  // Control
  input  logic                         start,        // Pulse: bắt đầu extend
  output logic                         done,         // Pulse: extend hoàn thành

  // Inputs: diagonal và offset đầu vào
  input  logic signed [K_WIDTH_P-1:0]  k_in,         // Đường chéo k (có dấu)
  input  logic [OFF_WIDTH_P-1:0]       offset_in,    // h đầu vào

  // Sequence lengths (để biết khi nào chạm cuối)
  input  logic [LEN_WIDTH_P-1:0]       seq_len1,     // Độ dài seq1
  input  logic [LEN_WIDTH_P-1:0]       seq_len2,     // Độ dài seq2

  // Sequence data: packed groups (từ Extractor)
  input  logic [15:0] seq1_groups [0:MAX_GROUPS_P-1], // seq1 groups 16-bit
  input  logic [15:0] seq2_groups [0:MAX_GROUPS_P-1], // seq2 groups 16-bit

  // Output: offset sau khi extend
  output logic [OFF_WIDTH_P-1:0]       offset_out    // h mới sau trượt
);

  // ---------------------------------------------------------------------------
  // Local parameters
  // ---------------------------------------------------------------------------
  localparam int GRPIDX_W = $clog2(MAX_GROUPS_P);  // bit width for group index

  // ---------------------------------------------------------------------------
  // FSM states
  // ---------------------------------------------------------------------------
  typedef enum logic [1:0] {
    E_IDLE    = 2'd0,
    E_COMPARE = 2'd1,
    E_DONE    = 2'd2
  } ext_state_t;

  ext_state_t ext_state_r;

  // ---------------------------------------------------------------------------
  // Registers
  // ---------------------------------------------------------------------------
  logic [OFF_WIDTH_P-1:0] h_r;    // Current h offset
  logic signed [K_WIDTH_P-1:0] k_r; // Registered k (stable during extend)

  // ---------------------------------------------------------------------------
  // Combinational: compute group index and bit-offset within group for h
  //   group_idx = h / BLOCK_SIZE  (h >> 3 for BLOCK_SIZE=8)
  //   bit_off   = h mod BLOCK_SIZE (h[2:0] for BLOCK_SIZE=8)
  // ---------------------------------------------------------------------------
  logic [GRPIDX_W-1:0] h_grp_idx;
  logic [2:0]           h_bit_off;
  logic [GRPIDX_W-1:0] v_grp_idx;
  logic [2:0]           v_bit_off;

  // v = h - k  (may be negative → invalid, but controller prevents that)
  logic [OFF_WIDTH_P-1:0] v_r;

  always_comb begin
    v_r        = h_r - OFF_WIDTH_P'(signed'(k_r));  // v = h - k
    h_grp_idx  = h_r[OFF_WIDTH_P-1:3];              // h / 8
    h_bit_off  = h_r[2:0];                           // h % 8
    v_grp_idx  = v_r[OFF_WIDTH_P-1:3];              // v / 8
    v_bit_off  = v_r[2:0];                           // v % 8
  end

  // ---------------------------------------------------------------------------
  // MUX: select 2 consecutive groups for seq1 and seq2
  // group[g] and group[g+1] to allow shifting across boundary
  // ---------------------------------------------------------------------------
  logic [15:0] s1_grpA, s1_grpB;  // seq1 group A=floor(h/8), B=floor(h/8)+1
  logic [15:0] s2_grpA, s2_grpB;  // seq2 group A=floor(v/8), B=floor(v/8)+1

  always_comb begin
    s1_grpA = seq1_groups[h_grp_idx];
    s1_grpB = (h_grp_idx < MAX_GROUPS_P-1) ? seq1_groups[h_grp_idx + 1'b1] : 16'h0;
    s2_grpA = seq2_groups[v_grp_idx];
    s2_grpB = (v_grp_idx < MAX_GROUPS_P-1) ? seq2_groups[v_grp_idx + 1'b1] : 16'h0;
  end

  // ---------------------------------------------------------------------------
  // Concatenate and shift to align 8 bases for comparison
  // 32-bit concat = {grpA[15:0], grpB[15:0]} = 16 base window
  // Shift left by (bit_off * 2) bits to align starting base to MSB
  // Then take top 16 bits = 8 bases for comparison
  // ---------------------------------------------------------------------------
  logic [31:0] s1_concat, s2_concat;
  logic [31:0] s1_shifted, s2_shifted;
  logic [15:0] s1_aligned, s2_aligned; // top 16 bits after shift = 8 bases

  always_comb begin
    s1_concat  = {s1_grpA, s1_grpB};
    s2_concat  = {s2_grpA, s2_grpB};
    // Shift left by h_bit_off*2 bits: each base is 2 bits
    s1_shifted = s1_concat << (h_bit_off * 2);
    s2_shifted = s2_concat << (v_bit_off * 2);
    s1_aligned = s1_shifted[31:16];
    s2_aligned = s2_shifted[31:16];
  end

  // ---------------------------------------------------------------------------
  // 8-wide parallel comparator
  // Compare 8 pairs of 2-bit bases simultaneously.
  // match_vec[i] = 1 if base i matches (s1 == s2)
  // matches_num = number of consecutive matches from MSB (position 0)
  //   using XOR: mismatch_vec[i] = 1 if base i mismatches
  //   priority encode the first '1' in mismatch_vec → that is the first mismatch
  // ---------------------------------------------------------------------------
  logic [7:0]  match_vec;   // match_vec[i]=1 if base[i] matches
  logic [3:0]  matches_num; // number of consecutive matches (0..8)

  // Generate match_vec by comparing 2-bit pairs
  always_comb begin
    for (int i = 0; i < BLOCK_SIZE_P; i++) begin
      // Base i occupies bits [15-i*2 : 14-i*2] in the aligned 16-bit word
      // MSB is base 0 (earliest in sequence)
      match_vec[i] = (s1_aligned[15 - i*2 -: 2] == s2_aligned[15 - i*2 -: 2]);
    end
  end

  // Priority encoder: find first mismatch from position 0 (MSB)
  // matches_num = index of first 0 in match_vec, or 8 if all match
  always_comb begin
    matches_num = 4'd8; // default: all 8 match
    // Scan from highest priority (index 0 = leftmost base)
    for (int i = BLOCK_SIZE_P-1; i >= 0; i--) begin
      if (!match_vec[i])
        matches_num = 4'(i); // first mismatch at position i
    end
  end

  // ---------------------------------------------------------------------------
  // Boundary checking: how many bases can we actually compare?
  // Limited by both seq1 and seq2 remaining length.
  //   remaining1 = seq_len1 - h_r
  //   remaining2 = seq_len2 - v_r
  //   can_compare = min(remaining1, remaining2, 8)
  // ---------------------------------------------------------------------------
  logic [OFF_WIDTH_P-1:0] remaining1, remaining2;
  logic [3:0] can_compare;  // 0..8

  always_comb begin
    remaining1 = seq_len1 - h_r;
    remaining2 = seq_len2 - v_r;
    // min(remaining1, remaining2)
    can_compare = (remaining1 < remaining2) ?
                  ((remaining1 > 8) ? 4'd8 : 4'(remaining1)) :
                  ((remaining2 > 8) ? 4'd8 : 4'(remaining2));
  end

  // Effective matches: limited by available bases
  logic [3:0] eff_matches;
  always_comb begin
    eff_matches = (matches_num > can_compare) ? can_compare : matches_num;
  end

  // Reached end of one or both sequences
  logic at_end;
  always_comb begin
    at_end = (h_r >= seq_len1) || (v_r >= seq_len2);
  end

  // ---------------------------------------------------------------------------
  // FSM controller
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ext_state_r <= E_IDLE;
      h_r         <= '0;
      k_r         <= '0;
      done        <= 1'b0;
      offset_out  <= '0;
    end else begin
      done <= 1'b0; // default: not done

      case (ext_state_r)
        // ----------------------------------------------------------------
        E_IDLE: begin
          if (start) begin
            h_r         <= offset_in;
            k_r         <= k_in;
            ext_state_r <= E_COMPARE;
          end
        end

        // ----------------------------------------------------------------
        E_COMPARE: begin
          if (at_end) begin
            // Already at end, no more comparing possible
            offset_out  <= h_r;
            ext_state_r <= E_DONE;
          end else begin
            // Add effective matches to h
            h_r <= h_r + OFF_WIDTH_P'(eff_matches);

            if (eff_matches == 4'd8 && !at_end) begin
              // All 8 matched and not at end → continue to next block
              ext_state_r <= E_COMPARE; // loop
            end else begin
              // Less than 8 matched or at end → done
              offset_out  <= h_r + OFF_WIDTH_P'(eff_matches);
              ext_state_r <= E_DONE;
            end
          end
        end

        // ----------------------------------------------------------------
        E_DONE: begin
          done        <= 1'b1;
          ext_state_r <= E_IDLE;
        end

        default: ext_state_r <= E_IDLE;
      endcase
    end
  end

endmodule

`default_nettype wire
