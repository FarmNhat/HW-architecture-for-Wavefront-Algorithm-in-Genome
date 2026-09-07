// =============================================================================
// extractor_assign.sv
// WFA Accelerator — Extractor: Group Maker + Assign FSM
// =============================================================================
// Chức năng:
//   1. Nhận byte stream (data_in / data_in_valid) chứa:
//      [ID:2B][Len1:1B][Seq1:Len1B][Len2:1B][Seq2:Len2B] cho mỗi job.
//   2. Mã hóa từng byte ASCII → 2-bit bằng base_encoder.
//   3. Đóng gói 8 base liên tiếp thành 1 group 16-bit (MSB = base đầu tiên).
//   4. FSM Assign: khi có Aligner báo IDLE (aligner_status[i]=1),
//      phân công job tiếp theo cho Aligner đó qua bus job_*.
//
// FSM States:
//   IDLE        → chờ data_in_valid
//   READ_HEADER → đọc 2 byte ID
//   READ_LEN1   → đọc 1 byte Len1
//   READ_SEQ1   → đọc Len1 bytes, encode + pack thành groups
//   READ_LEN2   → đọc 1 byte Len2
//   READ_SEQ2   → đọc Len2 bytes, encode + pack
//   ASSIGN      → tìm Aligner rảnh, phát job, quay lại IDLE
// =============================================================================

`default_nettype none
`include "pkg_wfa_params.sv"

module extractor_assign
  import wfa_pkg::*;
#(
  // Override-able parameters
  parameter int NUM_ALIGNERS_P  = NUM_ALIGNERS,
  parameter int MAX_GROUPS_P    = MAX_GROUPS,
  parameter int ID_WIDTH_P      = ID_WIDTH,
  parameter int LEN_WIDTH_P     = LEN_WIDTH
) (
  input  logic                      clk,
  input  logic                      rst_n,

  // -------------------------------------------------------------------------
  // Input byte stream interface
  // -------------------------------------------------------------------------
  input  logic [7:0]                data_in,        // Byte stream (raw ASCII DNA)
  input  logic                      data_in_valid,  // Byte stream valid strobe
  output logic                      data_in_ready,  // Backpressure (1 = ready to accept)

  // -------------------------------------------------------------------------
  // Aligner status interface (from Aligners)
  // -------------------------------------------------------------------------
  input  logic [NUM_ALIGNERS_P-1:0] aligner_status, // 1 = Aligner[i] is IDLE/ready

  // -------------------------------------------------------------------------
  // Output job interface (broadcast, gated by aligner_start)
  // -------------------------------------------------------------------------
  output logic [NUM_ALIGNERS_P-1:0] aligner_start,  // Pulse: 1 cycle per assigned job
  output logic [ID_WIDTH_P-1:0]     job_id,          // Job identifier
  output logic [LEN_WIDTH_P-1:0]    job_len1,        // Length of seq1
  output logic [LEN_WIDTH_P-1:0]    job_len2,        // Length of seq2
  // Packed groups: group[0] = first 8 bases (MSB=base0, LSB=base7)
  output logic [15:0] job_seq1_groups [0:MAX_GROUPS_P-1], // seq1 encoded groups
  output logic [15:0] job_seq2_groups [0:MAX_GROUPS_P-1]  // seq2 encoded groups
);

  // ---------------------------------------------------------------------------
  // Local parameters
  // ---------------------------------------------------------------------------
  localparam int SEQ_BYTES_MAX = SEQ_LEN_MAX; // max bytes per sequence

  // ---------------------------------------------------------------------------
  // Internal registers
  // ---------------------------------------------------------------------------

  // FSM state encoding
  typedef enum logic [2:0] {
    S_IDLE       = 3'd0,
    S_RD_HDR1    = 3'd1,  // Read ID byte 0
    S_RD_HDR2    = 3'd2,  // Read ID byte 1
    S_RD_LEN1    = 3'd3,  // Read Len1
    S_RD_SEQ1    = 3'd4,  // Read Seq1 bytes
    S_RD_LEN2    = 3'd5,  // Read Len2
    S_RD_SEQ2    = 3'd6,  // Read Seq2 bytes
    S_ASSIGN     = 3'd7   // Assign job to idle aligner
  } state_t;

  state_t state_r, state_nx;

  // Staging registers for current job being assembled
  logic [ID_WIDTH_P-1:0]  cur_id_r;
  logic [LEN_WIDTH_P-1:0] cur_len1_r, cur_len2_r;

  // Encoded group storage for current job
  logic [15:0] seq1_groups_r [0:MAX_GROUPS_P-1];
  logic [15:0] seq2_groups_r [0:MAX_GROUPS_P-1];

  // Counters
  logic [LEN_WIDTH_P-1:0] byte_cnt_r;   // bytes read in current sequence
  logic [2:0]              base_pos_r;   // position within current group (0..7)
  logic [$clog2(MAX_GROUPS_P)-1:0] grp_cnt_r; // current group index

  // Current group being assembled (16 bits for 8 bases)
  logic [15:0] cur_group_r;

  // ---------------------------------------------------------------------------
  // Base encoder instantiation (1 encoder, used each clock when valid)
  // ---------------------------------------------------------------------------
  logic [1:0] enc_base;
  logic       enc_invalid;

  base_encoder u_enc (
    .ascii_in    (data_in),
    .base_out    (enc_base),
    .invalid_base(enc_invalid)
  );

  // ---------------------------------------------------------------------------
  // FSM: next-state logic
  // ---------------------------------------------------------------------------
  always_comb begin
    state_nx     = state_r;
    data_in_ready = 1'b0;

    case (state_r)
      S_IDLE: begin
        data_in_ready = 1'b1;
        if (data_in_valid) state_nx = S_RD_HDR1;
      end

      S_RD_HDR1: begin
        data_in_ready = 1'b1;
        if (data_in_valid) state_nx = S_RD_HDR2;
      end

      S_RD_HDR2: begin
        data_in_ready = 1'b1;
        if (data_in_valid) state_nx = S_RD_LEN1;
      end

      S_RD_LEN1: begin
        data_in_ready = 1'b1;
        if (data_in_valid) state_nx = S_RD_SEQ1;
      end

      S_RD_SEQ1: begin
        data_in_ready = 1'b1;
        // Stay in this state until all Len1 bytes are read
        if (data_in_valid && (byte_cnt_r == cur_len1_r - 1'b1))
          state_nx = S_RD_LEN2;
      end

      S_RD_LEN2: begin
        data_in_ready = 1'b1;
        if (data_in_valid) state_nx = S_RD_SEQ2;
      end

      S_RD_SEQ2: begin
        data_in_ready = 1'b1;
        if (data_in_valid && (byte_cnt_r == cur_len2_r - 1'b1))
          state_nx = S_ASSIGN;
      end

      S_ASSIGN: begin
        // Wait until an aligner accepts the job (aligner_start goes high)
        if (|aligner_status) state_nx = S_IDLE;
      end

      default: state_nx = S_IDLE;
    endcase
  end

  // ---------------------------------------------------------------------------
  // FSM: sequential logic + datapath
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state_r    <= S_IDLE;
      cur_id_r   <= '0;
      cur_len1_r <= '0;
      cur_len2_r <= '0;
      byte_cnt_r <= '0;
      base_pos_r <= '0;
      grp_cnt_r  <= '0;
      cur_group_r <= '0;
      for (int i = 0; i < MAX_GROUPS_P; i++) begin
        seq1_groups_r[i] <= '0;
        seq2_groups_r[i] <= '0;
      end
    end else begin
      state_r <= state_nx;

      case (state_r)
        // ------------------------------------------------------------------
        S_IDLE: begin
          // Reset counters for new job
          byte_cnt_r  <= '0;
          base_pos_r  <= '0;
          grp_cnt_r   <= '0;
          cur_group_r <= '0;
          if (data_in_valid)
            cur_id_r[ID_WIDTH_P-1:8] <= data_in; // first byte of ID
        end

        // ------------------------------------------------------------------
        S_RD_HDR1: begin
          if (data_in_valid)
            cur_id_r[ID_WIDTH_P-1:8] <= data_in;
        end

        // ------------------------------------------------------------------
        S_RD_HDR2: begin
          if (data_in_valid)
            cur_id_r[7:0] <= data_in;
        end

        // ------------------------------------------------------------------
        S_RD_LEN1: begin
          if (data_in_valid) begin
            cur_len1_r <= data_in[LEN_WIDTH_P-1:0];
            byte_cnt_r <= '0;
            base_pos_r <= '0;
            grp_cnt_r  <= '0;
            cur_group_r <= '0;
          end
        end

        // ------------------------------------------------------------------
        S_RD_SEQ1: begin
          if (data_in_valid) begin
            // Pack base into current group (MSB = earliest base)
            cur_group_r <= {cur_group_r[13:0], enc_base};

            if (base_pos_r == 3'd7) begin
              // Group is complete, store it
              seq1_groups_r[grp_cnt_r] <= {cur_group_r[13:0], enc_base};
              grp_cnt_r  <= grp_cnt_r + 1'b1;
              base_pos_r <= '0;
              cur_group_r <= '0;
            end else begin
              base_pos_r <= base_pos_r + 1'b1;
            end

            // Handle last byte (may be partial group)
            if (byte_cnt_r == cur_len1_r - 1'b1) begin
              // Flush partial group (pad with zeros on right)
              if (base_pos_r != 3'd7) begin
                // Shift remaining bits to MSB position
                seq1_groups_r[grp_cnt_r] <= {cur_group_r[13:0], enc_base} << (2 * (7 - base_pos_r));
              end
              // Reset for seq2
              byte_cnt_r  <= '0;
              base_pos_r  <= '0;
              grp_cnt_r   <= '0;
              cur_group_r <= '0;
            end else begin
              byte_cnt_r <= byte_cnt_r + 1'b1;
            end
          end
        end

        // ------------------------------------------------------------------
        S_RD_LEN2: begin
          if (data_in_valid) begin
            cur_len2_r <= data_in[LEN_WIDTH_P-1:0];
            byte_cnt_r <= '0;
            base_pos_r <= '0;
            grp_cnt_r  <= '0;
            cur_group_r <= '0;
          end
        end

        // ------------------------------------------------------------------
        S_RD_SEQ2: begin
          if (data_in_valid) begin
            cur_group_r <= {cur_group_r[13:0], enc_base};

            if (base_pos_r == 3'd7) begin
              seq2_groups_r[grp_cnt_r] <= {cur_group_r[13:0], enc_base};
              grp_cnt_r  <= grp_cnt_r + 1'b1;
              base_pos_r <= '0;
              cur_group_r <= '0;
            end else begin
              base_pos_r <= base_pos_r + 1'b1;
            end

            if (byte_cnt_r == cur_len2_r - 1'b1) begin
              if (base_pos_r != 3'd7)
                seq2_groups_r[grp_cnt_r] <= {cur_group_r[13:0], enc_base} << (2 * (7 - base_pos_r));
              byte_cnt_r  <= '0;
            end else begin
              byte_cnt_r <= byte_cnt_r + 1'b1;
            end
          end
        end

        // ------------------------------------------------------------------
        S_ASSIGN: begin
          // Nothing to latch here; output logic below drives job_* signals
        end

        default:;
      endcase
    end
  end

  // ---------------------------------------------------------------------------
  // Output: Assign job to first idle aligner (combinational priority encoder)
  // ---------------------------------------------------------------------------
  logic [NUM_ALIGNERS_P-1:0] selected_aligner;

  // Priority encoder: select lowest-index idle aligner
  always_comb begin
    selected_aligner = '0;
    aligner_start    = '0;
    job_id           = cur_id_r;
    job_len1         = cur_len1_r;
    job_len2         = cur_len2_r;
    for (int i = 0; i < MAX_GROUPS_P; i++) begin
      job_seq1_groups[i] = seq1_groups_r[i];
      job_seq2_groups[i] = seq2_groups_r[i];
    end

    if (state_r == S_ASSIGN) begin
      // Find lowest-index idle aligner and issue one-cycle start pulse
      for (int i = NUM_ALIGNERS_P-1; i >= 0; i--) begin
        if (aligner_status[i]) begin
          selected_aligner = NUM_ALIGNERS_P'(i);
          aligner_start[i] = 1'b1;
        end
      end
    end
  end

endmodule

`default_nettype wire
