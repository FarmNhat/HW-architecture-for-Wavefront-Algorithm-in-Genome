// =============================================================================
// backtrace_ctrl.sv
// WFA Accelerator — Backtrace Controller FSM
// =============================================================================
// FSM truy vết ngược từ (score_opt, k_target) về (0, 0) theo bảng origin
// trong Backtrace RAM, sinh compact CIGAR (RLE format).
//
// FSM States (từ spec Section 5.5):
//   BT_INIT  : khởi tạo s=score_opt, k=k_target, mat=M
//   BT_READ  : phát read request tới Backtrace RAM (1-cycle latency)
//   BT_WAIT  : chờ RAM output (registered read)
//   BT_PROC  : xử lý origin, cập nhật s/k/mat, emit CIGAR op
//   BT_DONE  : xuất compact CIGAR, báo done
//
// CIGAR RLE format:
//   64-bit = 8 runs × 8 bits/run
//   Each run: [7:4] = op-type (MATCH=0, MISMATCH=1, INS=2, DEL=3, EMPTY=15)
//             [3:0] = count - 1 (count = 1..16)
//   Runs packed MSB-first (run[0] = first op in sequence)
//
// Backtrace RAM có registered read: đọc ở cycle T, dữ liệu có ở cycle T+1.
// Mỗi word RAM = 40-bit = 8 cells × 5-bit origin.
// Cell k trong cột s ở offset = (k + K_MAX) % P_SUBMODULES trong word.
// Word address = s (theo address_decoder.sv).
// =============================================================================

//
`include "pkg_wfa_params.sv"

module backtrace_ctrl
  import wfa_pkg::*;
#(
  parameter int S_WIDTH_P   = S_WIDTH,
  parameter int K_WIDTH_P   = K_WIDTH,
  parameter int K_MAX_P     = K_MAX,
  parameter int X_PEN_P     = X_PEN,
  parameter int O_PEN_P     = O_PEN,
  parameter int E_PEN_P     = E_PEN
) (
  input  logic                           clk,
  input  logic                           rst_n,

  // -------------------------------------------------------------------------
  // Control
  // -------------------------------------------------------------------------
  input  logic                           start,       // Begin backtrace
  output logic                           done,        // Backtrace complete

  // -------------------------------------------------------------------------
  // Starting point
  // -------------------------------------------------------------------------
  input  logic [S_WIDTH_P-1:0]           score_opt,   // optimal score (final s)
  input  logic signed [K_WIDTH_P-1:0]   k_target,    // target diagonal (n1-n2)

  // -------------------------------------------------------------------------
  // Backtrace RAM interface (read only during backtrace)
  // -------------------------------------------------------------------------
  output logic                           ram_rd_en,   // Read enable
  output logic [$clog2(BT_RAM_DEPTH)-1:0] ram_rd_addr, // Read address
  input  logic [BT_RAM_WIDTH-1:0]       ram_rd_data, // Read data (1-cycle latency)

  // -------------------------------------------------------------------------
  // Output
  // -------------------------------------------------------------------------
  output logic [CIGAR_WIDTH-1:0]         compact_cigar // 64-bit RLE CIGAR
);

  // ---------------------------------------------------------------------------
  // Local types
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    BT_IDLE  = 3'd0,
    BT_INIT  = 3'd1,
    BT_READ  = 3'd2,
    BT_WAIT  = 3'd3,
    BT_PROC  = 3'd4,
    BT_DONE  = 3'd5
  } bt_state_t;

  // Current matrix: M, I, or D
  typedef enum logic [1:0] {
    MAT_M = 2'd0,
    MAT_I = 2'd1,
    MAT_D = 2'd2
  } mat_t;

  // CIGAR op codes matching wfa_pkg constants
  localparam logic [3:0] OP_MATCH    = CIGAR_MATCH;    // 4'b0000
  localparam logic [3:0] OP_MISMATCH = CIGAR_MISMATCH; // 4'b0001
  localparam logic [3:0] OP_INS      = CIGAR_INS;      // 4'b0010
  localparam logic [3:0] OP_DEL      = CIGAR_DEL;      // 4'b0011
  localparam logic [3:0] OP_EMPTY    = CIGAR_EMPTY;    // 4'b1111

  // ---------------------------------------------------------------------------
  // Registers
  // ---------------------------------------------------------------------------
  bt_state_t                    state_r;
  logic [S_WIDTH_P-1:0]         s_r;         // current score
  logic signed [K_WIDTH_P-1:0]  k_r;         // current diagonal
  mat_t                         mat_r;        // current matrix (M/I/D)

  // RAM address decoder
  logic [$clog2(BT_RAM_DEPTH)-1:0] addr_r;
  logic                             addr_valid_r;

  // Captured RAM word (registered, since BRAM has 1-cycle latency)
  logic [BT_RAM_WIDTH-1:0] ram_word_r;

  // Extract 5-bit origin for current k from the 40-bit RAM word
  // Cell index within word = (k + K_MAX) % P_SUBMODULES
  logic [2:0] cell_idx;
  logic [4:0] origin_cell; // 5-bit origin for current (s, k)

  always_comb begin
    cell_idx   = 3'((k_r + K_MAX_P) % P_SUBMODULES);
    origin_cell = ram_word_r[cell_idx * 5 +: 5];
  end

  // Parse origin fields
  logic       i_origin;  // [0]: 0=from M, 1=from I
  logic       d_origin;  // [1]: 0=from M, 1=from D
  logic [2:0] m_origin;  // [4:2]: 000=MM, 001=I, 010=D

  always_comb begin
    i_origin = origin_cell[0];
    d_origin = origin_cell[1];
    m_origin = origin_cell[4:2];
  end

  // ---------------------------------------------------------------------------
  // Compact CIGAR RLE assembly
  // We build the CIGAR in reverse (since backtrace goes backward),
  // then reverse at the end. Or: collect ops in a buffer, reverse, RLE encode.
  //
  // Simpler approach: accumulate ops in a shift register (LSB = most recent).
  // Max ops = SEQ_LEN_MAX * 2 but we limit to first 32 for 64-bit output.
  // Then we RLE-encode at the end.
  //
  // Buffer: store up to 64 raw ops (2-bit each), then RLE-encode to 64-bit.
  // ---------------------------------------------------------------------------
  localparam int MAX_OPS = SEQ_LEN_MAX * 2;  // generous buffer
  localparam int OP_BUF  = 128;              // buffer size

  logic [1:0] op_buf_r [0:OP_BUF-1];  // raw ops buffer (index 0 = first op)
  logic [$clog2(OP_BUF)-1:0] op_cnt_r; // number of ops stored
  logic [1:0] cur_op; // op being emitted this cycle

  // Op codes for buffer (2-bit)
  localparam logic [1:0] RAW_MATCH    = 2'b11;
  localparam logic [1:0] RAW_MISMATCH = 2'b00;
  localparam logic [1:0] RAW_INS      = 2'b01;
  localparam logic [1:0] RAW_DEL      = 2'b10;

  // ---------------------------------------------------------------------------
  // Address Decoder instantiation
  // ---------------------------------------------------------------------------
  logic [$clog2(BT_RAM_DEPTH)-1:0] dec_addr;
  logic                             dec_valid;

  address_decoder #(.RAM_DEPTH_P(BT_RAM_DEPTH), .S_WIDTH_P(S_WIDTH_P)) u_addr_dec (
    .score_in  (s_r),
    .ram_addr  (dec_addr),
    .addr_valid(dec_valid)
  );

  // ---------------------------------------------------------------------------
  // FSM
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state_r      <= BT_IDLE;
      done         <= 1'b0;
      s_r          <= '0;
      k_r          <= '0;
      mat_r        <= MAT_M;
      ram_rd_en    <= 1'b0;
      ram_rd_addr  <= '0;
      ram_word_r   <= '0;
      op_cnt_r     <= '0;
      compact_cigar <= '1; // all EMPTY runs
      for (int i = 0; i < OP_BUF; i++) op_buf_r[i] <= RAW_MATCH;
    end else begin
      done      <= 1'b0;
      ram_rd_en <= 1'b0;

      case (state_r)
        // ----------------------------------------------------------------
        BT_IDLE: begin
          if (start) begin
            s_r    <= score_opt;
            k_r    <= k_target;
            mat_r  <= MAT_M;
            op_cnt_r <= '0;
            state_r <= BT_INIT;
          end
        end

        // ----------------------------------------------------------------
        // BT_INIT: Check termination condition
        BT_INIT: begin
          if (s_r == '0 && k_r == '0 && mat_r == MAT_M) begin
            // Reached origin → done, encode CIGAR
            state_r <= BT_DONE;
          end else begin
            // Issue RAM read for current s
            ram_rd_en   <= 1'b1;
            ram_rd_addr <= dec_addr;
            state_r     <= BT_WAIT;
          end
        end

        // ----------------------------------------------------------------
        // BT_WAIT: Wait 1 cycle for registered BRAM output
        BT_WAIT: begin
          state_r <= BT_PROC;
        end

        // ----------------------------------------------------------------
        // BT_PROC: Capture RAM data and process origin
        BT_PROC: begin
          ram_word_r <= ram_rd_data; // latch the registered output

          case (mat_r)
            // --------------------------------------------------------------
            MAT_M: begin
              case (m_origin)
                M_ORIG_MM: begin
                  // Came from mismatch: emit X, go back x score
                  if (op_cnt_r < OP_BUF)
                    op_buf_r[op_cnt_r] <= RAW_MISMATCH;
                  op_cnt_r <= op_cnt_r + 1'b1;
                  s_r      <= s_r - S_WIDTH_P'(X_PEN_P);
                  // mat stays M, k stays
                  state_r  <= BT_INIT;
                end
                M_ORIG_I: begin
                  // Came from I: switch to I without emitting, no s/k change
                  mat_r   <= MAT_I;
                  state_r <= BT_INIT;
                end
                M_ORIG_D: begin
                  // Came from D: switch to D
                  mat_r   <= MAT_D;
                  state_r <= BT_INIT;
                end
                default: state_r <= BT_DONE; // safety
              endcase
            end

            // --------------------------------------------------------------
            MAT_I: begin
              // Emit insertion
              if (op_cnt_r < OP_BUF)
                op_buf_r[op_cnt_r] <= RAW_INS;
              op_cnt_r <= op_cnt_r + 1'b1;

              if (i_origin == ORIG_FROM_M) begin
                // Came from M (gap open): go back o+e score, k-1
                s_r     <= s_r - S_WIDTH_P'(O_PEN_P + E_PEN_P);
                k_r     <= k_r - K_WIDTH_P'(1);
                mat_r   <= MAT_M;
              end else begin
                // Came from I (gap extend): go back e score, k-1
                s_r     <= s_r - S_WIDTH_P'(E_PEN_P);
                k_r     <= k_r - K_WIDTH_P'(1);
                // mat stays I
              end
              state_r <= BT_INIT;
            end

            // --------------------------------------------------------------
            MAT_D: begin
              // Emit deletion
              if (op_cnt_r < OP_BUF)
                op_buf_r[op_cnt_r] <= RAW_DEL;
              op_cnt_r <= op_cnt_r + 1'b1;

              if (d_origin == ORIG_FROM_M) begin
                // Came from M (gap open): go back o+e score, k+1
                s_r     <= s_r - S_WIDTH_P'(O_PEN_P + E_PEN_P);
                k_r     <= k_r + K_WIDTH_P'(1);
                mat_r   <= MAT_M;
              end else begin
                // Came from D (gap extend): go back e score, k+1
                s_r     <= s_r - S_WIDTH_P'(E_PEN_P);
                k_r     <= k_r + K_WIDTH_P'(1);
                // mat stays D
              end
              state_r <= BT_INIT;
            end

            default: state_r <= BT_DONE;
          endcase
        end

        // ----------------------------------------------------------------
        // BT_DONE: RLE-encode the raw op buffer, output compact CIGAR
        BT_DONE: begin
          // The ops in op_buf_r are in REVERSE order (backtrace goes backward).
          // We need to output them in forward order.
          // RLE encode: scan from op_cnt_r-1 down to 0, accumulate runs.
          begin
            logic [63:0] cigar_out;
            logic [3:0]  run_op;
            logic [3:0]  run_cnt; // 1..16 → stored as 0..15
            logic [2:0]  run_idx; // index into 8 RLE slots (0..7)
            logic [1:0]  prev_op;
            logic        first_op;

            cigar_out = 64'hFFFF_FFFF_FFFF_FFFF; // all EMPTY
            run_op    = OP_EMPTY;
            run_cnt   = 4'd0;
            run_idx   = 3'd0;
            first_op  = 1'b1;
            prev_op   = 2'bxx;

            // Scan backward through op_buf (reverse to get forward order)
            for (int i = OP_BUF-1; i >= 0; i--) begin
              if (i < op_cnt_r) begin
                // This is a valid op (counting from op_cnt_r-1 down = forward order)
                // Actually: op_buf_r[op_cnt_r-1] is the LAST emitted = earliest op
                // op_buf_r[0] is the FIRST emitted = latest in sequence
                // So forward order = scan from [op_cnt_r-1] down to [0]
                logic [1:0] this_op;
                this_op = op_buf_r[op_cnt_r - 1 - i[$clog2(OP_BUF)-1:0]];

                // Map 2-bit raw to 4-bit CIGAR op-type
                case (this_op)
                  RAW_MATCH:    run_op = OP_MATCH;
                  RAW_MISMATCH: run_op = OP_MISMATCH;
                  RAW_INS:      run_op = OP_INS;
                  RAW_DEL:      run_op = OP_DEL;
                  default:      run_op = OP_EMPTY;
                endcase

                if (first_op) begin
                  // Start first run
                  prev_op  = this_op;
                  run_cnt  = 4'd0;
                  first_op = 1'b0;
                end else if (this_op == prev_op && run_cnt < 4'd15) begin
                  // Continue run
                  run_cnt = run_cnt + 4'd1;
                end else begin
                  // Flush current run to cigar_out
                  if (run_idx < 8) begin
                    cigar_out[63 - run_idx*8 -: 8] = {
                      (prev_op == RAW_MATCH)    ? OP_MATCH    :
                      (prev_op == RAW_MISMATCH) ? OP_MISMATCH :
                      (prev_op == RAW_INS)      ? OP_INS      : OP_DEL,
                      run_cnt
                    };
                    run_idx = run_idx + 3'd1;
                  end
                  // Start new run
                  prev_op = this_op;
                  run_cnt = 4'd0;
                end
              end
            end

            // Flush last run
            if (!first_op && run_idx < 8) begin
              cigar_out[63 - run_idx*8 -: 8] = {
                (prev_op == RAW_MATCH)    ? OP_MATCH    :
                (prev_op == RAW_MISMATCH) ? OP_MISMATCH :
                (prev_op == RAW_INS)      ? OP_INS      : OP_DEL,
                run_cnt
              };
            end

            compact_cigar <= cigar_out;
          end

          done    <= 1'b1;
          state_r <= BT_IDLE;
        end

        default: state_r <= BT_IDLE;
      endcase
    end
  end

endmodule

`default_nettype wire
