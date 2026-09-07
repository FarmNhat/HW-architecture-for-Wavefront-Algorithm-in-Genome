# WFA-FPGA Accelerator — Triển Khai Lại Bằng RTL

Triển khai lại (RTL, SystemVerilog) kiến trúc bộ tăng tốc FPGA cho thuật toán
**Wavefront Alignment (WFA)** dùng trong căn chỉnh cặp trình tự gen (genomic
pairwise alignment).

> **Tham chiếu gốc:** A. Haghi, S. Marco-Sola, L. Alvarez, D. Diamantopoulos,
> C. Hagleitner, M. Moreto, *"An FPGA Accelerator of the Wavefront Algorithm
> for Genomics Pairwise Alignment"*, 2021 31st International Conference on
> Field-Programmable Logic and Applications (FPL), DOI: 10.1109/FPL53798.2021.00033.

---

## 1. Tổng quan

Hệ thống gồm 3 tầng chính chạy trên FPGA, giao tiếp với CPU qua kênh dữ liệu
byte-stream:

1. **Extractor** — tiếp nhận trình tự DNA thô (ASCII), nén 2-bit/base, đóng
   gói theo nhóm 8 base, phân công cặp trình tự cho lõi Aligner đang rảnh.
2. **N x Aligner Core** — mỗi lõi chạy độc lập thuật toán WFA
   (extend -> compute -> backtrace) cho 1 cặp trình tự, N lõi chạy song song.
3. **Collector** — thu thập kết quả (compact CIGAR) từ tất cả Aligner, đóng
   gói thành từ 128-byte, xuất ra ngoài theo lịch trọng tài round-robin.

Tài liệu đặc tả chi tiết (danh sách tham số, interface từng module, công thức
toán học) nằm trong [`WFA_FPGA_RTL_SPEC.md`](./WFA_FPGA_RTL_SPEC.md).

---

## 2. Cấu trúc repo (đề xuất)

```
.
├── README.md                      # file này
├── WFA_FPGA_RTL_SPEC.md           # đặc tả thiết kế chi tiết cho agent/kỹ sư RTL
├── rtl/
│   ├── pkg_wfa_params.sv          # package tham số dùng chung
│   ├── base_encoder.sv            # mã hóa ASCII -> 2-bit (A,C,G,T)
│   ├── extractor_assign.sv        # đóng gói nhóm 8-base + dispatch job
│   ├── extractor_top.sv
│   ├── extend_submodule.sv        # so sánh 8-base song song, extend()
│   ├── compute_submodule.sv       # công thức I/D/M (Equation 1)
│   ├── wavefront_window.sv        # bộ nhớ cửa sổ trượt (lag buffer)
│   ├── backtrace_ram.sv           # RAM lưu origin 5-bit/ô
│   ├── address_decoder.sv         # giải mã địa chỉ backtrace
│   ├── backtrace_ctrl.sv          # FSM truy vết ngược + nén RLE CIGAR
│   ├── aligner_top.sv             # ghép 1 lõi Aligner hoàn chỉnh
│   ├── collector_packer.sv        # gom 8 kết quả -> 1 từ 128-byte
│   ├── collector_scheduler.sv     # trọng tài round-robin
│   ├── collector_top.sv
│   └── wfa_accelerator_top.sv     # top-level, generate N Aligner
├── tb/
│   ├── tb_extend_submodule.sv
│   ├── tb_compute_submodule.sv
│   ├── tb_backtrace_ctrl.sv
│   ├── tb_aligner_top.sv
│   └── tb_wfa_accelerator_top.sv
└── scripts/
    └── gen_reference_vectors.py   # mô hình phần mềm sinh test-vector đối chiếu
```

---

## 3. Tham số thiết kế chính

| Tham số | Ý nghĩa | Giá trị mặc định |
|---|---|---|
| `X_PEN` | mismatch penalty | 4 |
| `O_PEN` | gap-open penalty | 6 |
| `E_PEN` | gap-extend penalty | 2 |
| `K_MAX` | giới hạn đường chéo (-K..+K) | 32 |
| `SEQ_LEN_MAX` | độ dài trình tự tối đa | 150 |
| `P_SUBMODULES` | số cặp Extend/Compute song song | 8 |
| `BLOCK_SIZE` | số base/nhóm đóng gói | 8 |
| `NUM_ALIGNERS` | số lõi Aligner trên 1 FPGA | 64 |
| `BT_RAM_WIDTH` | độ rộng Backtrace RAM | 40 bit |
| `BT_RAM_DEPTH` | độ sâu Backtrace RAM | 250 từ |

Độ rộng cửa sổ wavefront: `window_I = E_PEN`, `window_D = E_PEN`,
`window_M = max(X_PEN, O_PEN + E_PEN)`.

---

## 4. Sơ đồ kiến trúc

### 4.1 Sơ đồ khối toàn hệ thống

```mermaid
graph TB
    subgraph TOP["wfa_accelerator_top"]
        direction TB
        subgraph IN["Input Interface"]
            DATA_IN["data_in [7:0]<br/>data_in_valid / ready"]
        end
        subgraph EXT["Extractor Top (extractor_top.sv)"]
            FSM_EXT["Extractor FSM & Parser"]
            ENC["Base Encoder (base_encoder.sv)<br/>ASCII → 2-bit (A,C,G,T)"]
            PACK["Group Packer<br/>(8 bases / 16-bit word)"]
            DIST["Job Dispatcher<br/>(extractor_assign.sv)"]
            FSM_EXT --> ENC --> PACK --> DIST
        end
        subgraph ALIGNERS["N × Aligner Core (aligner_top.sv)"]
            direction TB
            subgraph CORE0["Aligner Core #0"]
                CTRL0["Aligner FSM Controller"]
                CMP0["Compute Array (P=8 Submodules)<br/>compute_submodule.sv<br/>Equation 1: M, I, D"]
                EXT0["Extend Submodule<br/>extend_submodule.sv<br/>8-base Parallel Comparator"]
                WIN0["Wavefront Window<br/>wavefront_window.sv<br/>Lag Buffers (o+e, e, x)"]
                BT0["Backtrace Unit<br/>backtrace_ctrl.sv + backtrace_ram.sv<br/>5-bit Origin & CIGAR RLE"]

                CTRL0 <--> WIN0
                CTRL0 <--> CMP0
                CTRL0 <--> EXT0
                CMP0 <--> WIN0
                CTRL0 --> BT0
            end
            subgraph COREN["Aligner Core #(N-1)"]
                MORE["... Các Aligner song song độc lập ..."]
            end
        end
        subgraph COL["Collector Top (collector_top.sv)"]
            direction TB
            P0["Collector Packer #0<br/>collector_packer.sv<br/>(Buffer 8 results)"]
            PN["Collector Packer #(N-1)<br/>collector_packer.sv"]
            SCHED["Round-Robin Scheduler<br/>collector_scheduler.sv"]

            P0 --> SCHED
            PN --> SCHED
        end
        subgraph OUT["Output Interface"]
            DATA_OUT["data_out [127:0]<br/>{Job_ID, Score, CIGAR_64, Pad}<br/>data_out_valid"]
        end
    end
    DATA_IN --> EXT
    DIST -- "job_id, lengths, packed_seqs, start" --> CORE0
    DIST -- "dispatch" --> COREN
    CORE0 -- "aligner_status_idle" --> DIST
    COREN -- "aligner_status_idle" --> DIST
    CORE0 -- "result_word [127:0]" --> P0
    COREN -- "result_word [127:0]" --> PN
    SCHED --> DATA_OUT
```

### 4.2 Chi tiết bên trong 1 lõi Aligner

```mermaid
graph LR
    subgraph ALIGNER_DETAIL["Chi tiết 1 lõi Aligner (aligner_top.sv)"]
        direction TB
        subgraph MEM_HIST["1. Wavefront Window (Bộ nhớ trượt)"]
            BUF_M["M Buffer (Depth = o+e+1)<br/>M[s - (o+e), k±1], M[s-x, k]"]
            BUF_I["I Buffer (Depth = e+1)<br/>I[s - e, k-1]"]
            BUF_D["D Buffer (Depth = e+1)<br/>D[s - e, k+1]"]
        end
        subgraph CALC["2. Compute & Extend Datapath"]
            subgraph COMP_ARR["Compute Submodule Array (P = 8 copies)"]
                C0["Compute #0"]
                C1["Compute #1"]
                CDOTS["..."]
                C7["Compute #7"]
            end
            subgraph EXT_UNIT["Extend Submodule (extend_submodule.sv)"]
                SHIFTER["32-bit Window Shifter<br/>(h_bit_off, v_bit_off)"]
                COMP8["8-wide Parallel Comparator<br/>(2-bit XOR match)"]
                PRIO["Priority Encoder<br/>(Đếm số match liên tục)"]
                SHIFTER --> COMP8 --> PRIO
            end
        end
        subgraph TRACE["3. Backtrace & CIGAR Engine"]
            BRAM["Backtrace Dual-Port RAM<br/>(40-bit word: 8 × 5-bit origins)"]
            ADDR_DEC["Address Decoder (address_decoder.sv)<br/>Linear Addr = s * K_WORDS + col"]
            BT_FSM["Backtrace FSM (backtrace_ctrl.sv)<br/>Truy vết từ (Score, k_target) → (0,0)"]
            RLE["RLE CIGAR Packer<br/>(64-bit compact CIGAR)"]

            ADDR_DEC --> BRAM
            BT_FSM <--> BRAM
            BT_FSM --> RLE
        end
        MEM_HIST -- "Đọc lịch sử lag" --> COMP_ARR
        COMP_ARR -- "m_pre (trước extend)" --> EXT_UNIT
        EXT_UNIT -- "offset_out (sau trượt)" --> MEM_HIST
        COMP_ARR -- "5-bit origin {M, D, I}" --> BRAM
    end
```

### 4.3 Sơ đồ tuần tự xử lý dữ liệu

```mermaid
sequenceDiagram
    autonumber
    actor Host as Host / Testbench
    participant Ext as Extractor Top
    participant Aligner as Aligner Core
    participant Win as Wavefront Window
    participant ExtMod as Extend Submodule
    participant BT as Backtrace Unit
    participant Col as Collector Top
    Host->>Ext: Gửi luồng byte DNA qua data_in (Job ID, Len1, Len2, Seqs)
    Ext->>Ext: Base Encoder chuyển ASCII → 2-bit, gom thành các Group 16-bit
    Ext->>Aligner: Kích hoạt `start` khi Aligner báo `status_idle = 1`

    rect rgb(240, 245, 255)
        note over Aligner, ExtMod: Pha s = 0 (Khởi tạo)
        Aligner->>ExtMod: start extend(k=0, h=0)
        ExtMod-->>Aligner: done, trả về h0
        Aligner->>Win: Ghi M[0][0] = h0
    end
    rect rgb(245, 255, 240)
        note over Aligner, Win: Vòng lặp tính toán: s = 1, 2, ...
        loop Quét đường chéo k = -K_MAX .. +K_MAX (bước nhảy P=8)
            Aligner->>Win: Đọc các ô lùi: M[s-o-e], I[s-e], D[s-e], M[s-x]
            Aligner->>Aligner: 8 bộ Compute tính song song I, D, M_pre và Origin
            Aligner->>ExtMod: Trượt so khớp liên tục (Extend) cho từng k hợp lệ
            ExtMod-->>Aligner: Trả về offset mới
            Aligner->>Win: Cập nhật cột mới vào Wavefront Window
            Aligner->>BT: Ghi mã nguồn gốc (5-bit origin) vào Backtrace RAM
            Aligner->>Aligner: Kiểm tra tới đích: (k == k_target && h == len1 && v == len2)
        end
    end
    rect rgb(255, 250, 240)
        note over Aligner, BT: Pha Backtrace (Truy vết ngược)
        Aligner->>BT: Kích hoạt bt_start với score tối ưu
        BT->>BT: Đọc RAM ngược từ (score, k_target) về (0,0)
        BT->>BT: Nén Run-Length Encoding (RLE) thành 64-bit CIGAR
        BT-->>Aligner: bt_done, xuất bt_cigar
    end
    Aligner->>Col: Gửi result_word [127:0] qua Collector Packer
    Col->>Col: Round-Robin Scheduler chọn Aligner hoàn thành
    Col->>Host: Xuất data_out [127:0] kèm data_out_valid
```

### 4.4 Máy trạng thái FSM của 1 lõi Aligner

```mermaid
stateDiagram-v2
    [*] --> A_IDLE
    A_IDLE --> A_LOAD : start == 1
    A_LOAD --> A_INIT_S0 : Chốt chuỗi, tính k_target, reset pointer
    A_INIT_S0 --> A_INIT_WAIT : Phát xung ext_start (k=0, h=0)
    A_INIT_WAIT --> A_DONE_ALIGN : ext_done && chạm đích tại s=0 (Perfect Match)
    A_INIT_WAIT --> A_WR_WINDOW : ext_done && chưa chạm đích
    A_WR_WINDOW --> A_CHECK : Ghi cột wavefront và RAM

    A_CHECK --> A_DONE_ALIGN : Chạm đích tại (s, k_target)
    A_CHECK --> A_CPU_FALLBACK : s vượt ngưỡng chi phí K_MAX * (o+e)
    A_CHECK --> A_ADVANCE : Chưa chạm đích
    A_ADVANCE --> A_COMPUTE : s = s + 1, k_base = -K_MAX
    A_COMPUTE --> A_COMPUTE : Quét tiếp các nhóm P=8 đường chéo
    A_COMPUTE --> A_EXTEND_K : Đã quét xong toàn bộ k từ -K_MAX đến +K_MAX
    A_EXTEND_K --> A_EXTEND_WAIT : Phát xung ext_start cho đường chéo k hợp lệ
    A_EXTEND_WAIT --> A_EXTEND_K : ext_done, lưu offset_out, tăng k
    A_EXTEND_K --> A_WR_WINDOW : Tất cả các đường chéo đã extend xong
    A_DONE_ALIGN --> A_BACKTRACE : Kích hoạt bt_start
    A_BACKTRACE --> A_OUTPUT : bt_done == 1 (Truy vết xong CIGAR)
    A_CPU_FALLBACK --> A_OUTPUT : Gán cờ fallback & score sentinel 0xFFFF
    A_OUTPUT --> A_IDLE : Phát xung result_valid, trả về trạng thái rảnh
```

### 4.5 Sơ đồ component (PlantUML)

Nếu dùng PlantUML (VS Code extension, IntelliJ, hoặc plantuml.com), tạo file
`docs/architecture.puml` với nội dung sau:

```plantuml
@startuml WFA_FPGA_Accelerator_Architecture
!theme plain
skinparam componentStyle uml2
skinparam packageStyle rectangle
title Architecture of WFA FPGA Accelerator (Haghi et al., FPL 2021)

package "wfa_accelerator_top" {
  interface "data_in [7:0]" as DIN
  interface "data_out [127:0]" as DOUT

  package "Extractor Unit (extractor_top.sv)" {
    [Extractor FSM] as ExtFSM
    [Base Encoder (base_encoder.sv)] as BaseEnc
    [Group Packer (16-bit / 8-base)] as GrpPack
    [Dispatcher (extractor_assign.sv)] as Dispatcher
    ExtFSM --> BaseEnc
    BaseEnc --> GrpPack
    GrpPack --> Dispatcher
  }

  package "Parallel Aligner Array" {
    package "Aligner Core #0 (aligner_top.sv)" as Core0 {
      [Aligner Controller FSM] as FSM0

      package "Wavefront Window" {
        database "M Buffer (Depth=o+e+1)" as MBuf
        database "I Buffer (Depth=e+1)" as IBuf
        database "D Buffer (Depth=e+1)" as DBuf
      }

      package "Compute Engine" {
        [8x Compute Submodules\n(compute_submodule.sv)] as CompSub
      }
      package "Extend Engine" {
        [Extend Submodule\n(extend_submodule.sv)\n8-base Parallel Comp] as ExtSub
      }
      package "Backtrace Engine" {
        [Backtrace FSM\n(backtrace_ctrl.sv)] as BtFSM
        database "Backtrace Dual-Port RAM\n(backtrace_ram.sv)" as BtRAM
        [Address Decoder\n(address_decoder.sv)] as AddrDec
      }
    }
    package "Aligner Core #1 .. #(N-1)" as CoreN {
      [Parallel Instances] as OtherCores
    }
  }

  package "Collector Unit (collector_top.sv)" {
    [Collector Packer #0] as Pack0
    [Collector Packer #(N-1)] as PackN
    [Round-Robin Scheduler\n(collector_scheduler.sv)] as RRSched
    Pack0 --> RRSched
    PackN --> RRSched
  }

  DIN --> ExtFSM
  Dispatcher --> Core0 : job_id, seq_groups, start
  Dispatcher --> CoreN : job_id, seq_groups, start
  Core0 ..> Dispatcher : status_idle
  CoreN ..> Dispatcher : status_idle

  FSM0 --> CompSub
  CompSub --> ExtSub
  ExtSub --> MBuf
  MBuf --> CompSub : lag M[s-o-e], M[s-x]
  IBuf --> CompSub : lag I[s-e]
  DBuf --> CompSub : lag D[s-e]
  CompSub --> BtRAM : 5-bit Origin write
  FSM0 --> BtFSM : bt_start
  BtFSM <--> BtRAM : read reverse
  AddrDec --> BtRAM

  Core0 --> Pack0 : result_word [127:0]
  CoreN --> PackN : result_word [127:0]
  RRSched --> DOUT : result_valid
}
@enduml
```

---

## 5. Trạng thái dự án

| Hạng mục | Trạng thái |
|---|---|
| Đặc tả kiến trúc (`WFA_FPGA_RTL_SPEC.md`) | Hoàn thành |
| Sơ đồ UML/Mermaid/PlantUML | Hoàn thành (file này) |
| Mô hình phần mềm tham chiếu (Python) | Hoàn thành (`scripts/gen_reference_vectors.py`) |
| RTL các module (`rtl/*.sv`) | Chưa triển khai — xem prompt agent trong `WFA_FPGA_RTL_SPEC.md` |
| Testbench (`tb/*.sv`) | Chưa triển khai |
| Synthesis / timing closure (Vivado, Xilinx UltraScale+) | Chưa thực hiện |

> **Lưu ý:** các sơ đồ và đặc tả trong repo mô tả đúng tinh thần kiến trúc và
> công thức toán học trong bài báo FPL 2021, nhưng **chưa được đối chiếu với
> RTL/silicon thật** của nhóm tác giả (repo gốc: `gitlab.bsc.es/ahaghi/wfa_fpga_accelerator`).
> Các chi tiết vi mạch cụ thể (định dạng địa chỉ Backtrace RAM, độ sâu FIFO
> Scheduler...) là đề xuất thiết kế hợp lý dựa trên mô tả trong bài báo, cần
> được kiểm chứng bằng mô phỏng trước khi tổng hợp (synthesis).

---

## 6. Cách sử dụng (dự kiến)

```bash
# 1. Sinh test-vector tham chiếu từ mô hình phần mềm
python3 scripts/gen_reference_vectors.py --out tb/vectors/

# 2. Chạy mô phỏng (ví dụ dùng Verilator hoặc trình mô phỏng SystemVerilog)
#    (thay bằng công cụ mô phỏng bạn đang dùng)
verilator --binary -j 0 -sv rtl/*.sv tb/tb_wfa_accelerator_top.sv
./obj_dir/Vtb_wfa_accelerator_top

# 3. (Tương lai) Tổng hợp cho Xilinx UltraScale+ qua Vivado
vivado -mode batch -source scripts/build_bitstream.tcl
```

---

## 7. Tài liệu tham khảo

- A. Haghi et al., *"An FPGA Accelerator of the Wavefront Algorithm for
  Genomics Pairwise Alignment"*, FPL 2021.
- S. Marco-Sola, J. C. Moure, M. Moreto, A. Espinosa, *"Fast gap-affine
  pairwise alignment using the wavefront algorithm"*, Bioinformatics, 2020.
- Xem chi tiết đặc tả trong [`WFA_FPGA_RTL_SPEC.md`](./WFA_FPGA_RTL_SPEC.md).