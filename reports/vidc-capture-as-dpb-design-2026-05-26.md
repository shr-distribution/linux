# VIDC decoder: CAPTURE-buffers-as-DPB rework — design & staged plan (2026-05-26)

Goal: stop the internal-DPB + per-frame CPU memcpy. Make the vb2 CAPTURE
buffers BE the firmware DPB Y/C frames (zero-copy), like every mainline
stateful decoder. Precedent = **s5p-mfc v5** (Samsung, no IOMMU, reserved
memory bank — same constraints). Firmware DPB recycling model = webOS DDL
`ddl_decoder_dpb_transact` (`hw_mask`/`client_mask`).

## Why
1080p fails: internal DPB (~36 MB) + CAPTURE pool (~24 MB) both in the 61 MB
SMI pool → -ENOMEM. CAPTURE=DPB → only one set (~28 MB Y/C) + small MV pool →
fits. Also removes a per-frame memcpy and is mainline-correct.

## Key enabler (already true)
CAPTURE format is V4L2_PIX_FMT_NV12MT, sized exactly y_size+c_size = a DPB
slot (vidc_dec_get_framesize == vidc_dpb_calc_sizes). So a CAPTURE buffer is
byte-compatible with a DPB slot — no detile, no format change. The memcpy was
the only thing touching the tiled bytes. CAPTURE already allocates from the
SMI-bound core->dev (MMAP lands in SMI automatically).

## Confirmed finding
submit_frame pixel-cache loop reads dpb_y_dma_addr (encoder field) → currently
0 for the decoder post per-slot change, yet 720p decodes. So pixel-cache luma
bases are NOT load-bearing; DPB_LUMA[i] regs are. Fix the loop to dpb_cap_dma[i]
in the rework (neutral/beneficial).

## New ownership
- DPB Y (slot i)  = CAPTURE vb2 buf dma  -> DPB_LUMA[i]   = (cap_dma-fw)>>11
- DPB C (slot i)  = same CAPTURE buf + y_size -> DPB_CHROMA[i]
- DPB MV (slot i) = NEW internal SMI MV pool (dpb_count*ALIGN(mv_size,4K)) -> DPB_MV[i]
- vert_nb_mv / nb_ip = internal SMI (keep)
slot index == vb2 buffer index (s5p-mfc style); dpb_hw_mask bit i keyed by index.

## Flow change
- seq_done_work: DROP the vidc_init_buffers() call + 0xCC sentinel. Only set
  seq_parsed, stash y/c/mv sizes, publish MIN_BUFFERS_FOR_CAPTURE=min_dpb, emit
  SOURCE_CHANGE.
- CAPTURE start_streaming: enumerate queued CAPTURE bufs -> dpb_cap_dma[index];
  then reworked vidc_init_buffers programs DPB regs from dpb_cap_dma[] + MV pool
  and issues INIT_BUFFERS. (m2m has no per-queue reqbufs hook; start_streaming
  is the "buffers exist" point, under the queue lock.) Record dpb_cap_dma in a
  CAPTURE branch of vidc_dec_buf_init (1:1 with s5p-mfc buf_init).

## Display (zero-copy)
Replace vidc_copy_dpb_to_dst with vidc_lookup_dpb_buffer: disp_phys =
display_y_raw<<11, match dpb_cap_dma[i] -> slot/vb2 buf; vb2_buffer_done() THAT
buffer (payload y+c, restore frame-tag ts). All CAPTURE bufs are firmware-owned
from qbuf until a FRAME_DONE names their address (not "next dst"). device_run
no longer passes a meaningful per-frame dst; job_ready gates on src && (>=1 free
DPB slot, hw_mask!=0).

## Recycling (dpb_hw_mask)
- INIT: hw_mask = (1<<dpb_count)-1 (all free).
- display (MARK_BUSY): hw_mask &= ~(1<<slot).
- CAPTURE buf_queue (re-queue / MARK_FREE): hw_mask |= (1<<index).
- each FRAME_DATA writes CH0_DPB_RELEASE = hw_mask (already done); gate FRAME_DATA
  on hw_mask!=0 (writing 0 = fw error 125).

## Delete
internal Y/C alloc in vidc_init_buffers; vidc_copy_dpb_to_dst + call site; the
0xCC sentinel; decoder use of dpb_slot_*/dpb_y_* (KEEP for encoder recon, guard
by inst->decoder). Drop VB2_DMABUF on the CAPTURE queue (no-IOMMU: CAPTURE must
be SMI-backed MMAP, like s5p-mfc v5). free_buffers frees MV pool, not CAPTURE.

## Count
queue_setup CAPTURE: *num_buffers = max(req, min_dpb + >=4 headroom), <=32.
dpb_count = actual q->num_buffers. Reject < min_dpb at CAPTURE start_streaming.

## Risks + STAGED on-device validation (regression-gate 720p 50/50 each stage)
S1 MV-pool split (keep internal DPB+copy) -> 320/640/720 decode OK.
S2 defer INIT_BUFFERS to CAPTURE start_streaming (still internal) -> 720 OK.
S3 zero-copy (DPB<-cap_dma, fix pix-cache loop, drop memcpy, wire hw_mask, drop
   CAPTURE DMABUF) -> 320->640->720 each 50/50 + visually correct.
S4 1080p decodes (the goal).
S5 B-frame reorder churn + DECODER_CMD STOP drain to DPB_EMPTY (watch fw err 125,
   no-match lookup stalls).
Encoder untouched (guard all decoder changes by inst->decoder).

Full cited plan: agent report in session transcript 2026-05-26.
