/***************************************************************************************
* Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
* Copyright (c) 2020-2021 Peng Cheng Laboratory
*
* XiangShan is licensed under Mulan PSL v2.
* You can use this software according to the terms and conditions of the Mulan PSL v2.
* You may obtain a copy of Mulan PSL v2 at:
*          http://license.coscl.org.cn/MulanPSL2
*
* THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
* EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
* MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
*
* See the Mulan PSL v2 for more details.
***************************************************************************************/

package xiangshan.cache

import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chisel3.util._
import utility._
import xiangshan._
import utils._
import freechips.rocketchip.tilelink._
import mem.{AddPipelineReg}
import huancun.{PaddrKey, PaddrField}

class RefillBufferEntry(edge: TLEdgeOut)(implicit p: Parameters) extends DCacheModule {
    val io = IO(new Bundle() {
        // to refill pipe
        val refill_pipe_req = DecoupledIO(new RefillPipeReq)
        // come from miss queue
        val miss_queue_req = Flipped(DecoupledIO(new RefillPipeReq))
        // forward information
        val forwardInfo = Output(new RefillBufferForwardIO)
        // tilelink D channel
        val mem_grant = Flipped(DecoupledIO(new TLBundleD(edge.bundle)))

        val state_idle = Output(Bool())
        val state_sleep = Output(Bool())
        val req_miss_id = Output(UInt(log2Up(cfg.nMissEntries).W))

        val entry_release_vec = Input(Vec(cfg.nMissEntries, Bool()))
        val mshr_paddr_vec = Input(Vec(cfg.nMissEntries, UInt(PAddrBits.W)))

        val sbuffer_data = Flipped(ValidIO(new SbufferToRefillBufferIO))
        val mshr_read = new DCacheBundle {
            val r = Flipped(ValidIO(new MissEntryReadRefillBufferIO))
            val resp = ValidIO(new RefillBufferToMissEntry)
        }
        val consumed = Output(Bool())
    })

    val req = RegInit(0.U.asTypeOf(new RefillPipeReq))
    val opcode_r = RegInit(TLMessages.Grant)

    val s_idle :: s_wait_second_beat :: s_sleep :: s_send_refill :: Nil = Enum(4)
    val refillBufferState = RegInit(s_idle)

    val s_recv_sbuffer = WireInit(true.B)

    when(refillBufferState =/= s_idle && io.sbuffer_data.valid && io.sbuffer_data.bits.mshr_id === req.miss_id) {
        s_recv_sbuffer := false.B
    }

    io.mshr_read.resp.valid := refillBufferState === s_sleep && io.mshr_read.r.valid && io.mshr_read.r.bits.mshr_id === req.miss_id
    io.mshr_read.resp.bits.data := req.data.asUInt
    
    io.state_idle := refillBufferState === s_idle
    io.state_sleep := refillBufferState === s_sleep
    io.req_miss_id := req.miss_id

    when(io.mem_grant.fire()) {
        req.miss_id := io.mem_grant.bits.source
        opcode_r := io.mem_grant.bits.opcode
        // req.addr := io.mshr_paddr_vec(io.mem_grant.bits.source)
        req.addr := io.mem_grant.bits.user.lift(PaddrKey).getOrElse(0.U)
        assert(io.mem_grant.bits.user.lift(PaddrKey).getOrElse(0.U) === io.mshr_paddr_vec(io.mem_grant.bits.source), "sourceD paddr is not equals to MSHR paddr")
        when(refillBufferState === s_idle) {
            when(io.mem_grant.bits.opcode === TLMessages.Grant) {
                refillBufferState := s_sleep
            }.otherwise {
                // TLMessages.GrantData
                refillBufferState := s_wait_second_beat
            }
            // req.data.asTypeOf(Vec(blockBytes/beatBytes, UInt(beatBits.W)))(0) := io.mem_grant.bits.data (bad chisel syntax)
            for (i <- 0 until beatRows) {
                val idx = i
                val grant_row = io.mem_grant.bits.data(rowBits * (i + 1) - 1, rowBits * i)
                req.data(idx) := grant_row
            }
        }.elsewhen(refillBufferState === s_wait_second_beat) {
            refillBufferState := s_sleep
            // req.data.asTypeOf(Vec(blockBytes/beatBytes, UInt(beatBits.W)))(1) := io.mem_grant.bits.data (bad chisel syntax)
            for (i <- 0 until beatRows) {
                val idx = (1 << log2Floor(beatRows)) + i
                val grant_row = io.mem_grant.bits.data(rowBits * (i + 1) - 1, rowBits * i)
                req.data(idx) := grant_row
            }
        }
    }
    io.mem_grant.ready := (refillBufferState === s_idle) || (refillBufferState === s_wait_second_beat && req.miss_id === io.mem_grant.bits.source)

    when(io.miss_queue_req.fire()) {
        req := io.miss_queue_req.bits
        // don't change data
        req.data := req.data
        // for now, just check the value, TODO: remove this
        assert(req.data.asUInt === io.miss_queue_req.bits.data.asUInt, "data in mshr should equal to data in refillBuffer, otherwise refillBuffer is wrong")
        refillBufferState := s_send_refill
    }
    io.miss_queue_req.ready := refillBufferState === s_sleep

    when(io.refill_pipe_req.fire()) {
        refillBufferState := s_idle
    }
    io.refill_pipe_req.valid := refillBufferState === s_send_refill
    io.refill_pipe_req.bits := req

    def mergePutData(old_data: UInt, new_data: UInt, wmask: UInt): UInt = {
        val full_wmask = FillInterleaved(8, wmask)
        (~full_wmask & old_data | full_wmask & new_data)
    }
    val new_data = Wire(Vec(blockRows, UInt(rowBits.W)))
    val new_mask = Wire(Vec(blockRows, UInt(rowBytes.W)))

    val consumed = WireInit(false.B)
    io.consumed := consumed

    for (i <- 0 until blockRows) {
        new_data(i) := io.sbuffer_data.bits.data(rowBits * (i + 1) - 1, rowBits * i)
        new_mask(i) := io.sbuffer_data.bits.mask(rowBytes * (i + 1) - 1, rowBytes * i)
    }

    val old_data = Wire(Vec(blockRows, UInt(rowBits.W)))
    // merge first beat(req.data) and second beat(mem_grant) with data from sbuffer
    for (i <- 0 until beatRows) {
        old_data(i) := req.data(i)
    }
    for (i <- 0 until beatRows) {
        val idx = (1 << log2Floor(beatRows)) + i
        val grant_row = io.mem_grant.bits.data(rowBits * (i + 1) - 1, rowBits * i)
        old_data(idx) := grant_row
    }

    when(refillBufferState === s_wait_second_beat && !s_recv_sbuffer) {
        // grantData situation
        when(io.mem_grant.fire()) {
            for (i <- 0 until blockRows) {
                val idx = i
                req.data(idx) := mergePutData(old_data(i), new_data(i), new_mask(i))
                consumed := true.B
            }
        }
    }

    when(refillBufferState === s_sleep && !s_recv_sbuffer) {
        // grant situation, write req.data using data from sbuffer
        assert(io.sbuffer_data.bits.mask.andR === true.B)
        for (i <- 0 until blockRows) {
            val idx = i
            val sbuffer_data = io.sbuffer_data.bits.data(rowBits * (i + 1) - 1, rowBits * i)
            req.data(idx) := sbuffer_data
            consumed := true.B
        }
    }

    // flush logic
    // if a mshr is going to release in next cycle, and no coming refill req from it(AMO), release this entry
    when(refillBufferState =/= s_idle && io.entry_release_vec(req.miss_id)) {
        refillBufferState := s_idle
    }

    // NOTE: if the response is Grant without data, do not forward until mshr give the whole data to this entry
    io.forwardInfo.apply(refillBufferState =/= s_idle, req.addr, 
                         req.data.asTypeOf(Vec(blockBytes/beatBytes, UInt(beatBits.W))), 
                         Mux(opcode_r === TLMessages.Grant, refillBufferState === s_send_refill, refillBufferState >= s_wait_second_beat),
                         Mux(opcode_r === TLMessages.Grant, refillBufferState === s_send_refill, refillBufferState >= s_sleep)
                         )
}

class RefillBuffer(edge: TLEdgeOut)(implicit p: Parameters) extends DCacheModule with HasCircularQueuePtrHelper{
    val io = IO(new Bundle() {
        // to refill pipe
        val refill_pipe_req = DecoupledIO(new RefillPipeReq)
        // dup to reduce fanout
        val refill_pipe_req_dup = Vec(nDupStatus, DecoupledIO(new RefillPipeReqCtrl))
        // come from miss queue
        val miss_queue_req = Flipped(DecoupledIO(new RefillPipeReq))
        // incoming forward req
        val forward = Vec(LoadPipelineWidth, new LduToRefillBufferForwardIO)
        // tilelink D channel
        val mem_grant = Flipped(DecoupledIO(new TLBundleD(edge.bundle)))

        val entry_release_vec = Input(Vec(cfg.nMissEntries, Bool()))
        val mshr_paddr_vec = Input(Vec(cfg.nMissEntries, UInt(PAddrBits.W)))
        
        val sbuffer_data = Flipped(DecoupledIO(new SbufferToRefillBufferIO))
        val mshr_read = new DCacheBundle {
            val r = Flipped(ValidIO(new MissEntryReadRefillBufferIO))
            val resp = ValidIO(new RefillBufferToMissEntry)
        }
    })

    io := DontCare

    val entries = Seq.fill(DcacheRefillBufferSize)(Module(new RefillBufferEntry(edge)))

    val forwardInfo_vec = VecInit(entries.map(_.io.forwardInfo))
    val req_miss_id_vec = VecInit(entries.map(_.io.req_miss_id))
    val state_idle_vec = VecInit(entries.map(_.io.state_idle))
    val state_sleep_vec = VecInit(entries.map(_.io.state_sleep))

    val enqIdx = PriorityEncoder(state_idle_vec)
    val lastEnqIdx = RegInit(0.U(log2Ceil(DcacheRefillBufferSize).W))
    val lastEnqIdxValid = RegInit(false.B)
    val realEnqIdx = Mux(lastEnqIdxValid, lastEnqIdx, enqIdx)

    val (_, _, refill_done, _) = edge.count(io.mem_grant)
    
    entries.zipWithIndex.foreach {
        case (e, i) =>
            e.io.entry_release_vec := io.entry_release_vec
            e.io.mshr_paddr_vec := io.mshr_paddr_vec
            // incoming tilelink D 
            e.io.mem_grant.valid := false.B
            e.io.mem_grant.bits := DontCare
            when(realEnqIdx === i.U) {
                io.mem_grant.ready := e.io.mem_grant.ready
                e.io.mem_grant.valid := io.mem_grant.valid
                e.io.mem_grant.bits := io.mem_grant.bits
            }
            // incoming req from miss queue
            e.io.miss_queue_req.valid := false.B
            e.io.miss_queue_req.bits := DontCare
            when(io.miss_queue_req.bits.miss_id === req_miss_id_vec(i) && state_sleep_vec(i)) {
                io.miss_queue_req.ready := e.io.miss_queue_req.ready
                e.io.miss_queue_req.valid := io.miss_queue_req.valid
                e.io.miss_queue_req.bits := io.miss_queue_req.bits
            }
            e.io.sbuffer_data.valid := io.sbuffer_data.valid
            e.io.sbuffer_data.bits := io.sbuffer_data.bits
            e.io.mshr_read.r <> io.mshr_read.r
    }
    assert(PopCount((0 until DcacheRefillBufferSize).map(i => {state_sleep_vec(i) && io.miss_queue_req.bits.miss_id === req_miss_id_vec(i) && io.miss_queue_req.valid})) <= 1.U, "miss queue req should only match one entry")

    when(io.mem_grant.fire() && refill_done) {
        lastEnqIdxValid := false.B
    }.elsewhen(io.mem_grant.fire() && !refill_done) {
        lastEnqIdxValid := true.B
        lastEnqIdx := enqIdx
    }

    // outgoing req to refill pipe
    val out_refill_pipe_req = Wire(Decoupled(new RefillPipeReq))
    val out_refill_pipe_req_ctrl = Wire(Decoupled(new RefillPipeReqCtrl))
    rrArbiter(entries.map(_.io.refill_pipe_req), out_refill_pipe_req, Some("refill_pipe_req_rr_arb"))
    out_refill_pipe_req_ctrl.valid := out_refill_pipe_req.valid
    out_refill_pipe_req_ctrl.bits := out_refill_pipe_req.bits.getCtrl
    out_refill_pipe_req.ready := out_refill_pipe_req_ctrl.ready
    for (dup <- io.refill_pipe_req_dup) {
        AddPipelineReg(out_refill_pipe_req_ctrl, dup, false.B)
    }
    AddPipelineReg(out_refill_pipe_req, io.refill_pipe_req, false.B)

    // forward logic
    (0 until LoadPipelineWidth).map(i => {
        val req_valid = io.forward(i).valid
        val paddr = io.forward(i).paddr

        val (forward_valid_vec, forwardData_vec) = forwardInfo_vec.map{info => info.forward(req_valid, paddr)}.unzip
        io.forward(i) := DontCare
        io.forward(i).forward_refillBuffer := Cat(forward_valid_vec).orR
        io.forward(i).forwardData := ParallelMux(forward_valid_vec zip forwardData_vec)
        assert(!io.forward(i).forward_refillBuffer || (PopCount(forward_valid_vec) === 0.U || PopCount(forward_valid_vec) === 1.U), s"port{$i} can not forward 2 or more entries in refillBuffer")
        // io.forward(i).forward_data := forwardData_vec.zip(forward_valid_vec).reduce(Mux(_._2, _._1, 0.U) | Mux(_._2, _._1, 0.U))
    })
    io.sbuffer_data.ready := VecInit(entries.map(_.io.consumed)).asUInt.orR

    assert(RegNext(PopCount(entries.map(_.io.consumed)) <= 1.U))

    io.mshr_read.resp.valid     := VecInit(entries.map(_.io.mshr_read.resp.valid)).asUInt.orR
    io.mshr_read.resp.bits.data := ParallelOR((entries.map(_.io.mshr_read.resp)).map{case (resp) => {
        Fill(resp.bits.data.getWidth, resp.valid.asUInt) & resp.bits.data
    }})

    assert(RegNext(PopCount(entries.map(_.io.mshr_read.resp.valid)) <= 1.U))

    // perf 
    val validCount = PopCount(VecInit(state_idle_vec.map(!_)))
    QueuePerf(DcacheRefillBufferSize, validCount, validCount === DcacheRefillBufferSize.U)
    XSPerfAccumulate("forward_refillBuffer", PopCount((0 until LoadPipelineWidth).map(i => io.forward(i).forward_refillBuffer)))
    XSPerfAccumulate("refill_buffer_not_ready", !io.mem_grant.ready)
    XSPerfAccumulate("tl_d_user_paddr_not_zero", io.mem_grant.fire() && (io.mem_grant.bits.user.lift(PaddrKey).getOrElse(0.U) =/= 0.U))
}
