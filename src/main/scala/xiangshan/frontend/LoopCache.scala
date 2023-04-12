package xiangshan.frontend

import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chisel3.util._
import utils._
import xiangshan._
import xiangshan.frontend.icache._
import xiangshan.backend.CtrlToFtqIO

class BpuBypassUpdate(implicit p: Parameters) extends XSBundle with LoopPredictorParams {
  val expected_loop_cnt = UInt(cntBits.W)
  val single_entry = new BranchPredictionBundle
  val last_stage_meta = UInt(MaxMetaLength.W)
  val last_stage_spec_info = new SpeculativeInfo
  val last_stage_ftb_entry = new FTBEntry
}

class BpuBypassIO(implicit p: Parameters) extends XSBundle {
  val BpuIn = Flipped(new BpuToFtqIO)
  val BpuOut = new BpuToFtqIO

  val update = Flipped(Valid(new BpuBypassUpdate))

  // redirect from ftq
  val redirect = Flipped(Valid(new FtqPtr))

  // debug purpose only
  val BpuPtr = Input(new FtqPtr)
}

class LoopCacheSpecInfo(implicit p: Parameters) extends XSBundle {
  val pc = UInt(VAddrBits.W)
  val hit_data = new LoopCacheSpecEntry
  val pd = new PredecodeWritebackBundle
}

class LoopCacheQuery(implicit p: Parameters) extends XSBundle with LoopPredictorParams {
  val pc = UInt(VAddrBits.W)
  val cfiValid = Bool()
  val target = UInt(VAddrBits.W)
  val cfiIndex = UInt(log2Ceil(PredictWidth).W)
  val ftqPtr = new FtqPtr

  val lpInfo = new xsLPpredInfo

  val isDouble = Bool()
  val isExit = Bool()

  val bpu_in = new BranchPredictionBundle
}

class LoopCacheResp(implicit p: Parameters) extends XSBundle {
  val entry = new LoopCacheSpecEntry
  val valids = Vec(LoopCacheSpecSize, Bool())
  val pd = Output(new PredecodeWritebackBundle)
}

class LoopCacheFenceBundle(implicit p: Parameters) extends XSBundle {
  val sfence_valid = Bool()
  val fencei_valid = Bool()
}

class LoopCacheNonSpecIO(implicit p: Parameters) extends XSBundle with HasCircularQueuePtrHelper {
  val req = Flipped(Decoupled(new LoopCacheQuery))
  val l0_hit = Output(Bool())
  val l0_redirect_scheduled = Output(Bool())
  val resp = Decoupled(new LoopCacheResp)

  val update = Flipped(Valid(new LoopCacheSpecInfo))

  val flush = Input(new LoopCacheFenceBundle)
  val redirectFlush = Input(Bool())

  val pd_valid = Output(Bool())
  val pd_data = Output(new PredecodeWritebackBundle)

  val toFtqRedirect = Valid(new Redirect)
  val toBypass = Valid(new BpuBypassUpdate)

  val last_stage_info = Flipped(Valid(new LastStageInfo))

  val flushFromBpuIfu = Flipped(new Bundle {
    // when ifu pipeline is not stalled,
    // a packet from bpu s3 can reach f1 at most
    val s2 = Valid(new FtqPtr)
    val s3 = Valid(new FtqPtr)
    val ifu = Valid(new FtqPtr)
    def shouldFlushBy(src: Valid[FtqPtr], idx_to_flush: FtqPtr) = {
      src.valid && !isAfter(src.bits, idx_to_flush)
    }
    def shouldFlushByStage2(idx: FtqPtr) = shouldFlushBy(s2, idx)
    def shouldFlushByStage3(idx: FtqPtr) = shouldFlushBy(s3, idx)
    def shouldFlushByIfu(idx: FtqPtr) = shouldFlushBy(ifu, idx)
  })
}

class LoopCacheNonSpecEntry(implicit p: Parameters) extends XSModule with HasBPUConst with LoopPredictorParams {
  val io = IO(new LoopCacheNonSpecIO)

  val LcHitThreshold: Int = 10
  val LcHandoverThreshold: Int = 10

  val cache_valid = RegInit(0.B)
  val cache_pc = Reg(UInt(VAddrBits.W))
  val cache_data = Reg(new LoopCacheSpecEntry)
  val cache_pd = Reg(new PredecodeWritebackBundle)

  val l0_fire = Wire(Bool())
  val l1_fire = Wire(Bool())
  val l2_fire = Wire(Bool())
  /*
  * l0: accept req and generate hit info
  * */
  val l0_hit = Wire(Bool())
  val l0_pc = Wire(UInt(VAddrBits.W))
  val l0_data = Wire(new LoopCacheSpecEntry)
  val l0_taken_pc = Wire(UInt(VAddrBits.W))
  val l0_is_exit = io.req.bits.lpInfo.isConfExitLoop
  val l0_isInterNumGT2 = io.req.bits.lpInfo.isInterNumGT2
  val l0_remainIterNum = 1000.U //io.req.bits.lpInfo.remainIterNum + 1.U // FIXME: provide data from loop predictor
  val l0_flush_by_bpu = io.flushFromBpuIfu.shouldFlushByStage2(io.req.bits.ftqPtr) || io.flushFromBpuIfu.shouldFlushByStage3(io.req.bits.ftqPtr)
  val l0_flush_by_ifu = io.flushFromBpuIfu.shouldFlushByIfu(io.req.bits.ftqPtr)
  val prev_hit = RegInit(0.B)
  val l0_pc_hit = VecInit(cache_data.instEntry.map(_.pc === l0_taken_pc))
  val l0_pc_hit_pos = OHToUInt(l0_pc_hit)
  val l0_bpu_resp = io.req.bits.bpu_in
  val l0_redirect_scheduled = Wire(Bool())
  val l0_isDouble = io.req.bits.isDouble
  val l0_isExit = io.req.bits.isExit

  val l0_redirect = WireInit(0.U.asTypeOf(new Redirect()))

  l0_redirect.ftqIdx := io.req.bits.ftqPtr
  // should flush all
  l0_redirect.ftqOffset := 0.U
  l0_redirect.level := RedirectLevel.flush

  val l0_redirect_cfiUpdate = l0_redirect.cfiUpdate
  l0_redirect_cfiUpdate.pc := l0_taken_pc
  l0_redirect_cfiUpdate.pd := cache_pd.pd(l0_pc_hit_pos)
  l0_redirect_cfiUpdate.predTaken := true.B
  l0_redirect_cfiUpdate.target := l0_taken_pc + Mux(cache_pd.pd(l0_pc_hit_pos).isRVC, 2.U, 4.U)
  l0_redirect_cfiUpdate.taken := false.B
  l0_redirect_cfiUpdate.isMisPred := true.B



  l0_pc := DontCare
  l0_data := DontCare
  l0_hit := false.B

  XSPerfAccumulate(f"loop_cache_query", io.req.fire)
  XSPerfAccumulate(f"loop_cache_query_hit", io.req.fire && l0_hit)

  val loop_lowerbound = WireInit(20.U)
  when (io.req.fire) {
    when (cache_valid && io.req.bits.pc === cache_pc && io.req.bits.cfiValid && (io.req.bits.target === cache_pc || io.req.bits.isExit) && io.req.bits.lpInfo.isConf && l0_remainIterNum > LcHitThreshold.U) {
      l0_hit := true.B
      l0_data := cache_data
      prev_hit := true.B
    } .otherwise {
      prev_hit := false.B
    }

  }
  io.l0_hit := l0_hit
  l0_taken_pc := io.req.bits.pc + Cat(io.req.bits.cfiIndex, 0.U.asTypeOf(UInt(1.W)))

  when (io.req.valid && io.req.ready && l0_hit && !prev_hit && !l0_flush_by_bpu && !l0_flush_by_ifu && !io.redirectFlush && !io.flush.sfence_valid && !io.flush.fencei_valid) {
    // we are at the start of a new loop
    io.l0_redirect_scheduled := true.B
    l0_redirect_scheduled := true.B
  } .otherwise {
    io.l0_redirect_scheduled := false.B
    l0_redirect_scheduled := false.B
  }

  val scheduled_redirect_valid = RegInit(0.B)
  val scheduled_redirect = Reg(new Redirect)
  val scheduled_bpu_resp = Reg(new BranchPredictionBundle)
  val scheduled_counter = Reg(UInt(cntBits.W))
  val last_stage_data_arrive = Wire(Bool())

  last_stage_data_arrive := io.last_stage_info.valid && io.last_stage_info.bits.ftqIdx === (scheduled_redirect.ftqIdx)

  val last_stage_info_reg = Reg(new LastStageInfo)
  when (last_stage_data_arrive) {
    last_stage_info_reg := io.last_stage_info.bits
  }

  io.toFtqRedirect.valid := false.B
  io.toFtqRedirect.bits := DontCare

  io.toBypass.valid := false.B
  io.toBypass.bits := DontCare

  when (l0_redirect_scheduled) {
    scheduled_redirect_valid := true.B
    scheduled_redirect := l0_redirect
    scheduled_bpu_resp := l0_bpu_resp
    scheduled_counter := l0_remainIterNum - LcHandoverThreshold.U
  } .elsewhen (scheduled_redirect_valid && (io.flushFromBpuIfu.shouldFlushByIfu(scheduled_redirect.ftqIdx) || io.flushFromBpuIfu.shouldFlushByStage2(scheduled_redirect.ftqIdx) || io.flushFromBpuIfu.shouldFlushByStage3(scheduled_redirect.ftqIdx))) {
    // if any bpu redirect, flush scheduled redirect
    scheduled_redirect_valid := false.B
  } .elsewhen (RegNext(last_stage_data_arrive) && scheduled_redirect_valid) {
    scheduled_redirect_valid := false.B

    // launch redirect
    io.toFtqRedirect.valid := true.B
    io.toFtqRedirect.bits := scheduled_redirect
    io.toFtqRedirect.bits.cfiUpdate.folded_hist := last_stage_info_reg.last_stage_spec_info.folded_hist
    io.toFtqRedirect.bits.cfiUpdate.lastBrNumOH := last_stage_info_reg.last_stage_spec_info.lastBrNumOH
    io.toFtqRedirect.bits.cfiUpdate.afhob := last_stage_info_reg.last_stage_spec_info.afhob
    io.toFtqRedirect.bits.cfiUpdate.histPtr := last_stage_info_reg.last_stage_spec_info.histPtr
    io.toFtqRedirect.bits.cfiUpdate.rasSp := last_stage_info_reg.last_stage_spec_info.rasSp
    io.toFtqRedirect.bits.cfiUpdate.rasEntry := last_stage_info_reg.last_stage_spec_info.rasTop

    // launch update
    io.toBypass.valid := true.B
    io.toBypass.bits.expected_loop_cnt := scheduled_counter// fixme with actual count
    io.toBypass.bits.single_entry := scheduled_bpu_resp
    io.toBypass.bits.single_entry.ftq_idx := scheduled_redirect.ftqIdx
    io.toBypass.bits.single_entry.isDouble := scheduled_bpu_resp.full_pred(dupForFtq).cfiIndex.bits < (PredictWidth / 2).U
    io.toBypass.bits.last_stage_meta := last_stage_info_reg.last_stage_meta
    io.toBypass.bits.last_stage_ftb_entry := last_stage_info_reg.last_stage_ftb_entry
    io.toBypass.bits.last_stage_spec_info := last_stage_info_reg.last_stage_spec_info
  }
  XSPerfAccumulate(f"loop_cache_redirect_frontend", io.toFtqRedirect.valid)
  /*
  * l1: identify taken instruction position
  * */
  val l1_hit = RegInit(0.B)
  val l1_pc = Reg(UInt(VAddrBits.W))
  val l1_data = Reg(new LoopCacheSpecEntry)
  val l1_taken_pc = Reg(UInt(VAddrBits.W))
  val l1_pc_hit = Reg(l0_pc_hit.cloneType)
  val l1_ftqPtr = Reg(new FtqPtr)
  val l1_pd = Reg(new PredecodeWritebackBundle)
  val l1_is_exit = Reg(Bool())
  val l1_isInterNumGT2 = Reg(Bool())
  val l1_isDouble = Reg(Bool())
  val l1_isExit = Reg(Bool())

  val l1_flush_by_bpu = io.flushFromBpuIfu.shouldFlushByStage3(l1_ftqPtr)
  val l1_flush_by_ifu = io.flushFromBpuIfu.shouldFlushByIfu(l1_ftqPtr)
  val l1_prev_hit = Reg(Bool())
  l0_fire := l0_hit && io.req.ready
  io.req.ready := !l1_hit || l1_fire

  when (l0_fire && !l0_flush_by_bpu && !l0_flush_by_ifu) {
    l1_hit := l0_hit
    l1_pc := l0_pc
    l1_data := l0_data
    l1_pc_hit := l0_pc_hit
    l1_ftqPtr := io.req.bits.ftqPtr
    l1_pd := cache_pd
    l1_is_exit := l0_is_exit
    l1_isInterNumGT2 := l0_isInterNumGT2
    l1_prev_hit := prev_hit
    l1_isDouble := l0_isDouble
    l1_isExit := l0_isExit
  } .elsewhen (l1_fire || l1_flush_by_bpu || l1_flush_by_ifu) {
    l1_hit := false.B
  }

  val l1_pc_hit_pos = OHToUInt(l1_pc_hit)

  /*
  * l2: cut instruction flow after loop hit position
  * */
  val l2_hit = RegInit(0.B)
  val l2_pc = Reg(UInt(VAddrBits.W))
  val l2_data = Reg(new LoopCacheSpecEntry)
  l2_fire := l2_hit && io.resp.ready
  val l2_pc_hit_pos = Reg(l1_pc_hit_pos.cloneType)
  val l2_ftqPtr = Reg(new FtqPtr)
  val l2_flush_by_ifu = io.flushFromBpuIfu.shouldFlushByIfu(l2_ftqPtr)
  val l2_pd = Reg(new PredecodeWritebackBundle())
  val l2_is_exit = Reg(Bool())
  val l2_isInterNumGT2 = Reg(Bool())
  val l2_isDouble = Reg(Bool())
  val l2_isExit = Reg(Bool())

  l1_fire := l1_hit && (!l2_hit || l2_fire)



  def double_pd(orig: PredecodeWritebackBundle, isDouble: Bool): PredecodeWritebackBundle = {
    val doubled = WireInit(orig)
    for (i <- PredictWidth / 2 until PredictWidth) {
      val offset = i - PredictWidth / 2
      doubled.pc(i) := doubled.pc(offset)
      doubled.pd(i) := doubled.pd(offset)
      doubled.instrRange(i) := doubled.instrRange(offset)
      // double only occurs at loop cache, ftqOffset, misOffset and cfiOffset should not work
      // target and jal target must be the same for two iterations
    }
    Mux(isDouble, doubled, orig)
  }
  def double_data(orig: LoopCacheSpecEntry, isDouble: Bool): LoopCacheSpecEntry = {
    val doubled = WireInit(orig)

    for (i <- PredictWidth / 2 until PredictWidth) {
      val offset = i - PredictWidth / 2
      doubled.instEntry(i) := doubled.instEntry(offset)
      // ftqOffset retained
      doubled.instEntry(i).ftqOffset := i.U
    }

    Mux(isDouble, doubled, orig)
  }

  when (l1_fire && !l1_flush_by_bpu && !l1_flush_by_ifu) {
    l2_hit := l1_hit
    l2_pc := l1_pc
    l2_data := double_data(l1_data, l1_isDouble)
    l2_pc_hit_pos := l1_pc_hit_pos
    for (i <- 0 until LoopCacheSpecSize) {
      l2_data.instEntry(i).pred_taken := false.B
    }
    l2_data.instEntry(l1_pc_hit_pos).pred_taken := true.B
    l2_data.instEntry(l1_pc_hit_pos + (PredictWidth / 2).U).pred_taken := l1_isDouble

    // override exit case
    when (l1_isExit) {
      when (l1_isDouble) {
        l2_data.instEntry(l1_pc_hit_pos + (PredictWidth / 2).U).pred_taken := false.B
      } .otherwise {
        l2_data.instEntry(l1_pc_hit_pos).pred_taken := false.B
      }
    }

    l2_ftqPtr := l1_ftqPtr
    l2_pd := double_pd(l1_pd, l1_isDouble)
    l2_is_exit := l1_is_exit
    l2_isInterNumGT2 := l1_isInterNumGT2
    l2_isDouble := l1_isDouble
    l2_isExit := l1_isExit
  } .elsewhen (l2_fire || l2_flush_by_ifu) {
    l2_hit := false.B
  }


  val l2_valids = Wire(Vec(LoopCacheSpecSize, Bool()))
  for (i <- 0 until LoopCacheSpecSize) {
    l2_valids(i) := (i.U <= l2_pc_hit_pos || (l2_isDouble && i.U >= (PredictWidth / 2).U && i.U <= l2_pc_hit_pos + (PredictWidth / 2).U)) && l2_pd.pd(i).valid
  }

  io.resp.valid := l2_hit
  io.resp.bits.entry := l2_data
  io.resp.bits.valids := l2_valids
  io.resp.bits.pd := l2_pd

  XSPerfAccumulate(f"loop_cache_spec_fill_hit", io.resp.fire);
  XSPerfAccumulate(f"loop_cache_provide_double", io.resp.fire && l2_isDouble)

  XSPerfAccumulate(f"loop_cache_l0_flush_by_bpu", l0_fire && l0_flush_by_bpu)
  XSPerfAccumulate(f"loop_cache_l0_flush_by_ifu", l0_fire && l0_flush_by_ifu)
  XSPerfAccumulate(f"loop_cache_l1_flush_by_bpu", l1_hit && l1_flush_by_bpu)
  XSPerfAccumulate(f"loop_cache_l1_flush_by_ifu", l1_hit && l1_flush_by_ifu)
  XSPerfAccumulate(f"loop_cache_l2_flush_by_ifu", l2_hit && l2_flush_by_ifu)

  io.pd_valid := RegNext(l2_fire && !(io.flush.sfence_valid || io.flush.fencei_valid || io.redirectFlush || l2_flush_by_ifu))
  // io.pd_ftqIdx := RegNext(l2_ftqPtr)
  io.pd_data := RegNext(l2_pd)
  io.pd_data.ftqIdx := RegNext(l2_ftqPtr)
  io.pd_data.instrRange := RegNext(l2_valids)// RegNext(VecInit((l2_pd.instrRange zip l2_valids).map{ case (range, valid) => range && valid}))

  /*
   * update
   */
  when (io.update.valid) {
    cache_valid := true.B
    cache_pc := io.update.bits.pc
    cache_data := io.update.bits.hit_data
    cache_pd := io.update.bits.pd
  }

  XSPerfAccumulate(f"loop_cache_update", io.update.valid)

  when (io.flush.sfence_valid || io.flush.fencei_valid || io.redirectFlush) {
    cache_valid := false.B
    l1_hit := false.B
    l2_hit := false.B
    io.resp.valid := false.B
    io.pd_valid := false.B
    scheduled_redirect_valid := false.B
    io.toFtqRedirect.valid := false.B
  }
}

class LoopToArbiter(implicit p: Parameters) extends LoopCacheResp {
  def toFetchToIBuffer(ptr: FtqPtr) = {
    val ret = Wire(new FetchToIBuffer)
    val _instr = Seq.tabulate(LoopCacheMaxInst * 2)(i => entry.instEntry(i).inst)
    val _instrPad = Seq.tabulate(PredictWidth - LoopCacheMaxInst * 2)(i => 0.U.asTypeOf(UInt(32.W)))

    val _valid = Seq.tabulate(LoopCacheMaxInst * 2)(i => valids(i))
    val _validPad = Seq.tabulate(PredictWidth - LoopCacheMaxInst * 2)(i => false.B)

    val _enqEnable = Seq.tabulate(LoopCacheMaxInst * 2)(i => valids(i))
    val _enqPad = Seq.tabulate(PredictWidth - LoopCacheMaxInst * 2)(i => false.B)

    val _pd = Seq.tabulate(LoopCacheMaxInst * 2)(i => entry.instEntry(i).pd)
    val _pdPad = Seq.tabulate(PredictWidth - LoopCacheMaxInst * 2)(i => 0.U.asTypeOf(new PreDecodeInfo))

    val _pc = Seq.tabulate(LoopCacheMaxInst * 2)(i => entry.instEntry(i).pc)
    val _pcPad = Seq.tabulate(PredictWidth - LoopCacheMaxInst * 2)(i => 0.U.asTypeOf(UInt(VAddrBits.W)))

    val _foldpc = Seq.tabulate(LoopCacheMaxInst * 2)(i => entry.instEntry(i).foldpc)
    val _foldpcPad = Seq.tabulate(PredictWidth - LoopCacheMaxInst * 2)(i => 0.U.asTypeOf(UInt(VAddrBits.W)))

    val _predTaken = Seq.tabulate(LoopCacheMaxInst * 2)(i => entry.instEntry(i).pred_taken)
    val _predTakenPad = Seq.tabulate(PredictWidth - LoopCacheMaxInst * 2)(i => false.B)

    //val _ftqPtr = Seq.tabulate(LoopCacheMaxInst)(i => ptr)
    //val _ftqPtrPad = Seq.tabulate(PredictWidth - LoopCacheMaxInst)(i => 0.U.asTypeOf(new FtqPtr))

    val _ftqOffset = Seq.tabulate(LoopCacheMaxInst * 2)(i => entry.instEntry(i).ftqOffset)
    val _ftqOffsetPad = Seq.tabulate(PredictWidth - LoopCacheMaxInst * 2)(i => 0.U.asTypeOf(UInt(log2Ceil(PredictWidth).W)))

    val _trigger = Seq.tabulate(LoopCacheMaxInst * 2)(i => entry.instEntry(i).triggered)
    val _triggerPad = Seq.tabulate(PredictWidth - LoopCacheMaxInst * 2)(i => 0.U.asTypeOf(new TriggerCf))

    // should never trigger
    val _ipf = Seq.tabulate(PredictWidth)(i => false.B)
    val _acf = Seq.tabulate(PredictWidth)(i => false.B)
    val _crossPageIPFFix = Seq.tabulate(PredictWidth)(i => false.B)

    val _instr_t = _instr ++ _instrPad
    val _valid_t = _valid ++ _validPad
    val _enqEnable_t = _enqEnable ++ _enqPad
    val _pd_t = _pd ++ _pdPad
    val _pc_t = _pc ++ _pcPad
    val _foldpc_t = _foldpc ++ _foldpcPad
    //val _ftqPtr_t = _ftqPtr ++ _ftqPtrPad
    val _ftqOffset_t = _ftqOffset ++ _ftqOffsetPad
    val _trigger_t = _trigger ++ _triggerPad
    val _predTaken_t = _predTaken ++ _predTakenPad

    ret.instrs := VecInit(_instr_t)
    ret.is_loop := true.B
    ret.loop_pd := pd
    ret.valid := VecInit(_valid_t).asUInt
    ret.enqEnable := VecInit(_enqEnable_t).asUInt
    ret.pd := VecInit(_pd_t)
    ret.pc := VecInit(_pc_t)
    ret.foldpc := VecInit(_foldpc_t)
    ret.ftqPtr := ptr
    // fixme: valid seems to be related with pred_taken in Ibuffer
    ret.ftqOffset.zipWithIndex.map{case (a,i) => (a.valid := _predTaken_t(i), a.bits := _ftqOffset_t(i))} // := VecInit(_ftqOffset_t)
    ret.crossPageIPFFix := VecInit(_crossPageIPFFix)
    ret.triggered := VecInit(_trigger_t)

    ret.ipf := VecInit(_ipf)
    ret.acf := VecInit(_acf)
    ret.crossPageIPFFix := VecInit(_crossPageIPFFix)

    ret
  }
}

class LoopIFUArbiterIO(implicit p: Parameters) extends XSBundle {
  val fromLoop = Flipped(Decoupled(new LoopToArbiter))
  val fromIFU = Flipped(Decoupled(new FetchToIBuffer))
  val toIBuffer = Decoupled(new FetchToIBuffer())
  val flags = Input(Vec(FtqSize, Bool()))
  val redirect = Flipped(Valid(new FtqPtr))
}

class LoopIFUArbiter(implicit p: Parameters) extends XSModule with HasCircularQueuePtrHelper {
  val io = IO(new LoopIFUArbiterIO)

  val currentPtr = RegInit(0.U.asTypeOf(new FtqPtr))

  val selLoop = io.flags(currentPtr.value)

  io.fromLoop.ready := selLoop && io.toIBuffer.ready
  io.fromIFU.ready := (!selLoop && io.toIBuffer.ready)

  when (io.toIBuffer.fire) {
    currentPtr := currentPtr + 1.U
  }

  when (io.redirect.valid && isBefore(io.redirect.bits, currentPtr)) {
    currentPtr := io.redirect.bits
  }

  // instruction flow must be consecutive
  XSError(io.toIBuffer.fire && io.toIBuffer.bits.ftqPtr =/= currentPtr, "Ftqptr mismatch on arbiter")

  io.toIBuffer.bits := Mux(selLoop, io.fromLoop.bits.toFetchToIBuffer(currentPtr), io.fromIFU.bits)
  io.toIBuffer.bits.is_loop := selLoop
  io.toIBuffer.bits.loop_pd := Mux(selLoop, io.fromLoop.bits.pd, DontCare)
  io.toIBuffer.valid := Mux(io.redirect.valid && !isAfter(io.redirect.bits, currentPtr), false.B, Mux(selLoop, io.fromLoop.valid, io.fromIFU.valid))
}



class BpuBypass(implicit p: Parameters) extends XSModule with LoopPredictorParams with HasCircularQueuePtrHelper {
  val io = IO(new BpuBypassIO)

  val BypassSel = RegInit(0.B)
  val BypassCnt = Reg(UInt(cntBits.W))
  val BypassTemplate = Reg(new BranchPredictionBundle)
  val BypassLastStageInfo = Reg(new LastStageInfo)
  val BypassPtr = Reg(new FtqPtr)



  val BypassOut = Wire(new BpuToFtqIO)

  BypassOut.resp.bits.s3 := DontCare
  BypassOut.resp.bits.s3.valid := VecInit(Seq.fill(numDup)(false.B))
  BypassOut.resp.bits.last_stage_meta := DontCare
  BypassOut.resp.bits.last_stage_ftb_entry := DontCare
  BypassOut.resp.bits.last_stage_spec_info := DontCare


  when (BypassSel && io.BpuOut.resp.fire) {
    BypassCnt := BypassCnt - Mux(BypassTemplate.isDouble && BypassCnt > 1.U, 2.U, 1.U)
    BypassPtr := BypassPtr + 1.U
    XSError(io.BpuPtr =/= BypassPtr, "Bypass and Bpu ptr mismatch on handshake, could be error")
  }

  when (RegNext(RegNext(BypassSel && io.BpuOut.resp.fire))) {
    BypassOut.resp.bits.s3 := RegNext(RegNext(BypassOut.resp.bits.s1))
    BypassOut.resp.bits.last_stage_meta := RegNext(RegNext(BypassLastStageInfo.last_stage_meta))
    BypassOut.resp.bits.last_stage_ftb_entry := RegNext(RegNext(BypassLastStageInfo.last_stage_ftb_entry))
    BypassOut.resp.bits.last_stage_spec_info := RegNext(RegNext(BypassLastStageInfo.last_stage_spec_info))
  }

  when ((BypassSel && io.redirect.valid && !isAfter(io.redirect.bits, BypassPtr)) || (BypassSel && BypassCnt === 0.U/*(BypassCnt === 1.U || (BypassCnt === 2.U && BypassTemplate.isDouble))*/)) {
    BypassSel := false.B
  }

  when (RegNext(io.update.valid && !io.redirect.valid) && !io.redirect.valid) {
    BypassSel := true.B
    BypassCnt := RegNext(io.update.bits.expected_loop_cnt)
    BypassTemplate := RegNext(io.update.bits.single_entry)
    // should start at next entry
    BypassPtr := RegNext(io.update.bits.single_entry.ftq_idx + 1.U)
    BypassLastStageInfo.ftqIdx := RegNext(io.update.bits.single_entry.ftq_idx + 1.U)
    BypassLastStageInfo.last_stage_spec_info := RegNext(io.update.bits.last_stage_spec_info)
    BypassLastStageInfo.last_stage_meta := RegNext(io.update.bits.last_stage_meta)
    BypassLastStageInfo.last_stage_ftb_entry := RegNext(io.update.bits.last_stage_ftb_entry)
  }



  BypassOut.resp.valid := BypassSel && !RegNext(io.update.valid) && !(BypassCnt === 0.U)
  /* always provide s1 and s3 only data */
  BypassOut.resp.bits.s2 := DontCare
  BypassOut.resp.bits.s2.valid := VecInit(Seq.fill(numDup)(false.B))



  BypassOut.resp.bits.s1 := BypassTemplate
  BypassOut.resp.bits.s1.valid := BypassTemplate.valid.map(v => v && !RegNext(io.update.valid) && !(BypassCnt === 0.U))
  BypassOut.resp.bits.s1.hasRedirect := VecInit(Seq.fill(numDup)(false.B))
  BypassOut.resp.bits.s1.isDouble := BypassCnt > 1.U && BypassTemplate.isDouble
  BypassOut.resp.bits.s1.fromBypass := true.B
  BypassOut.resp.bits.s1.remainCnt := BypassCnt

  BypassOut.resp.bits.s1.ftq_idx := BypassPtr

  when (BypassCnt === 1.U || (BypassCnt === 2.U && BypassTemplate.isDouble)) {
    BypassOut.resp.bits.s1.isExit := true.B
  } .otherwise {
    BypassOut.resp.bits.s1.isExit := false.B
  }


  BypassOut.resp.ready := Mux(BypassSel, io.BpuOut.resp.ready, false.B)

  /*
  * BpuOut switch should be done from s1 to s3 cycle by cycle in order to prevent missing last stage info
  * */
  io.BpuOut.resp.valid  := Mux(BypassSel, BypassOut.resp.valid, io.BpuIn.resp.valid)
  io.BpuOut.resp.bits.s1 := Mux(BypassSel, BypassOut.resp.bits.s1, io.BpuIn.resp.bits.s1)
  io.BpuOut.resp.bits.s2 := Mux(BypassSel, BypassOut.resp.bits.s2, io.BpuIn.resp.bits.s2)
  io.BpuOut.resp.bits.s3 := Mux(RegNext(BypassSel, init = false.B), BypassOut.resp.bits.s3, io.BpuIn.resp.bits.s3)
  io.BpuOut.resp.bits.last_stage_ftb_entry := Mux(RegNext(BypassSel, init = false.B), BypassOut.resp.bits.last_stage_ftb_entry, io.BpuIn.resp.bits.last_stage_ftb_entry)
  io.BpuOut.resp.bits.last_stage_meta := Mux(RegNext(BypassSel, init = false.B), BypassOut.resp.bits.last_stage_meta, io.BpuIn.resp.bits.last_stage_meta)
  io.BpuOut.resp.bits.last_stage_spec_info := Mux(RegNext(BypassSel, init = false.B), BypassOut.resp.bits.last_stage_spec_info, io.BpuIn.resp.bits.last_stage_spec_info)
  io.BpuIn.resp.ready := Mux(BypassSel, false.B, io.BpuOut.resp.ready)
  XSPerfAccumulate("lc_bypass_pred", io.BpuOut.resp.fire() && BypassSel)
  XSPerfAccumulate("lc_bypass_pred_double", io.BpuOut.resp.fire() && BypassSel && io.BpuOut.resp.bits.s1.isDouble)
  XSPerfAccumulate("lc_bypass_pred_single", io.BpuOut.resp.fire() && BypassSel && !io.BpuOut.resp.bits.s1.isDouble)

  XSDebug(io.BpuOut.resp.fire() && BypassSel, p"BypassPred: PC: ${Hexadecimal(io.BpuOut.resp.bits.s1.pc(0))} remainCnt: ${BypassCnt} isDouble: ${io.BpuOut.resp.bits.s1.isDouble} is Exit: ${io.BpuOut.resp.bits.s1.isExit} Pred: ${io.BpuOut.resp.bits.s1.full_pred}\n")

}
