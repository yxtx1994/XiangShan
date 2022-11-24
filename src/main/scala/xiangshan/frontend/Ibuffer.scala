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

package xiangshan.frontend

import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chisel3.util._
import xiangshan._
import utils._
import xiangshan.ExceptionNO._

class IbufPtr(implicit p: Parameters) extends CircularQueuePtr[IbufPtr](
  p => p(XSCoreParamsKey).IBufSize
){
}

class IBufferIO(implicit p: Parameters) extends XSBundle {
  val flush = Input(Bool())
  val in = Flipped(DecoupledIO(new FetchToIBuffer))
  val loop_peek = Flipped(new FtqToIBuffer)
  val loop_out = new IBufferToFtq
  val fence = Input(new LoopCacheFenceBundle)
  val PdWb = Flipped(Valid(new PredecodeWritebackBundle))
  val out = Vec(DecodeWidth, DecoupledIO(new CtrlFlow))
  val full = Output(Bool())
}

class IBufEntry(implicit p: Parameters) extends XSBundle {
  val inst = UInt(32.W)
  val pc = UInt(VAddrBits.W)
  val foldpc = UInt(MemPredPCWidth.W)
  val pd = new PreDecodeInfo
  val pred_taken = Bool()
  val ftqPtr = new FtqPtr
  val ftqOffset = UInt(log2Ceil(PredictWidth).W)
  val ipf = Bool()
  val acf = Bool()
  val crossPageIPFFix = Bool()
  val triggered = new TriggerCf

  def fromFetch(fetch: FetchToIBuffer, i: Int): IBufEntry = {
    inst   := fetch.instrs(i)
    pc     := fetch.pc(i)
    foldpc := fetch.foldpc(i)
    pd     := fetch.pd(i)
    pred_taken := fetch.ftqOffset(i).valid
    ftqPtr := fetch.ftqPtr
    ftqOffset := fetch.ftqOffset(i).bits
    ipf := fetch.ipf(i)
    acf := fetch.acf(i)
    crossPageIPFFix := fetch.crossPageIPFFix(i)
    triggered := fetch.triggered(i)
    this
  }

  def toCtrlFlow: CtrlFlow = {
    val cf = Wire(new CtrlFlow)
    cf.instr := inst
    cf.pc := pc
    cf.foldpc := foldpc
    cf.exceptionVec := 0.U.asTypeOf(ExceptionVec())
    cf.exceptionVec(instrPageFault) := ipf
    cf.exceptionVec(instrAccessFault) := acf
    cf.trigger := triggered
    cf.pd := pd
    cf.pred_taken := pred_taken
    cf.crossPageIPFFix := crossPageIPFFix
    cf.storeSetHit := DontCare
    cf.waitForRobIdx := DontCare
    cf.loadWaitBit := DontCare
    cf.loadWaitStrict := DontCare
    cf.ssid := DontCare
    cf.ftqPtr := ftqPtr
    cf.ftqOffset := ftqOffset
    cf
  }
}

class LoopCacheSpecEntry(implicit p: Parameters) extends XSBundle  {
  val instEntry = Vec(LoopCacheMaxInst, new IBufEntry())
}

class Ibuffer(implicit p: Parameters) extends XSModule with HasCircularQueuePtrHelper with HasPerfEvents {
  val io = IO(new IBufferIO)

  val ibuf = Module(new SyncDataModuleTemplate(new IBufEntry, IBufSize, 2 * DecodeWidth, PredictWidth, "IBuffer"))

  val deqPtrVec = RegInit(VecInit.tabulate(2 * DecodeWidth)(_.U.asTypeOf(new IbufPtr)))
  val deqPtrVecNext = Wire(Vec(2 * DecodeWidth, new IbufPtr))
  deqPtrVec := deqPtrVecNext
  val deqPtr = deqPtrVec(0)

  val enqPtrVec = RegInit(VecInit.tabulate(PredictWidth)(_.U.asTypeOf(new IbufPtr)))
  val enqPtr = enqPtrVec(0)

  val validEntries = distanceBetween(enqPtr, deqPtr)
  val allowEnq = RegInit(true.B)

  val numEnq = Mux(io.in.fire, PopCount(io.in.bits.valid), 0.U)
  val numTryDeq = Mux(validEntries >= DecodeWidth.U, DecodeWidth.U, validEntries)
  val numDeq = Mux(io.out.head.ready, numTryDeq, 0.U)
  deqPtrVecNext := Mux(io.out.head.ready, VecInit(deqPtrVec.map(_ + numTryDeq)), deqPtrVec)

  val numAfterEnq = validEntries +& numEnq
  val nextValidEntries = Mux(io.out(0).ready, numAfterEnq - numTryDeq, numAfterEnq)
  allowEnq := (IBufSize - PredictWidth).U >= nextValidEntries

  // Enque
  io.in.ready := allowEnq

  val enqOffset = Seq.tabulate(PredictWidth)(i => PopCount(io.in.bits.valid.asBools.take(i)))
  val enqData = Seq.tabulate(PredictWidth)(i => Wire(new IBufEntry).fromFetch(io.in.bits, i))
  for (i <- 0 until PredictWidth) {
    ibuf.io.waddr(i) := enqPtrVec(enqOffset(i)).value
    ibuf.io.wdata(i) := enqData(i)
    ibuf.io.wen(i)   := io.in.bits.enqEnable(i) && io.in.fire && !io.flush
  }

  // val loopCacheInstSize = Vec(LoopCacheSpecSize, Reg(UInt(log2Ceil(LoopCacheMaxInst).W)))
  val loopCachePc = Reg(Vec(LoopCacheSpecSize, UInt(VAddrBits.W)))
  val loopCacheValid = RegInit(VecInit(Seq.fill(LoopCacheSpecSize)(0.B)))

  val loopCacheInstBody = Module(new SRAMTemplate(new LoopCacheSpecEntry, LoopCacheSpecSize, 1, true, false))
  val loopCachePd = Module(new SRAMTemplate(new PredecodeWritebackBundle, LoopCacheSpecSize, 1, true, false))

  val PdReg = Reg(new PredecodeWritebackBundle)
  val Pdvalid = RegInit(0.B)

  when (RegNext(io.in.valid && !io.in.fire)) {
    Pdvalid := true.B
    PdReg := io.PdWb.bits
  } .elsewhen (RegNext(io.in.fire)) {
    Pdvalid := false.B
  }

  val loopCacheSpecReplacer = ReplacementPolicy.fromString("plru", LoopCacheSpecSize)
  val loopCacheReplacer_touch_ways = Wire(Vec(1, Valid(UInt(log2Ceil(LoopCacheSpecSize).W))))

  val loopCacheEnqData = Wire(new LoopCacheSpecEntry)

  val enqIsDup = (loopCacheValid zip loopCachePc).map{case (v,p) => v && p === enqData(0).pc}.reduce((a,b) => a|b)

  for (i <- 0 until LoopCacheMaxInst) {
    loopCacheEnqData.instEntry(i) := enqData(i)
  }

  loopCacheInstBody.io.w(
    RegNext(io.in.bits.valid.orR && io.in.fire && !io.flush && !enqIsDup),
    RegNext(loopCacheEnqData),
    RegNext(loopCacheSpecReplacer.way),
    1.U
  )

  loopCachePd.io.w(
    RegNext(io.in.bits.valid.orR && io.in.fire && !io.flush && !enqIsDup),
    Mux(Pdvalid, PdReg, Mux(io.PdWb.valid, io.PdWb.bits, RegNext(io.PdWb.bits))),
    RegNext(loopCacheSpecReplacer.way),
    1.U
  )

  val peek_valid_reg = RegInit(0.B)
  val peek_pc_reg = Reg(UInt(VAddrBits.W))
  val peek_pc = Mux(io.loop_peek.pc.valid, io.loop_peek.pc.bits, peek_pc_reg)
  val pc_hit = (loopCacheValid zip loopCachePc).map{case (v,p) => v && p === peek_pc}.reduce((a,b) => a|b)

  when (io.loop_peek.pc.valid && !loopCacheInstBody.io.r.req.fire && pc_hit) {
    peek_valid_reg := true.B
    peek_pc_reg := io.loop_peek.pc.bits
  } .elsewhen (loopCacheInstBody.io.r.req.fire) {
    peek_valid_reg := false.B
  }

  loopCacheInstBody.io.r.req.valid := (io.loop_peek.pc.valid || peek_valid_reg) && pc_hit
  loopCacheInstBody.io.r.req.bits.setIdx := OHToUInt((loopCacheValid zip loopCachePc).map{case (v,p) => v && p === peek_pc})

  loopCachePd.io.r.req.valid := (io.loop_peek.pc.valid || peek_valid_reg) && pc_hit
  loopCachePd.io.r.req.bits.setIdx := OHToUInt((loopCacheValid zip loopCachePc).map{case (v,p) => v && p === peek_pc})

  io.loop_out.update.valid := RegNext(loopCacheInstBody.io.r.req.fire)
  io.loop_out.update.bits.hit_data := loopCacheInstBody.io.r.resp.data(0)
  io.loop_out.update.bits.pd := loopCachePd.io.r.resp.data(0)
  io.loop_out.update.bits.pc := RegNext(Mux(io.loop_peek.pc.valid, io.loop_peek.pc.bits, peek_pc_reg))

  val loop_refill_valid = RegInit(0.B)
  val loop_refill_pc = Reg(UInt(VAddrBits.W))

  when (io.loop_peek.pc.valid && !pc_hit) {
    loop_refill_valid := true.B
    loop_refill_pc := io.loop_peek.pc.bits
  } .elsewhen (io.in.bits.valid.orR && io.in.fire && !io.flush && loop_refill_pc === enqData(0).pc) {
    io.loop_out.update.valid := RegNext(true.B)
    io.loop_out.update.bits.hit_data := RegNext(loopCacheEnqData)
    io.loop_out.update.bits.pc := RegNext(loop_refill_pc)
  }
  /*
  io.loop_out.specReq.ready := loopCacheInstBody.io.r.req.ready
  loopCacheInstBody.io.r.req.valid := io.loop_out.specReq.valid
  loopCacheInstBody.io.r.req.bits.setIdx := io.loop_out.specReq.bits
   */


  when (io.in.bits.valid.orR && io.in.fire && !io.flush && !enqIsDup) {
    // fixme: proper conditions here
    loopCachePc(loopCacheSpecReplacer.way) := enqData(0).pc
    loopCacheValid(loopCacheSpecReplacer.way) := true.B
  }

  when (io.fence.fencei_valid || io.fence.sfence_valid) {
    for (i <- 0 until LoopCacheSpecSize) {
      loopCacheValid(i) := false.B
    }
  }

  loopCacheReplacer_touch_ways(0).valid := io.in.bits.enqEnable.orR && io.in.fire && !io.flush && !enqIsDup
  loopCacheReplacer_touch_ways(0).bits := RegEnable(loopCacheSpecReplacer.way, io.in.bits.enqEnable.orR && io.in.fire && !io.flush)

  loopCacheSpecReplacer.access(loopCacheReplacer_touch_ways)

  when (io.in.fire && !io.flush) {
    enqPtrVec := VecInit(enqPtrVec.map(_ + PopCount(io.in.bits.enqEnable)))
  }

  // Dequeue
  val validVec = Mux(validEntries >= DecodeWidth.U,
    ((1 << DecodeWidth) - 1).U,
    UIntToMask(validEntries(log2Ceil(DecodeWidth) - 1, 0), DecodeWidth)
  )
  val deqData = Reg(Vec(DecodeWidth, new IBufEntry))
  for (i <- 0 until DecodeWidth) {
    io.out(i).valid := validVec(i)
    // by default, all bits are from the data module (slow path)
    io.out(i).bits := ibuf.io.rdata(i).toCtrlFlow
    // some critical bits are from the fast path
    val fastData = deqData(i).toCtrlFlow
    io.out(i).bits.instr := fastData.instr
    io.out(i).bits.exceptionVec := fastData.exceptionVec
    io.out(i).bits.foldpc := fastData.foldpc
    XSError(io.out(i).fire && fastData.instr =/= ibuf.io.rdata(i).toCtrlFlow.instr, "fast data error\n")
  }
  val nextStepData = Wire(Vec(2 * DecodeWidth, new IBufEntry))
  val ptrMatch = new QPtrMatchMatrix(deqPtrVec, enqPtrVec)
  for (i <- 0 until 2 * DecodeWidth) {
    val enqMatchVec = VecInit(ptrMatch(i))
    val enqBypassEnVec = io.in.bits.valid.asBools.zip(enqOffset).map{ case (v, o) => v && enqMatchVec(o) }
    val enqBypassEn = io.in.fire && VecInit(enqBypassEnVec).asUInt.orR
    val enqBypassData = Mux1H(enqBypassEnVec, enqData)
    val readData = if (i < DecodeWidth) deqData(i) else ibuf.io.rdata(i)
    nextStepData(i) := Mux(enqBypassEn, enqBypassData, readData)
  }
  val deqEnable_n = io.out.map(o => !o.fire) :+ true.B
  for (i <- 0 until DecodeWidth) {
    deqData(i) := ParallelPriorityMux(deqEnable_n, nextStepData.drop(i).take(DecodeWidth + 1))
  }
  ibuf.io.raddr := VecInit(deqPtrVecNext.map(_.value))

  // Flush
  when (io.flush) {
    allowEnq := true.B
    deqPtrVec := deqPtrVec.indices.map(_.U.asTypeOf(new IbufPtr))
    enqPtrVec := enqPtrVec.indices.map(_.U.asTypeOf(new IbufPtr))
  }
  io.full := !allowEnq

  // Debug info
  XSDebug(io.flush, "IBuffer Flushed\n")

  when(io.in.fire) {
    XSDebug("Enque:\n")
    XSDebug(p"MASK=${Binary(io.in.bits.valid)}\n")
    for(i <- 0 until PredictWidth){
      XSDebug(p"PC=${Hexadecimal(io.in.bits.pc(i))} ${Hexadecimal(io.in.bits.instrs(i))}\n")
    }
  }

  for (i <- 0 until DecodeWidth) {
    XSDebug(io.out(i).fire,
      p"deq: ${Hexadecimal(io.out(i).bits.instr)} PC=${Hexadecimal(io.out(i).bits.pc)}" +
      p"v=${io.out(i).valid} r=${io.out(i).ready} " +
      p"excpVec=${Binary(io.out(i).bits.exceptionVec.asUInt)} crossPageIPF=${io.out(i).bits.crossPageIPFFix}\n")
  }

  XSDebug(p"ValidEntries: ${validEntries}\n")
  XSDebug(p"EnqNum: ${numEnq}\n")
  XSDebug(p"DeqNum: ${numDeq}\n")

  val afterInit = RegInit(false.B)
  val headBubble = RegInit(false.B)
  when (io.in.fire) { afterInit := true.B }
  when (io.flush) {
    headBubble := true.B
  } .elsewhen(validEntries =/= 0.U) {
    headBubble := false.B
  }
  val instrHungry = afterInit && (validEntries === 0.U) && !headBubble

  QueuePerf(IBufSize, validEntries, !allowEnq)
  XSPerfAccumulate("flush", io.flush)
  XSPerfAccumulate("hungry", instrHungry)

  val perfEvents = Seq(
    ("IBuffer_Flushed  ", io.flush                                                                     ),
    ("IBuffer_hungry   ", instrHungry                                                                  ),
    ("IBuffer_1_4_valid", (validEntries >  (0*(IBufSize/4)).U) & (validEntries < (1*(IBufSize/4)).U)   ),
    ("IBuffer_2_4_valid", (validEntries >= (1*(IBufSize/4)).U) & (validEntries < (2*(IBufSize/4)).U)   ),
    ("IBuffer_3_4_valid", (validEntries >= (2*(IBufSize/4)).U) & (validEntries < (3*(IBufSize/4)).U)   ),
    ("IBuffer_4_4_valid", (validEntries >= (3*(IBufSize/4)).U) & (validEntries < (4*(IBufSize/4)).U)   ),
    ("IBuffer_full     ",  validEntries.andR                                                           ),
    ("Front_Bubble     ", PopCount((0 until DecodeWidth).map(i => io.out(i).ready && !io.out(i).valid)))
  )
  generatePerfEvent()
}
