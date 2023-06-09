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

package xiangshan.mem

import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chisel3.util._
import utils._
import utility._
import xiangshan.ExceptionNO._
import xiangshan._
import xiangshan.backend.fu.PMPRespBundle
import xiangshan.backend.rob.{DebugLsInfoBundle, LsTopdownInfo, RobPtr}
import xiangshan.cache._
import xiangshan.cache.dcache.ReplayCarry
import xiangshan.cache.mmu.{TlbCmd, TlbReq, TlbRequestIO, TlbResp}
import xiangshan.mem.mdp._


class HybridUnit_WriteBack(implicit p: Parameters) extends XSModule {
  val io = IO(new Bundle() {
    val redirect = Flipped(Valid(new Redirect))
    // flow in
    val in = Vec(2, Flipped(Decoupled(new ExuOutput())))
    val triggerAddrHitIn = Vec(3, Input(Bool()))
    val triggerLastDataHitIn = Vec(3, Input(Bool()))
    // flow out
    val out = DecoupledIO(new ExuOutput)
    val triggerAddrHitOut = Vec(3, Output(Bool()))
    val triggerLastDataHitOut = Vec(3, Output(Bool()))
  })

  io.in.foreach { 
    case x => x.ready := true.B 
  }

  val SelectGroupSize = RollbackGroupSize
  val lgSelectGroupSize = log2Ceil(SelectGroupSize)
  val TotalSelectCycles = scala.math.ceil(log2Ceil(LoadQueueRAWSize).toFloat / lgSelectGroupSize).toInt + 1

  // delay TotalSelectCycles - 2 cycle(s)
  var valid = io.in(0).valid
  var bits = io.in(0).bits
  for (i <- 0 until TotalSelectCycles - 2) {
    valid = RegNext(valid && !bits.uop.robIdx.needFlush(io.redirect))
    bits = RegNext(bits)
  }

  var triggerAddrHit = io.triggerAddrHitIn
  var triggerLastDataHit = io.triggerLastDataHitIn
  for (i <- 0 until TotalSelectCycles - 2) {
    triggerAddrHit = RegNext(triggerAddrHit)
    triggerLastDataHit = RegNext(triggerLastDataHit)
  }

  io.out.valid := valid && !bits.uop.robIdx.needFlush(io.redirect) || io.in(1).valid
  io.out.bits := Mux(io.in(1).valid, io.in(1).bits, bits)
  io.triggerAddrHitOut := triggerAddrHit
  io.triggerLastDataHitOut := triggerLastDataHit
}

class HybridUnit(implicit p: Parameters) extends XSModule 
  with HasLoadHelper
  with HasPerfEvents
  with HasDCacheParameters
  with HasCircularQueuePtrHelper
{
  val io = IO(new Bundle() {
    val redirect = Flipped(Valid(new Redirect))
    val csrCtrl = Flipped(new CustomCSRCtrlIO)

    // flow in
    val in = Flipped(Decoupled(new ExuInput))
    val rsIdx = Input(UInt(log2Up(IssQueSize).W))
    val isFirstIssue = Input(Bool())

    // flow out
    val out_toRS = Decoupled(new ExuOutput) // to RS
    val out_toROB = Decoupled(new ExuOutput) // to ROB

    // load io
    val loadIo = new Bundle() {
      val sbuffer = new LoadForwardQueryIO
      val lsq = new LoadForwardQueryIO

      // early wakeup signal generated in load_s1, send to RS in load_s2
      val fastUop = ValidIO(new MicroOp) 

      // load trigger
      val trigger = Vec(3, new LoadUnitTriggerIO)

      // refill
      val refill = Flipped(ValidIO(new Refill))

      // bus data forward
      val tlDchannel = Input(new DcacheToLduForwardIO)
      val forward_mshr = Flipped(new LduToMissqueueForwardIO)

      // provide prefetch info
      val prefetch_train = ValidIO(new LdPrefetchTrainBundle())
      // hardware prefetch to l1 cache req
      val prefetch_req = Flipped(ValidIO(new L1PrefetchReq))

      // load to load fast path
      val fastpathIn = Input(new LoadToLoadIO)
      val fastpathOut = Output(new LoadToLoadIO)
      val loadFastMatch = Input(Bool())
      val loadFastImm = Input(UInt(12.W))

      // provide right pc for hw prefetch
      val s2IsPointerChasing = Output(Bool()) 

      // load ecc error
      val s3_delayedLoadError = Output(Bool()) // Note that io.s3_delayed_load_error and io.lsq.s3_delayed_load_error is different

      // Load fast replay path
      val fastReplayIn = Flipped(Decoupled(new LqWriteBundle))
      val fastReplayOut = Decoupled(new LqWriteBundle)

      // load Queue based replay
      val replay = Flipped(Decoupled(new LsPipelineBundle))
      val lqReplayFull = Input(Bool())

      // l2 dcache hint
      val l2Hint = Input(Valid(new L2ToL1Hint))

      // store-load violation check
      val reExecuteQueryIn = Flipped(Vec(StorePipelineWidth, Valid(new LoadReExecuteQueryIO)))

      // rs feedback
      val feedbackFast = ValidIO(new RSFeedback) // stage 2
      val feedbackSlow = ValidIO(new RSFeedback) // stage 3

      // debug
      val debug_ls = Output(new DebugLsInfoBundle)

      // topdown
      val lsTopdownInfo = Output(new LsTopdownInfo)
    }

    // store io 
    val storeIo = new Bundle() {
      // to lsq
      val lsq = ValidIO(new LsPipelineBundle)
      val lsq_replenish = Output(new LsPipelineBundle())

      // store mask, send to sq in store_s0
      val storeMaskOut = Valid(new StoreMaskBundle)

      // store-load violation check
      val reExecuteQueryOut = Valid(new LoadReExecuteQueryIO)

      // store issue 
      val issue = Valid(new ExuInput)

      // rs feedback
      val feedbackFast = ValidIO(new RSFeedback) // stage 1
      val feedbackSlow = ValidIO(new RSFeedback) // stage 2

      // debug
      val debug_ls = Output(new DebugLsInfoBundle)
    }

    // general ports
    val tlb = new TlbRequestIO(2)
    val dcache = new DCacheLoadIO
    val pmp = Flipped(new PMPRespBundle)
  })  

  val ldu = Module(new LoadUnit)
  val sta = Module(new StoreUnit)
  val wb = Module(new HybridUnit_WriteBack)

  val isLoadFlow = FuType.isLoadExu(io.in.bits.uop.ctrl.fuType)

  // load unit 
  // 
  ldu.io.redirect <> io.redirect 
  ldu.io.csrCtrl <> io.csrCtrl
  ldu.io.loadIn.valid := io.in.valid && isLoadFlow
  ldu.io.loadIn.bits := io.in.bits
  ldu.io.rsIdx <> io.rsIdx 
  ldu.io.isFirstIssue <> io.isFirstIssue
  ldu.io.replay <> io.loadIo.replay
  ldu.io.fastReplayIn <> io.loadIo.fastReplayIn
  ldu.io.fastpathIn <> io.loadIo.fastpathIn
  ldu.io.lqReplayFull <> io.loadIo.lqReplayFull
  ldu.io.pmp <> io.loadIo.pmp
  ldu.io.loadFastMatch <> io.loadIo.loadFastMatch
  ldu.io.loadFastImm <> io.loadIo.loadFastImm
  ldu.io.l2Hint <> io.loadIo.l2Hint
  ldu.io.tlDchannel <> io.loadIo.tlDchannel
  ldu.io.forward_mshr <> io.loadIo.forward_mshr
  ldu.io.reExecuteQuery <> io.loadIo.reExecuteQueryIn
  ldu.io.s2IsPointerChasing <> io.loadIo.s2IsPointerChasing
  ldu.io.debug_ls <> io.loadIo.debug_ls 
  ldu.io.lsTopdownInfo <> io.loadIo.lsTopdownInfo
  ldu.io.refill <> io.loadIo.refill
  ldu.io.prefetch_req <> io.loadIo.prefetch_req

  //
  io.loadIo.sbuffer <> ldu.io.sbuffer
  io.loadIo.lsq <> ldu.io.lsq
  io.loadIo.trigger <> ldu.io.trigger
  io.loadIo.prefetch_train <> ldu.io.prefetch_train
  io.loadIo.fastpathOut <> ldu.io.fastpathOut
  io.loadIo.fastReplayOut <> ldu.io.fastReplayOut
  io.loadIo.s3_delayedLoadError <> ldu.io.s3_delayedLoadError

  // store unit
  sta.io.redirect <> io.redirect
  sta.io.stin.valid := io.in.valid && !isLoadFlow
  sta.io.stin.bits := io.in.bits
  sta.io.rsIdx <> io.rsIdx 
  sta.io.isFirstIssue <> io.isFirstIssue
  sta.io.pmp <> io.pmp

  io.storeIo.lsq <> sta.io.lsq
  io.storeIo.lsq_replenish <> sta.io.lsq_replenish
  io.storeIo.storeMaskOut <> sta.io.storeMaskOut
  io.storeIo.reExecuteQueryOut <> sta.io.reExecuteQuery
  io.storeIo.issue <> sta.io.issue
  io.storeIo.debug_ls <> sta.io.debug_ls

  // share ports
  // flow in ready
  io.in.ready := Mux(isLoadFlow, ldu.io.loadIn.ready, sta.io.stin.ready)

  // tlb request
  io.tlb.req_kill := ldu.io.tlb.req_kill || sta.io.tlb.req_kill

  ldu.io.tlb.req.ready := false.B
  sta.io.tlb.req.ready := false.B
  when (ldu.io.tlb.req.valid) {
    io.tlb.req <> ldu.io.tlb.req
  } .otherwise {
    io.tlb.req <> sta.io.tlb.req
  }

  io.tlb.resp.ready := false.B
  when (RegNext(ldu.io.tlb.req.valid)) {
    ldu.io.tlb.resp <> io.tlb.resp
  } .otherwise {
    sta.io.tlb.resp <> io.tlb.resp
  }

  // dcache request
  io.dcache <> ldu.io.dcache

  // rs feedback
  io.loadIo.feedbackFast <> ldu.io.feedbackFast
  io.loadIo.feedbackSlow <> ldu.io.feedbackSlow

  io.storeIo.feedbackFast.valid := false.B
  io.storeIo.feedbackFast.bits := DontCare
  io.storeIo.feedbackSlow <> sta.io.feedbackSlow


  // mixed writeback
  wb.io.redirect <> io.redirect
  wb.io.in(0) <> ldu.io.loadOut
  wb.io.in(1) <> sta.io.stout
  wb.io.triggerAddrHitIn := VecInit(ldu.io.trigger.map(_.addrHit))
  wb.io.triggerLastDataHitIn := VecInit(ldu.io.trigger.map(_.lastDataHit))

  io.out_toRS <> ldu.io.loadOut
  io.out_toROB <> wb.io.out

  for (i <- 0 until 3) {
    io.loadIo.trigger(i).triggerAddrHit := wb.io.triggerAddrHitOut
    io.loadIo.trigger(i).triggerLastDataHit := wb.io.triggerLastDataHitOut
  }

  when (io.out_toRS.fire) {
    XSDebug("loadOut %x\n", io.out_toRS.bits.uop.cf.pc)    
  }

  val perfEvents = Seq(ldu, sta).flatMap(_.getPerfEvents)
  generatePerfEvent()  

  // end
}