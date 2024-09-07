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

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.diplomacy.{BundleBridgeSource, LazyModule, LazyModuleImp}
import utils._
import utility._
import xiangshan._


class MemUnit(params: MemUnitParams)(implicit p: Parameters) extends LazyModule
  with HasXSParameter {
  implicit val unitParams: MemUnitParams = params
  lazy val module: MemUnitImp = unitParams.unitType match {
    case StoreUnit() => new StoreUnitImp(this)
    case LoadUnit() => new LoadUnitImp(this)
    case HybridUnit() => new HybridUnitImp(this)
    case AtomicUnit() => new AtomicUnitImp(this)
    case _ => null
  }
}

class MemUnitIO()(implicit p: Parameters, params: MemUnitParams) extends XSBundle {
  // from
  val fromCtrlPath = new Bundle() {
    val redirect = Flipped(ValidIO(new Redirect))
    val hartId = Input(UInt(hartIdLen.W))
    val csrCtrl = Flipped(new CustomCSRCtrlIO)
  }
  val fromIssuePath = new Bundle() {
    val intIn = OptionWrapper(params.hasIntPort, Flipped(Decoupled(new MemExuInput)))
    val vecIn = OptionWrapper(params.hasVecPort, Flipped(Decoupled(new VecPipeBundle)))
    val mbIn = OptionWrapper(params.hasMisAlign, Flipped(Decoupled(new LsPipelineBundle)))
  }
  val fromDataPath = new Bundle() {
    val tlb = new TlbRespIO(2)
    val pmp = Flipped(new PMPRespBundle)
    val dcache = OptionWrapper(params.hasDCacheResp, new DCacheResp)
    val lsq = OptionWrapper(params.hasStoreForward, new LsqForwardResp)
    val sbuffer = OptionWrapper(params.hasStoreForward, new SbufferForwardResp)
    val dataBus = OptionWrapper(params.hasBusForward, new DcacheToLduForwardResp)
    val mshr = OptionWrapper(params.hasMSHRForward, new LduToMissqueueForwardResp)
  }
  val fromPrefetch = new Bundle() {
    val in = OptionWrapper(params.hasPrefetch, Flipped(DecoupledIO(new StorePrefetchReq)))
  }

  // to
  val toIssuePath = new Bundle() {
    val feedback = OptionWrapper(params.hasFeedback, ValidIO(new RSFeedback))
    val lfstUpdate = OptionWrapper(params.isStoreUnit || params.isHybridUnit, Valid(new MemExuInput))
    val intOut = OptionWrapper(params.hasIntPort, Decoupled(new MemExuOutput))
    val vecOut = OptionWrapper(params.hasVecPort, Decoupled(new VecPipelineFeedbackIO(isVStore = false)))
  }
  val toDataPath = new Bundle() {
    val tlb = new TlbReqIO(2)
    val dcache = OptionWrapper(params.hasDCacheReq, new DcacheReq)
    val lsq = OptionWrapper(params.hasStoreForward, new LsqForwardReq)
    val sbuffer = OptionWrapper(params.hasStoreForward, new SbufferForwardReq)
    val mshr = OptionWrapper(params.hasMSHRForward, new LduToMissqueueForwardReq)
  }
  val debugLsInfo = OptionWrapper(params.hasDebugInfo, Output(new DebugLsInfoBundle))
  val lsTopdownInfo = OptionWrapper(params.hasTopDownInfo, Output(new LsTopdownInfo))
}

class MemUnitImp(override val wrapper: MemUnit)(implicit p: Parameters, val params: MemUnitParams)
  extends LazyModuleImp(wrapper) with HasXSParameter {
  lazy val io = IO(new MemUnitIO())

  println(s"[MemUnitImp] ${params.name}: " +
          s"  unitType: ${params.unitTypeString}" +
          s"  dataBits: ${params.dataBits}" +
          s"  hasIntPort: ${params.hasIntPort}" +
          s"  hasVecPort: ${params.hasVecPort}" +
          s"  hasStoreForward: ${params.hasStoreForward}" +
          s"  hasBusForward: ${params.hasBusForward}" +
          s"  hasMSHRForward: ${params.hasMSHRForward}" +
          s"  hasFastReplay: ${params.hasFastReplay}" +
          s"  hasReplay: ${params.hasReplay}" +
          s"  hasPrefetch: ${params.hasPrefetch}" +
          s"  hasMisalign: ${params.hasMisalign}" +
          s"  hasFeedback: ${params.hasFeedback}" +
          s"  hasDebugInfo: ${params.hasDebugInfo}" +
          s"  hasTopDownInfo: ${params.hasTopDownInfo}")
}

class StoreUnitImp(override val wrapper: MemUnit)(implicit p: Parameters, params: MemUnitParams)
  extends MemUnitImp(wrapper)
{
  io.suggestName("none")
  override lazy val io = IO(new MemUnitIO()).suggestName("io")

  val fromCtrlPath = io.fromCtrlPath
  val fromIntIssue = io.fromIssuePath.intIn.get
  val fromVecIssue = io.fromVecIssue.vecIn.get
  val formMisalignBuff = io.fromIssuePath.mbIn.get
  val fromPrefetch = io.fromPrefetch.in.get
  val toTlb = io.toDataPath.tlb
  val toDCache = io.fromDataPath.dcache.get
  val lsq = io.toDataPath.lsq

  val s1Ready, s2Ready, s3Ready = WireInit(false.B)
  // Pipeline
  // --------------------------------------------------------------------------------
  // stage 0
  // --------------------------------------------------------------------------------
  // generate addr, use addr to query DCache and DTLB
  val s0IntValid = fromIntIssue.valid
  val s0VecValid = fromVecIssue.valid
  val s0PfValid = fromPrefetch.valid && toDCache.ready
  val s0MbValid = formMisalignBuff.valid
  val s0Valid = s0IntValid || s0PfValid || s0VecValid || s0MbValid
  val s0SelectMb = s0MbValid
  val s0SelectVec = s0VecValid && !s0SelectMb
  val s0SelectInt = s0IntValid && !s0SelectVec
  val s0SelectPf = s0PfValid && !s0SelectInt
  val s0NonPfSelection =  s0SelectMb || s0SelectInt || s0SelectVec
  val s0IntBits = Mux(s0SelectInt, fromIntIssue.bits, 0.U.asTypeOf(fromIntIssue.bits))
  val s0VecBits = Mux(s0SelectVec, fromVecIssue.bits, 0.U.asTypeOf(fromVecIssue.bits))
  val s0MbBits = fromMisalignBuff.bits
  val s0PfBits = fromPrefetch.bits
  val s0Uop = Mux(
    s0SelectMb,
    fromMisalignBuff.bits.uop,
    Mux(s0SelectInt, s0IntBits.uop, s0VecBits.uop)
  )
  val s0IsFirstIssue = Mux(
    s0SelectMb,
    false.B,
    s0SelectInt && fromIntIssue.isFirstIssue || s0SelectVec && fromVecIssue.isFirstIssue
  )
  val s0Size = Mux(s0NonPfSelection, s0Uop,fuOpType(2, 0), 0.U)
  val s0MemIdx = Mux(s0NonPfSelection, s0Uop.sqIdx.value, 0.U)
  val s0RobIdx = Mux(s0NonPfSelection, s0Uop.robIdx, 0.U.asTypeOf(s0Uop.robIdx))
  val s0Pc = Mux(s0NonPfSelection, s0Uop.pc, 0.U)
  val s0InstrType = Mux(s0NonPfSelection, STORE_SOURCE.U, DCACHE_PREFETCH_SOURCE.U)
  val s0WLineFlag = Mux(s0SelectInt, s0Uop.fuOpType === LSUOpType.cbo_zero, false.B)
  val s0Is128Bits = Mux(s0SelectMb, s0MbBits.is128bit, is128bit(s0VecBits.alignedType))
  val s0VecActive = !s0SelectVec || s0VecBits.vecActive
  val s0VecSecondInv = s0VecBits.usSecondInv
  val s0VecElemIdx = s0VecBits.elemIdx
  val s0VecAlignedType = s0VecBits.alignedType
  val s0VecmBIdx = s0VecBits.mBIndex
  // val s0VecFlowPtr = s0VecBits.flowPtr
  // val s0VecIsLastElem = s0VecBits.isLastElem
  val s0Kill = s0Uop.robIdx.needFlush(fromCtrlPath.redirect)

  val s0CanGo = s1Ready
  val s0Fire = s0Valid && !s0Kill && s0CanGO
  // generate addr
  val s0SAddr = s0IntBits.src(0) + SignExt(s0Uop.imm(11, 0), VAddrBits)
  val s0VAddr = Mux(
    s0SelectMb,
    s0MbBits.vaddr,
    Mux(
      s0SelectInt,
      s0SAddr,
      Mux(
        s0SelectVec,
        s0VecBits.vaddr,
        s0PfBits.vaddr
      )
    )
  )
  val s0Mask = Mux(
    s0SelectMb,
    s0MbBits.mask,
    Mux(
      s0SelectInt,
      genVWmask128(s0SAddr, s0Uop.fuOpType(2, 0)),
      Mux(
        s0SelectVec,
        s0VecBits.mask,
        Fill(VLEN/8, 1.U(1.W))
      )
    )
  )

  toTlb.valid := s0Valid
  toTlb.bits.vaddr := s0Vaddr
  toTlb.bits.cmd   := TlbCmd.write
  toTlb.bits.size  := s0Size
  toTlb.bits.kill  := false.B
  toTlb.bits.hlvx  := false.B
  toTlb.bits.req_kill  := false.B
  toTlb.bits.hyperinst := LSUOpType.isHsv(s0Uop.fuOpType)
  toTlb.bits.pmp_addr  := DontCare
  toTlb.bits.memIdx.is_ld := false.B
  toTlb.bits.memIdx.is_st := true.B
  toTlb.bits.memIdx.idx   := s0MemIdx
  toTlb.bits.debug.robIdx := s0RobIdx
  toTlb.bits.no_translate := false.B
  toTlb.bits.debug.pc     := s0IsFirstIssue
  toTlb.bits.debug.isFirstIssue := s0IsFirstIssue

  if (params.hasStorePrefetch) {
    // Dcache access here: not **real** dcache write
    // just read meta and tag in dcache, to find out the store will hit or miss

    // NOTE: The store request does not wait for the dcache to be ready.
    //       If the dcache is not ready at this time, the dcache is not queried.
    //       But, store prefetch request will always wait for dcache to be ready to make progress.
    toDCache.valid := s0_fire
    toDCache.bits.cmd   := MemoryOpConstants.M_PFW
    toDCache.bits.vaddr := s0VAddr
    toDCache.bits.instrtype := s0InstrType
  }

  lsq.maskOut.valid := s0SelectInt || s0SelectVec
  lsq.maskOut.bits.mask := s0Out.mask
  lsq.maskOut.bits.sqIdx := s0Out.uop.sqIdx

  fromIntIssue.ready := s1Ready && s0SelectInt
  fromVecIssue.ready := s1Ready && s0SelectVec
  fromPrefetch.ready := s1Ready && toDCache.ready && !s0IntValid && !s0VecValid && !s0MbValid
  fromMisalignBuff.ready := s0Ready && s0SelectMb

  val s0Out = Wire(new LsPipelineBundle)
  s0Out := DontCare
  s0Out.uop  := s0Uop
  s0Out.vaddr:= s0VAddr
  s0Out.mask := s0Mask
  s0Out.data := s0IntBits.src(1)
  s0Out.miss := false.B
  s0Out.isvec := s0SelectVec
  s0Out.elemIdx  := s0VecElemIdx
  s0Out.mbIndex  := s0VecmBIdx
  s0Out.is128bit := s0Is128Bits
  s0Out.vecActive   := s0VecActive
  s0Out.usSecondInv := s0VecSecondInv
  s0Out.alignedType := s0VecAlignedType
  when (s0Valid && s0IsFirstIssue) {
    s0Out.uop.debugInfo.tlbFirstReqTime := GTimer()
  }
  s0Out.isFrmMisAlignBuf := s0SelectMb
  // exception check
  val s0AddrAligned = LookupTree(s0SelectVec, s0VecBits.alignedType(1, 0), s0Uop.fuOpType(1, 0), List(
    "b00".U   -> true.B,              //b
    "b01".U   -> (s0Out.vaddr(0) === 0.U),   //h
    "b10".U   -> (s0Out.vaddr(1,0) === 0.U), //w
    "b11".U   -> (s0Out.vaddr(2,0) === 0.U)  //d
  ))
  // if vector store sends 128-bit requests, its address must be 128-aligned
  XSError(s0SelectVec && s0Out.vaddr(3, 0) =/= 0.U && s0VecBits.alignedType(2), "unit stride 128 bit element is not aligned!")

  // Pipeline
  // --------------------------------------------------------------------------------
  // stage 1
  // --------------------------------------------------------------------------------
  // TLB resp (send paddr to dcache)
  val fromTlb = io.fromDataPath.tlb

  val s1Valid = RegInit(false.B)
  val s1In    = RegEnable(s0Out, s0Fire)
  val s1CanGo = s2Ready
  val s1Kill  = Wire(BOol())
  val s1Fire  = s1Valid && !s1Kill && s1CanGo
  val s1IsVec = RegEnable(s0Out.isvec, false.B, s0Fire)
  val s1VecActive = RegEnable(s0Out.vecActive, true.B, s0Fire)
  val s1FromMaBuf = s1In.isFrmMisAlignBuf
  val s1MmioCbo = s1In.uop.fuOpType === LSUOpType.cbo_clean ||
                  s1In.uop.fuOpType === LSUOpType.cbo_flush ||
                  s1In.uop.fuOpType === LSUOpType.cbo_inval
  val s1Mmio = s1MmioCbo
  val s1Pbmt = fromTlb.bits.pbmt(0)
  val s1PAddr = fromTlb.bits.paddr(0)
  val s1GPAddr = fromTlb.bits.gpaddr(0)
  val s1TlbMiss = fromTlb.bits.miss
  val s1Exception = ExceptionNo.selectByFu(s1Out.uop.exceptionVec, StaCfg).asUInt.orR

  fromTlb.ready := true.B
  s1Kill := s1In.uop.robIdx.needFlush(fromCtrlPath.redirect) ||
            (s1TlbMiss && !s1IsVec && !s1FromMaBuf)
  s1Ready := !s1Valid || s1Kill || s2Readt
  when (s0Fire) { s1Valid := true.B }
  .elsewhen (s1Fire) { s1Valid := false.B }
  .elsewhen (s1Kill) { s1Valid := false.B }

  // st-ld violation dectect request.
  toLsq.nukeQuery.valid := s1Valid && !s1TlbMiss && !s1In.isHWPrefetch && !s1FromMaBuf
  toLsq.nukeQuery.bits.robIdx := s1In.uop.robIdx
  toLsq.nukeQuery.bits.paddr  := s1PAddr
  toLsq.nukeQuery.bits.mask   := s1In.mask
  toLsq.nukeQuery.bits.matchLine := s1In.isvec && s1In.is128bit

  // Update LFST
  toIssue.lfstUpdate.valid := s1Valid && !s1TlbMiss && !s1In.isHWPrefetch && !s1IsVec && !s1FromMaBuf
  toIssue.lfstUpdate.bits  := RegEnable(s0IntBits, s0Valid)

  val s1Out   = Wire(new LsPipelineBundle)
  s1Out := s1In
  s1Out.paddr  := s1PAddr
  s1Out.gpaddr := s1GPAddr
  s1Out.miss   := false.B
  s1Out.mmio   := s1Mmio
  s1Out.atomic  := s1Mmio
  s1Out.tlbMiss := s1TlbMiss
  s1Out.uop.exceptionVec(storePageFault)   := fromTlb.bits.excp(0).pf.st && s1VecActive
  s1Out.uop.exceptionVec(storeAccessFault) := fromTlb.bits.excp(0).af.st && s1VecActive
  s1Out.uop.exceptionVec(storeGuestPageFault) := fromTlb.bits.excp(0).gpf.st && s1VecActive

  // Send TLB feedback to store issue queue
  // Store feedback is generated in store_s1, sent to RS in store_s2

}

class LoadUnitImp(override val wrapper: MemUnit)(implicit p: Parameters, params: MemUnitParams)
  extends MemUnitImp(wrapper)
{
  io.suggestName("none")
  override lazy val io = IO(new MemUnitIO()).suggestName("io")
}

class HybridUnitImp(override val wrapper: MemUnit)(implicit p: Parameters, params: MemUnitParams)
  extends MemUnitImp(wrapper)
{
  io.suggestName("none")
  override lazy val io = IO(new MemUnitIO()).suggestName("io")
}

class AtomicsUnitImp(override val wrapper: MemUnit)(implicit p: Parameters, params: MemUnitParams)
  extends MemUnitImp(wrapper)
{
  io.suggestName("none")
  override lazy val io = IO(new MemUnitIO()).suggestName("io")
}