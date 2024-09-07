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