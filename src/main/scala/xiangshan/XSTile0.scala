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

package xiangshan

import org.chipsalliance.cde.config
import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import device.MsiInfoBundle
import freechips.rocketchip.diplomacy.{BundleBridgeSource, LazyModule, LazyModuleImp}
import freechips.rocketchip.tile.HasFPUParameters
import system.HasSoCParameter
import utils._
import utility._
import xiangshan.backend._
import xiangshan.backend.ctrlblock.DebugLSIO
import xiangshan.backend.fu.{FenceToSbuffer, PMPRespBundle}
import xiangshan.backend.rob.RobDebugRollingIO
import xiangshan.backend.Bundles._
import xiangshan.cache.mmu._
import xiangshan.frontend._
import xiangshan.mem.L1PrefetchFuzzer

import scala.collection.mutable.ListBuffer
import xiangshan.cache.mmu.TlbRequestIO

abstract class XSModule(implicit val p: Parameters) extends Module
  with HasXSParameter
  with HasFPUParameters

//remove this trait after impl module logic
trait NeedImpl {
  this: RawModule =>
  protected def IO[T <: Data](iodef: T): T = {
    println(s"[Warn]: (${this.name}) please reomve 'NeedImpl' after implement this module")
    val io = chisel3.IO(iodef)
    io <> DontCare
    io
  }
}

abstract class XSBundle(implicit val p: Parameters) extends Bundle
  with HasXSParameter

abstract class XSTile0Base()(implicit p: config.Parameters) extends LazyModule
  with HasXSParameter
{
  override def shouldBeInlined: Boolean = false
  // outer facing nodes
  val frontend = LazyModule(new Frontend())
  val csrOut = BundleBridgeSource(Some(() => new DistributedCSRIO()))
  val backend = LazyModule(new Backend(backendParams))
}

class XSTile0()(implicit p: config.Parameters) extends XSTile0Base
{
  lazy val module = new XSTile0Imp(this)
}

class XSTile0Imp(outer: XSTile0Base) extends LazyModuleImp(outer)
  with HasXSParameter
  with HasSoCParameter {
  implicit private val params: BackendParams = outer.backend.params
  val io = IO(new Bundle {
    val fromTop = new Bundle {
      val hartId = Input(UInt(hartIdLen.W))
      val externalInterrupt = Input(new ExternalInterruptIO)
      val msiInfo = Input(ValidIO(new MsiInfoBundle))
      val clintTime = Input(ValidIO(UInt(64.W)))
    }
    val reset_vector = Input(UInt(PAddrBits.W))
    val csrCustomCtrl_l2_pf_enable = Output(Bool())
    val perfEvents = Input(Vec(numPCntHc * coreParams.L2NBanks, new PerfEvent))
    val beu_errors_icache = Output(new L1BusErrorUnitInfo)
    val debugTopDown = new Bundle {
      val robHeadPaddr = Valid(UInt(PAddrBits.W))
      val l3MissMatch = Input(Bool())
    }
    val debugTopDown2L2 = new Bundle {
      val robTrueCommit = Output(UInt(64.W))
      val robHeadPaddr = Valid(UInt(PAddrBits.W))
      val l2MissMatch = Input(Bool())
    }
    val debugTopDown2MemBlock = new Bundle {
      val robHeadVaddr = Valid(UInt(VAddrBits.W))
      val toCore = Flipped(new MemCoreTopDownIO)
    }
    val mem = new BackendMemIO
    val fenceio = new Bundle {
      val sbuffer = new FenceToSbuffer
    }
    val ptw = new TlbPtwIO()
    val reset = Input(Reset())
    val memInfo = new Bundle {
      val sqFull = Input(Bool())
      val lqFull = Input(Bool())
      val dcacheMSHRFull = Input(Bool())
    }
    val perfEventsLsu = Input(Vec(numCSRPCntLsu, new PerfEvent))
    val redirect = ValidIO(new Redirect)
    val debugRolling = new RobDebugRollingIO
    val softPrefetch = Vec(backendParams.LduCnt, Flipped(Valid(new SoftIfetchPrefetchBundle)))
    val toTop = new BackendToTopBundle
  })

  println(s"FPGAPlatform:${env.FPGAPlatform} EnableDebug:${env.EnableDebug}")

  val frontend = outer.frontend.module
  val backend = outer.backend.module

  frontend.io.hartId := io.fromTop.hartId
  frontend.io.reset_vector := io.reset_vector
  frontend.io.backend <> backend.io.frontend
  frontend.io.sfence <> backend.io.frontendSfence
  frontend.io.tlbCsr <> backend.io.frontendTlbCsr
  frontend.io.csrCtrl <> backend.io.frontendCsrCtrl
  frontend.io.fencei <> backend.io.fenceio.fencei
  frontend.io.softPrefetch <> io.softPrefetch

  backend.io.fromTop := io.fromTop
  io.toTop := backend.io.toTop

  backend.io.mem <> io.mem

  backend.io.fenceio.sbuffer.sbIsEmpty := io.fenceio.sbuffer.sbIsEmpty
  io.fenceio.sbuffer.flushSb := backend.io.fenceio.sbuffer.flushSb

  backend.io.perf.frontendInfo := frontend.io.frontendInfo
  backend.io.perf.perfEventsFrontend := frontend.getPerf
  backend.io.perf.perfEventsHc := io.perfEvents
  backend.io.perf.perfEventsBackend := DontCare
  backend.io.perf.retiredInstr := DontCare
  backend.io.perf.ctrlInfo := DontCare

  backend.io.fromTop.externalInterrupt := io.fromTop.externalInterrupt
  backend.io.mem.debugLS := io.mem.debugLS
  backend.io.perf.memInfo := io.memInfo
  backend.io.perf.perfEventsLsu := io.perfEventsLsu
  io.redirect <> backend.io.mem.redirect
  io.debugRolling := backend.io.debugRolling

  // frontend -> memBlock
  io.beu_errors_icache <> frontend.io.error.bits.toL1BusErrorUnitInfo(frontend.io.error.valid)
  io.csrCustomCtrl_l2_pf_enable := backend.io.csrCustomCtrl.l2_pf_enable

  backend.io.mem.issueHysta.foreach(_.ready := false.B) // this fake port should not be used

  io.ptw <> frontend.io.ptw

  // if l2 prefetcher use stream prefetch, it should be placed in XSTile0

  // top-down info
  io.debugTopDown2MemBlock.robHeadVaddr := backend.io.debugTopDown.fromRob.robHeadVaddr
  backend.io.debugTopDown.fromCore.fromMem := io.debugTopDown2MemBlock.toCore
  frontend.io.debugTopDown.robHeadVaddr := backend.io.debugTopDown.fromRob.robHeadVaddr
  io.debugTopDown.robHeadPaddr := backend.io.debugTopDown.fromRob.robHeadPaddr
  backend.io.debugTopDown.fromCore.l3MissMatch := io.debugTopDown.l3MissMatch
  io.debugTopDown2L2.robHeadPaddr := backend.io.debugTopDown.fromRob.robHeadPaddr
  io.debugTopDown2L2.robTrueCommit := backend.io.debugRolling.robTrueCommit
  backend.io.debugTopDown.fromCore.l2MissMatch := io.debugTopDown2L2.l2MissMatch

  if (debugOpts.ResetGen) {
    backend.reset := io.reset
    frontend.reset := backend.io.frontendReset
  }
}
