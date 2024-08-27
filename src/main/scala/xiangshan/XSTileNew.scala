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

import org.chipsalliance.cde.config.{Config, Parameters}
import chisel3._
import chisel3.util.{Valid, ValidIO, log2Up}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.interrupts._
import freechips.rocketchip.tile.{BusErrorUnit, BusErrorUnitParams, BusErrors}
import freechips.rocketchip.tilelink._
import freechips.rocketchip.amba.axi4._
import device.MsiInfoBundle
import system.HasSoCParameter
import top.{BusPerfMonitor, ArgParser, Generator}
import utility.{DelayN, ResetGen, TLClientsMerger, TLEdgeBuffer, TLLogger, Constantin, ChiselDB, FileRegisters}
import coupledL2.EnableCHI
import coupledL2.tl2chi.PortIO

class XSTileNew()(implicit p: Parameters) extends LazyModule
  with HasXSParameter
  with HasSoCParameter
  with HasXSDts
{
  override def shouldBeInlined: Boolean = false
  val xstile0 = LazyModule(new XSTile0())
  val xstile1 = LazyModule(new XSTile1())

  xstile1.memBlock.frontendBridge.icache_node := xstile0.frontend.icache.clientNode
  xstile1.memBlock.frontendBridge.instr_uncache_node := xstile0.frontend.instrUncache.clientNode

  val enableL2 = coreParams.L2CacheParamsOpt.isDefined
  // =========== Public Ports ============
  val core_l3_pf_port = xstile1.memBlock.l3_pf_sender_opt
  val memory_port = if (enableCHI && enableL2) None else Some(xstile1.l2top.memory_port.get)
  val tl_uncache = xstile1.l2top.mmio_port
  // val axi4_uncache = if (enableCHI) Some(AXI4UserYanker()) else None
  val beu_int_source = xstile1.l2top.beu.intNode
  val core_reset_sink = BundleBridgeSink(Some(() => Reset()))
  val clint_int_node = xstile1.l2top.clint_int_node
  val plic_int_node = xstile1.l2top.plic_int_node
  val debug_int_node = xstile1.l2top.debug_int_node
  xstile1.memBlock.clint_int_sink := clint_int_node
  xstile1.memBlock.plic_int_sink :*= plic_int_node
  xstile1.memBlock.debug_int_sink := debug_int_node

  // =========== Components' Connection ============
  // L1 to l1_xbar
  coreParams.dcacheParametersOpt.map { _ =>
    xstile1.l2top.misc_l2_pmu := xstile1.l2top.l1d_logger := xstile1.memBlock.dcache_port :=
      xstile1.memBlock.l1d_to_l2_buffer.node := xstile1.memBlock.dcache.clientNode
  }

  xstile1.l2top.misc_l2_pmu := xstile1.l2top.l1i_logger := xstile1.memBlock.frontendBridge.icache_node
  if (!coreParams.softPTW) {
    xstile1.l2top.misc_l2_pmu := xstile1.l2top.ptw_logger := xstile1.l2top.ptw_to_l2_buffer.node := xstile1.memBlock.ptw_to_l2_buffer.node
  }

  // L2 Prefetch
  xstile1.l2top.l2cache match {
    case Some(l2) =>
      l2.pf_recv_node.foreach(recv => {
        println("Connecting L1 prefetcher to L2!")
        recv := xstile1.memBlock.l2_pf_sender_opt.get
      })
    case None =>
  }

  // CMO
  xstile1.l2top.l2cache match {
    case Some(l2) =>
      l2.cmo_sink_node.foreach(recv => {
        recv := xstile1.memBlock.cmo_sender.get
      })
      l2.cmo_source_node.foreach(resp => {
        xstile1.memBlock.cmo_reciver.get := resp
      })
    case None =>
  }

  val core_l3_tpmeta_source_port = xstile1.l2top.l2cache match {
    case Some(l2) => l2.tpmeta_source_node
    case None => None
  }
  val core_l3_tpmeta_sink_port = xstile1.l2top.l2cache match {
    case Some(l2) => l2.tpmeta_sink_node
    case None => None
  }

  // mmio
  xstile1.l2top.i_mmio_port := xstile1.l2top.i_mmio_buffer.node := xstile1.memBlock.frontendBridge.instr_uncache_node
  xstile1.l2top.d_mmio_port := xstile1.memBlock.uncache.clientNode

  // =========== IO Connection ============
  class XSTileNewImp(wrapper: LazyModule) extends LazyModuleImp(wrapper) {
    val io = IO(new Bundle {
      val hartId = Input(UInt(hartIdLen.W))
      val msiInfo = Input(ValidIO(new MsiInfoBundle))
      val reset_vector = Input(UInt(PAddrBits.W))
      val cpu_halt = Output(Bool())
      val debugTopDown = new Bundle {
        val robHeadPaddr = Valid(UInt(PAddrBits.W))
        val l3MissMatch = Input(Bool())
      }
      val chi = if (enableCHI) Some(new PortIO) else None
      val nodeID = if (enableCHI) Some(Input(UInt(NodeIDWidth.W))) else None
      val clintTime = Input(ValidIO(UInt(64.W)))
    })

    dontTouch(io.hartId)
    dontTouch(io.msiInfo)
    if (!io.chi.isEmpty) { dontTouch(io.chi.get) }

    val core_soft_rst = core_reset_sink.in.head._1 // unused

    xstile0.module.io.fromTop := xstile1.module.io.mem_to_ooo.topToBackendBypass
    xstile0.module.io.reset_vector := xstile1.module.io.inner_reset_vector
    xstile1.module.io.ooo_to_mem.backendToTopBypass := xstile0.module.io.toTop
    xstile1.module.io.inner_l2_pf_enable := xstile0.module.io.csrCustomCtrl_l2_pf_enable // No Use
    xstile0.module.io.perfEvents <> DontCare
    xstile1.module.io.inner_beu_errors_icache <> xstile0.module.io.beu_errors_icache
    xstile0.module.io.debugTopDown2L2 <> xstile1.module.io.debugTopDown2L2
    xstile0.module.io.debugTopDown2MemBlock <> xstile1.module.io.debugTopDown2MemBlock
    xstile0.module.io.softPrefetch <> xstile1.module.io.ifetchPrefetch

    require(xstile0.module.io.mem.stIn.length == xstile1.module.io.mem_to_ooo.stIn.length)
    xstile0.module.io.mem.stIn.zip(xstile1.module.io.mem_to_ooo.stIn).foreach { case (sink, source) =>
      sink.valid := source.valid
      sink.bits := 0.U.asTypeOf(sink.bits)
      sink.bits.robIdx := source.bits.uop.robIdx
      sink.bits.ssid := source.bits.uop.ssid
      sink.bits.storeSetHit := source.bits.uop.storeSetHit
      // The other signals have not been used
    }
    xstile0.module.io.mem.memoryViolation := xstile1.module.io.mem_to_ooo.memoryViolation
    xstile0.module.io.mem.lsqEnqIO <> xstile1.module.io.ooo_to_mem.enqLsq
    xstile0.module.io.mem.sqDeq := xstile1.module.io.mem_to_ooo.sqDeq
    xstile0.module.io.mem.lqDeq := xstile1.module.io.mem_to_ooo.lqDeq
    xstile0.module.io.mem.sqDeqPtr := xstile1.module.io.mem_to_ooo.sqDeqPtr
    xstile0.module.io.mem.lqDeqPtr := xstile1.module.io.mem_to_ooo.lqDeqPtr
    xstile0.module.io.mem.lqCancelCnt := xstile1.module.io.mem_to_ooo.lqCancelCnt
    xstile0.module.io.mem.sqCancelCnt := xstile1.module.io.mem_to_ooo.sqCancelCnt
    xstile0.module.io.mem.otherFastWakeup := xstile1.module.io.mem_to_ooo.otherFastWakeup
    xstile0.module.io.mem.stIssuePtr := xstile1.module.io.mem_to_ooo.stIssuePtr
    xstile0.module.io.mem.ldaIqFeedback := xstile1.module.io.mem_to_ooo.ldaIqFeedback
    xstile0.module.io.mem.staIqFeedback := xstile1.module.io.mem_to_ooo.staIqFeedback
    xstile0.module.io.mem.hyuIqFeedback := xstile1.module.io.mem_to_ooo.hyuIqFeedback
    xstile0.module.io.mem.vstuIqFeedback := xstile1.module.io.mem_to_ooo.vstuIqFeedback
    xstile0.module.io.mem.vlduIqFeedback := xstile1.module.io.mem_to_ooo.vlduIqFeedback
    xstile0.module.io.mem.ldCancel := xstile1.module.io.mem_to_ooo.ldCancel
    xstile0.module.io.mem.wakeup := xstile1.module.io.mem_to_ooo.wakeup
    xstile0.module.io.mem.writebackLda <> xstile1.module.io.mem_to_ooo.writebackLda
    xstile0.module.io.mem.writebackSta <> xstile1.module.io.mem_to_ooo.writebackSta
    xstile0.module.io.mem.writebackHyuLda <> xstile1.module.io.mem_to_ooo.writebackHyuLda
    xstile0.module.io.mem.writebackHyuSta <> xstile1.module.io.mem_to_ooo.writebackHyuSta
    xstile0.module.io.mem.writebackStd <> xstile1.module.io.mem_to_ooo.writebackStd
    xstile0.module.io.mem.writebackVldu <> xstile1.module.io.mem_to_ooo.writebackVldu
    xstile0.module.io.mem.robLsqIO.mmio := xstile1.module.io.mem_to_ooo.lsqio.mmio
    xstile0.module.io.mem.robLsqIO.uop := xstile1.module.io.mem_to_ooo.lsqio.uop

    // memblock error exception writeback, 1 cycle after normal writeback
    xstile0.module.io.mem.s3_delayed_load_error := xstile1.module.io.mem_to_ooo.s3_delayed_load_error

    xstile0.module.io.mem.exceptionAddr.vaddr  := xstile1.module.io.mem_to_ooo.lsqio.vaddr
    xstile0.module.io.mem.exceptionAddr.gpaddr := xstile1.module.io.mem_to_ooo.lsqio.gpaddr
    xstile0.module.io.mem.lsTopdownInfo := xstile1.module.io.mem_to_ooo.lsTopdownInfo
    xstile0.module.io.mem.lqCanAccept := xstile1.module.io.mem_to_ooo.lsqio.lqCanAccept
    xstile0.module.io.mem.sqCanAccept := xstile1.module.io.mem_to_ooo.lsqio.sqCanAccept
    xstile0.module.io.fenceio.sbuffer.sbIsEmpty := xstile1.module.io.mem_to_ooo.sbIsEmpty

    xstile1.module.io.ooo_to_mem.issueLda <> xstile0.module.io.mem.issueLda
    xstile1.module.io.ooo_to_mem.issueSta <> xstile0.module.io.mem.issueSta
    xstile1.module.io.ooo_to_mem.issueStd <> xstile0.module.io.mem.issueStd
    xstile1.module.io.ooo_to_mem.issueHya <> xstile0.module.io.mem.issueHylda
    xstile1.module.io.ooo_to_mem.issueVldu <> xstile0.module.io.mem.issueVldu

    xstile1.module.io.clintTime := io.clintTime
    xstile1.module.io.msiInfo := io.msiInfo

    // By default, instructions do not have exceptions when they enter the function units.
    xstile1.module.io.ooo_to_mem.issueUops.map(_.bits.uop.clearExceptions())
    xstile1.module.io.ooo_to_mem.loadPc := xstile0.module.io.mem.loadPcRead
    xstile1.module.io.ooo_to_mem.storePc := xstile0.module.io.mem.storePcRead
    xstile1.module.io.ooo_to_mem.hybridPc := xstile0.module.io.mem.hyuPcRead
    xstile1.module.io.ooo_to_mem.flushSb := xstile0.module.io.fenceio.sbuffer.flushSb
    xstile1.module.io.ooo_to_mem.loadFastMatch := 0.U.asTypeOf(xstile1.module.io.ooo_to_mem.loadFastMatch)
    xstile1.module.io.ooo_to_mem.loadFastImm := 0.U.asTypeOf(xstile1.module.io.ooo_to_mem.loadFastImm)
    xstile1.module.io.ooo_to_mem.loadFastFuOpType := 0.U.asTypeOf(xstile1.module.io.ooo_to_mem.loadFastFuOpType)

    xstile1.module.io.ooo_to_mem.sfence <> xstile0.module.io.mem.sfence

    xstile1.module.io.ooo_to_mem.csrCtrl := xstile0.module.io.mem.csrCtrl
    xstile1.module.io.ooo_to_mem.tlbCsr  := xstile0.module.io.mem.tlbCsr
    xstile1.module.io.ooo_to_mem.lsqio.lcommit          := xstile0.module.io.mem.robLsqIO.lcommit
    xstile1.module.io.ooo_to_mem.lsqio.scommit          := xstile0.module.io.mem.robLsqIO.scommit
    xstile1.module.io.ooo_to_mem.lsqio.pendingUncacheld := xstile0.module.io.mem.robLsqIO.pendingUncacheld
    xstile1.module.io.ooo_to_mem.lsqio.pendingld        := xstile0.module.io.mem.robLsqIO.pendingld
    xstile1.module.io.ooo_to_mem.lsqio.pendingst        := xstile0.module.io.mem.robLsqIO.pendingst
    xstile1.module.io.ooo_to_mem.lsqio.pendingVst       := xstile0.module.io.mem.robLsqIO.pendingVst
    xstile1.module.io.ooo_to_mem.lsqio.commit           := xstile0.module.io.mem.robLsqIO.commit
    xstile1.module.io.ooo_to_mem.lsqio.pendingPtr       := xstile0.module.io.mem.robLsqIO.pendingPtr
    xstile1.module.io.ooo_to_mem.lsqio.pendingPtrNext   := xstile0.module.io.mem.robLsqIO.pendingPtrNext
    xstile1.module.io.ooo_to_mem.isStoreException       := xstile0.module.io.mem.isStoreException
    xstile1.module.io.ooo_to_mem.isVlsException         := xstile0.module.io.mem.isVlsException

    xstile0.module.io.ptw <> xstile1.module.io.fetch_to_mem.itlb
    xstile0.module.io.mem.debugLS := xstile1.module.io.debug_ls
    xstile0.module.io.memInfo := xstile1.module.io.memInfo
    xstile0.module.io.perfEventsLsu := xstile1.module.io.perfEventsLsu
    xstile1.module.io.redirect := xstile0.module.io.redirect
    xstile0.module.io.reset := xstile1.module.io.reset_backend

    xstile1.module.io.hartId := io.hartId
    xstile1.module.io.debugRolling := xstile0.module.io.debugRolling
    xstile1.module.io.reset_vector := io.reset_vector
    io.cpu_halt := xstile1.module.io.cpu_halt

    xstile0.module.io.perfEvents <> DontCare

    io.debugTopDown.robHeadPaddr := xstile0.module.io.debugTopDown.robHeadPaddr
    xstile0.module.io.debugTopDown.l3MissMatch := io.debugTopDown.l3MissMatch

    io.chi.foreach(_ <> xstile1.module.io.chi.get)
    xstile1.module.io.nodeID.foreach(_ := io.nodeID.get)

  }

  lazy val module = new XSTileNewImp(this)
}
