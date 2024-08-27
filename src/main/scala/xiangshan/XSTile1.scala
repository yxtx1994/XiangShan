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
import top.{ArgParser, BusPerfMonitor, Generator}
import utility.{ChiselDB, Constantin, DelayN, FileRegisters, ResetGen, TLClientsMerger, TLEdgeBuffer, TLLogger}
import coupledL2.EnableCHI
import coupledL2.tl2chi.PortIO
import utils.PerfEvent
import xiangshan.backend.ctrlblock.DebugLSIO
import xiangshan.backend._
import xiangshan.backend.rob.RobDebugRollingIO

class XSTile1()(implicit p: Parameters) extends LazyModule
  with HasXSParameter
  with HasSoCParameter
  with HasMemBlockParameters
{
  override def shouldBeInlined: Boolean = false
  val l2top = LazyModule(new L2Top())
  val memBlock = LazyModule(new MemBlock)

  val enableL2 = coreParams.L2CacheParamsOpt.isDefined

  // =========== IO Connection ============
  class XSTile1Imp(wrapper: LazyModule) extends LazyModuleImp(wrapper) {
    val io = IO(new Bundle {
      val hartId = Input(UInt(hartIdLen.W))
      val cpu_halt = Output(Bool())
      val reset_vector = Input(UInt(PAddrBits.W))
      val chi = if (enableCHI) Some(new PortIO) else None
      val nodeID = if (enableCHI) Some(Input(UInt(NodeIDWidth.W))) else None
      val debugTopDown2L2 = new Bundle {
        val robTrueCommit = Input(UInt(64.W))
        val robHeadPaddr = Flipped(Valid(UInt(PAddrBits.W)))
        val l2MissMatch = Output(Bool())
      }
      val debugTopDown2MemBlock = new Bundle {
        val robHeadVaddr = Flipped(Valid(UInt(VAddrBits.W)))
        val toCore = new MemCoreTopDownIO
      }
      val inner_hartId = Output(UInt(hartIdLen.W))
      val inner_reset_vector = Output(UInt(PAddrBits.W))
      val inner_l2_pf_enable = Input(Bool())
      val inner_beu_errors_icache = Input(new L1BusErrorUnitInfo)
      val ooo_to_mem = new ooo_to_mem
      val mem_to_ooo = new mem_to_ooo
      val fetch_to_mem = new fetch_to_mem
      val debug_ls = new DebugLSIO
      val memInfo = new Bundle {
        val sqFull = Output(Bool())
        val lqFull = Output(Bool())
        val dcacheMSHRFull = Output(Bool())
      }
      // All the signals from/to frontend/backend to/from bus will go through MemBlock
      val msiInfo   = Input(ValidIO(new MsiInfoBundle))
      val clintTime = Input(ValidIO(UInt(64.W)))
      val perfEventsLsu = Output(Vec(numCSRPCntLsu, new PerfEvent))
      val redirect = Flipped(ValidIO(new Redirect))
      val reset_backend = Output(Reset())
      val debugRolling = Flipped(new RobDebugRollingIO)
      val ifetchPrefetch = Vec(LduCnt, ValidIO(new SoftIfetchPrefetchBundle))
    })

    dontTouch(io.hartId)
    if (!io.chi.isEmpty) { dontTouch(io.chi.get) }

    l2top.module.hartId.fromTile := io.hartId
    l2top.module.cpu_halt.fromCore := memBlock.module.io.outer_cpu_halt
    l2top.module.reset_vector.fromTile := io.reset_vector
    io.cpu_halt := l2top.module.cpu_halt.toTile
    io.inner_hartId := memBlock.module.io.inner_hartId
    io.inner_reset_vector := memBlock.module.io.inner_reset_vector
    memBlock.module.io.inner_l2_pf_enable := io.inner_l2_pf_enable
    memBlock.module.io.inner_beu_errors_icache <> io.inner_beu_errors_icache
    memBlock.module.io.ooo_to_mem <> io.ooo_to_mem
    memBlock.module.io.mem_to_ooo <> io.mem_to_ooo
    memBlock.module.io.fetch_to_mem <> io.fetch_to_mem
    memBlock.module.io.fromTopToBackend.clintTime := io.clintTime
    memBlock.module.io.fromTopToBackend.msiInfo := io.msiInfo
    io.debug_ls := memBlock.module.io.debug_ls
    io.memInfo := memBlock.module.io.memInfo
    io.perfEventsLsu := memBlock.module.getPerf
    io.debugTopDown2MemBlock <> memBlock.module.io.debugTopDown
    memBlock.module.io.redirect <> io.redirect
    io.reset_backend := memBlock.module.reset_backend
    memBlock.module.io.debugRolling := io.debugRolling
    io.ifetchPrefetch <> memBlock.module.io.ifetchPrefetch
    // top -> memBlock
    memBlock.module.io.hartId := l2top.module.hartId.toCore
    memBlock.module.io.outer_reset_vector := l2top.module.reset_vector.toCore

    l2top.module.beu_errors.icache <> memBlock.module.io.outer_beu_errors_icache
    l2top.module.beu_errors.dcache <> memBlock.module.io.error.bits.toL1BusErrorUnitInfo(memBlock.module.io.error.valid)
    if (enableL2) {
      // TODO: add ECC interface of L2

      l2top.module.beu_errors.l2 <> 0.U.asTypeOf(l2top.module.beu_errors.l2)
      memBlock.module.io.l2_hint.bits.sourceId := l2top.module.l2_hint.bits.sourceId
      memBlock.module.io.l2_hint.bits.isKeyword := l2top.module.l2_hint.bits.isKeyword
      memBlock.module.io.l2_hint.valid := l2top.module.l2_hint.valid

      memBlock.module.io.l2PfqBusy := false.B
      io.debugTopDown2L2.l2MissMatch := l2top.module.debugTopDown.l2MissMatch
      l2top.module.debugTopDown.robHeadPaddr := io.debugTopDown2L2.robHeadPaddr
      l2top.module.debugTopDown.robTrueCommit := io.debugTopDown2L2.robTrueCommit
      l2top.module.l2_pmp_resp := memBlock.module.io.l2_pmp_resp
      memBlock.module.io.l2_tlb_req <> l2top.module.l2_tlb_req
    } else {

      l2top.module.beu_errors.l2 <> 0.U.asTypeOf(l2top.module.beu_errors.l2)
      memBlock.module.io.l2_hint.bits.sourceId := l2top.module.l2_hint.bits.sourceId
      memBlock.module.io.l2_hint.bits.isKeyword := l2top.module.l2_hint.bits.isKeyword
      memBlock.module.io.l2_hint.valid := l2top.module.l2_hint.valid

      memBlock.module.io.l2PfqBusy := false.B
      io.debugTopDown2L2.l2MissMatch := false.B

      memBlock.module.io.l2_tlb_req.req.valid := false.B
      memBlock.module.io.l2_tlb_req.req.bits := DontCare
      memBlock.module.io.l2_tlb_req.req_kill := DontCare
      memBlock.module.io.l2_tlb_req.resp.ready := true.B
    }

    io.chi.foreach(_ <> l2top.module.chi.get)
    l2top.module.nodeID.foreach(_ := io.nodeID.get)

    if (debugOpts.ResetGen && enableL2) {
      memBlock.module.reset := l2top.module.reset_core
    }
  }

  lazy val module = new XSTile1Imp(this)
}
