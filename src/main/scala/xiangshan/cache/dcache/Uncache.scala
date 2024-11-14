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

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import utils._
import utility._
import xiangshan._
import freechips.rocketchip.diplomacy.{IdRange, LazyModule, LazyModuleImp, TransferSizes}
import freechips.rocketchip.tilelink.{TLArbiter, TLBundleA, TLBundleD, TLClientNode, TLEdgeOut, TLMasterParameters, TLMasterPortParameters}

class UncacheFlushBundle extends Bundle {
  val valid = Output(Bool())
  val empty = Input(Bool())
}

class UncacheEntry(implicit p: Parameters) extends DCacheBundle {
  val cmd = UInt(M_SZ.W)
  val addr = UInt(PAddrBits.W)
  val data = UInt(XLEN.W)
  val mask = UInt((XLEN/8).W)
  val id = UInt(uncacheIdxBits.W)
  val nc = Bool()
  val atomic = Bool()
  
  // FIXME lyq: data and resp_data can be merged?
  val resp_data = UInt(XLEN.W)
  val resp_nderr = Bool()

  def set(x: UncacheWordReq): Unit = {
    cmd := x.cmd
    addr := x.addr
    data := x.data
    mask := x.mask
    id := x.id
    nc := x.nc
    atomic := x.atomic
    resp_nderr := false.B
    resp_data := 0.U
  }

  def update(x: TLBundleD): Unit = {
    resp_data := x.data
    resp_nderr := x.denied
  }

  def toUncacheWordResp(): UncacheWordResp = {
    val r = Wire(new UncacheWordResp)
    r := DontCare
    r.data := resp_data
    r.id := id
    r.nderr := resp_nderr
    r.nc := nc
    r.is2lq := cmd === MemoryOpConstants.M_XRD
    r.miss := false.B
    r.replay := false.B
    r.tag_error := false.B
    r.error := false.B
    r
  }
}

class UncacheEntryState(implicit p: Parameters) extends DCacheBundle {
  // FIXME lyq: state is multi bools or UInt()?
  // valid (-> waitSame) -> inflight -> waitReturn
  val valid = Bool()
  val inflight = Bool() // uncache -> L2
  val waitSame = Bool()
  val waitReturn = Bool() // uncache -> LSQ
  
  def init: Unit = {
    valid := false.B
    inflight := false.B
    waitSame := false.B
    waitReturn := false.B
  }

  def isValid(): Bool = valid
  def isInflight(): Bool = inflight
  def isWaitReturn(): Bool = waitReturn
  def isWaitSame(): Bool = waitSame
  def can2Uncache(): Bool = valid && !inflight && !waitSame && !waitReturn
  def can2Lsq(): Bool = valid && waitReturn
  
  def setValid(x: Bool): Unit = { valid := x}
  def setInflight(x: Bool): Unit = { inflight := x}
  def setWaitReturn(x: Bool): Unit = { waitReturn := x }
  def setWaitSame(x: Bool): Unit = { waitSame := x}
  
  def updateUncacheResp(): Unit = {
    assert(inflight, "The request was not sent and a response was received")
    inflight := false.B
    waitReturn := true.B
  }
  def updateReturn(): Unit = {
    valid := false.B
    inflight := false.B
    waitSame := false.B
    waitReturn := false.B
  }
}

class UncacheIO(implicit p: Parameters) extends DCacheBundle {
  val hartId = Input(UInt())
  val enableOutstanding = Input(Bool())
  val flush = Flipped(new UncacheFlushBundle)
  val lsq = Flipped(new UncacheWordIO)
}

// convert DCacheIO to TileLink
// for Now, we only deal with TL-UL

class Uncache()(implicit p: Parameters) extends LazyModule with HasXSParameter {
  override def shouldBeInlined: Boolean = false
  def idRange: Int = UncacheBufferSize

  val clientParameters = TLMasterPortParameters.v1(
    clients = Seq(TLMasterParameters.v1(
      "uncache",
      sourceId = IdRange(0, idRange)
    ))
  )
  val clientNode = TLClientNode(Seq(clientParameters))

  lazy val module = new UncacheImp(this)
}

/* Uncache Buffer */
class UncacheImp(outer: Uncache)extends LazyModuleImp(outer)
  with HasTLDump
  with HasXSParameter
  with HasPerfEvents
{
  println(s"Uncahe Buffer Size: $UncacheBufferSize entries")
  val io = IO(new UncacheIO)

  val (bus, edge) = outer.clientNode.out.head

  val req  = io.lsq.req
  val resp = io.lsq.resp
  val mem_acquire = bus.a
  val mem_grant   = bus.d
  val req_ready = WireInit(false.B)

  // assign default values to output signals
  bus.b.ready := false.B
  bus.c.valid := false.B
  bus.c.bits  := DontCare
  bus.d.ready := false.B
  bus.e.valid := false.B
  bus.e.bits  := DontCare
  io.lsq.req.ready := req_ready
  io.lsq.resp.valid := false.B
  io.lsq.resp.bits := DontCare


  /******************************************************************
   * Data Structure
   ******************************************************************/

  val entries = Reg(Vec(UncacheBufferSize, new UncacheEntry))
  val states = RegInit(VecInit(Seq.fill(UncacheBufferSize)(0.U.asTypeOf(new UncacheEntryState))))
  val fence = RegInit(Bool(), false.B)
  val s_idle :: s_refill_req :: s_refill_resp :: s_send_resp :: Nil = Enum(4)
  val uState = RegInit(s_idle)
  
  def sizeMap[T <: Data](f: Int => T) = VecInit((0 until UncacheBufferSize).map(f))


  /******************************************************************
   * uState for non-outstanding
   ******************************************************************/

  switch(uState){
    is(s_idle){
      when(req.fire){
        uState := s_refill_req
      }
    }
    is(s_refill_req){
      when(mem_acquire.fire){
        uState := s_refill_resp
      }
    }
    is(s_refill_resp){
      when(mem_grant.fire){
        uState := s_send_resp
      }
    }
    is(s_send_resp){
      when(resp.fire){
        uState := s_idle
      }
    }
  }


  /******************************************************************
   * Enter Buffer
   ******************************************************************/

  /** s0 judge: alloc/merge 
    TODO lyq: how to merge
    1. same addr
    2. same cmd
    3. valid
    FIXME lyq: not merge now due to the following issues
    1. load cann't be merged
    2. how to merge store and response precisely
  */
  val e1_alloc = Wire(Vec(UncacheBufferSize, Bool()))

  val e0_invalids = sizeMap(i => !states(i).isValid() && !e1_alloc(i))
  val e0_invalid_oh = VecInit(PriorityEncoderOH(e0_invalids)).asUInt
  val e0_fire = req.fire
  req_ready := e0_invalid_oh.orR
  
  /* s1 alloc */
  val e1_valid = RegNext(e0_fire)
  val e1_invalid_oh = RegEnable(e0_invalid_oh, e0_fire)
  val e1_req = RegEnable(req.bits, e0_fire)
  for (i <- 0 until UncacheBufferSize) {
    e1_alloc(i) := e1_valid && e1_invalid_oh(i)
    when(e1_alloc(i)){
      entries(i).set(e1_req)
      states(i).setValid(true.B)
      
      // e1 should judge whether wait same block
      val waitSameVec = sizeMap(j => 
        e1_req.addr === entries(j).addr && states(j).isValid() && states(j).isInflight()
      )
      when (waitSameVec.reduce(_ || _)) {
        states(i).setWaitSame(true.B)
      }
    }
  }


  /******************************************************************
   * Uncache Req
   ******************************************************************/

  /* q0: choose which one is sent */
  val q1_canSentIdx = Wire(UInt())
  val q1_canSent = Wire(Bool())

  val q0_canSentVec = sizeMap(i => 
    // (io.enableOutstanding || uState === s_refill_req) && // FIXME lyq: comment for debug
    states(i).can2Uncache() && 
    !(i.U === q1_canSentIdx && q1_canSent)
  )
  val (q0_canSentIdx, q0_canSent) = PriorityEncoderWithFlag(q0_canSentVec)

  /* q1: sent  */
  q1_canSent := RegNext(q0_canSent)
  q1_canSentIdx := RegEnable(q0_canSentIdx, q0_canSent)
  val q1_entry = entries(q1_canSentIdx)

  val size = PopCount(q1_entry.mask)
  val (lgSize, legal) = PriorityMuxWithFlag(Seq(
    1.U -> 0.U,
    2.U -> 1.U,
    4.U -> 2.U,
    8.U -> 3.U
  ).map(m => (size===m._1) -> m._2))
  assert(!(q1_canSent && !legal))

  val q1_load = edge.Get(
    fromSource      = q1_canSentIdx,
    toAddress       = q1_entry.addr,
    lgSize          = lgSize
  )._2

  val q1_store = edge.Put(
    fromSource      = q1_canSentIdx,
    toAddress       = q1_entry.addr,
    lgSize          = lgSize,
    data            = q1_entry.data,
    mask            = q1_entry.mask
  )._2

  val q1_isStore = q1_entry.cmd === MemoryOpConstants.M_XWR

  mem_acquire.valid := q1_canSent
  mem_acquire.bits := Mux(q1_isStore, q1_store, q1_load)
  when(mem_acquire.fire){
    states(q1_canSentIdx).setInflight(true.B)

    // q1 should judge whether wait same block
    (0 until UncacheBufferSize).map(j => 
      when(q1_entry.addr === entries(j).addr && states(j).isValid() && !states(j).isWaitReturn()){
        states(j).setWaitSame(true.B)
      }
    )
  }


  /******************************************************************
   * Uncache Resp
   ******************************************************************/

  val (_, _, refill_done, _) = edge.addr_inc(mem_grant)
  
  mem_grant.ready := true.B
  when (mem_grant.fire) {
    val id = mem_grant.bits.source
    entries(id).update(mem_grant.bits)
    states(id).updateUncacheResp()
    assert(refill_done, "Uncache response should be one beat only!")

    // remove state of wait same block
    (0 until UncacheBufferSize).map(j => 
      when(entries(id).addr === entries(j).addr && states(j).isValid() && states(j).isWaitSame()){
        states(j).setWaitSame(false.B)
      }
    )
  }


  /******************************************************************
   * Return to LSQ
   ******************************************************************/

  val r0_canSentVec = sizeMap(i => states(i).can2Lsq())
  val (r0_canSentIdx, r0_canSent) = PriorityEncoderWithFlag(r0_canSentVec)
  resp.valid := r0_canSent
  resp.bits := entries(r0_canSentIdx).toUncacheWordResp()
  when(resp.fire){
    states(r0_canSentIdx).updateReturn()
  }


  /******************************************************************
   * Buffer Flush
   * // FIXME lyq: how to deal
   * 1. when io.flush.valid is true
   * 2. when io.lsq.req.bits.atomic is true
   ******************************************************************/

  val invalid_entries = PopCount(states.map(!_.isValid()))
  io.flush.empty := invalid_entries === UncacheBufferSize.U


  /******************************************************************
   * Debug / Performance
   ******************************************************************/

  /* Debug Counters */
  // print all input/output requests for debug purpose
  // print req/resp
  XSDebug(req.fire, "req cmd: %x addr: %x data: %x mask: %x\n",
    req.bits.cmd, req.bits.addr, req.bits.data, req.bits.mask)
  XSDebug(resp.fire, "data: %x\n", req.bits.data)
  // print tilelink messages
  when(mem_acquire.valid){
    XSDebug("mem_acquire valid, ready=%d ", mem_acquire.ready)
    mem_acquire.bits.dump
  }
  when (mem_grant.fire) {
    XSDebug("mem_grant fire ")
    mem_grant.bits.dump
  }

  /* Performance Counters */
  def isStore: Bool = io.lsq.req.bits.cmd === MemoryOpConstants.M_XWR
  XSPerfAccumulate("uncache_mmio_store", io.lsq.req.fire && isStore && !io.lsq.req.bits.nc)
  XSPerfAccumulate("uncache_mmio_load", io.lsq.req.fire && !isStore && !io.lsq.req.bits.nc)
  XSPerfAccumulate("uncache_nc_store", io.lsq.req.fire && isStore && io.lsq.req.bits.nc)
  XSPerfAccumulate("uncache_nc_load", io.lsq.req.fire && !isStore && io.lsq.req.bits.nc)
  XSPerfAccumulate("uncache_outstanding", uState =/= s_refill_req && mem_acquire.fire)
  
  val perfEvents = Seq(
    ("uncache_mmio_store", io.lsq.req.fire && isStore && !io.lsq.req.bits.nc),
    ("uncache_mmio_load", io.lsq.req.fire && !isStore && !io.lsq.req.bits.nc),
    ("uncache_nc_store", io.lsq.req.fire && isStore && io.lsq.req.bits.nc),
    ("uncache_nc_load", io.lsq.req.fire && !isStore && io.lsq.req.bits.nc),
    ("uncache_outstanding", uState =/= s_refill_req && mem_acquire.fire)
  )

  generatePerfEvent()
  //  End
}
