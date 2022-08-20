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
import freechips.rocketchip.tilelink.ClientMetadata
import utils.{HasPerfEvents, XSDebug, XSPerfAccumulate}
import xiangshan.L1CacheErrorInfo

class LoadPipe(id: Int)(implicit p: Parameters) extends DCacheModule with HasPerfEvents {
  val io = IO(new DCacheBundle {
    // incoming requests
    val lsu = Flipped(new DCacheLoadIO)
    // req got nacked in stage 0?
    val nack      = Input(Bool())

    // meta and data array read port
    val meta_read = DecoupledIO(new MetaReadReq)
    val meta_resp = Input(Vec(nWays, new Meta))
    val error_flag_resp = Input(Vec(nWays, Bool()))

    val tag_read = DecoupledIO(new TagReadReq)
    val tag_resp = Input(Vec(nWays, UInt(encTagBits.W)))

    val banked_data_read = DecoupledIO(new L1BankedDataReadReq)
    val banked_data_resp = Input(Vec(DCacheBanks, new L1BankedDataReadResult()))
    val read_error_delayed = Input(Bool())

    // banked data read conflict
    val bank_conflict_slow = Input(Bool())
    val bank_conflict_fast = Input(Bool())

    // send miss request to miss queue
    val miss_req    = DecoupledIO(new MissReq)

    // update state vec in replacement algo
    val replace_access = ValidIO(new ReplacementAccessBundle)
    // find the way to be replaced
    val replace_way = new ReplacementWayReqIO

    // load fast wakeup should be disabled when data read is not ready
    val disable_ld_fast_wakeup = Input(Bool())

    // ecc error
    val error = Output(new L1CacheErrorInfo())
  })

  assert(RegNext(io.meta_read.ready))

  val s1_ready = Wire(Bool())
  val s2_ready = Wire(Bool())
  // LSU requests
  // it you got nacked, you can directly passdown
  val not_nacked_ready = io.meta_read.ready && io.tag_read.ready && s1_ready
  val nacked_ready     = true.B

  // ready can wait for valid
  io.lsu.req.ready := (!io.nack && not_nacked_ready) || (io.nack && nacked_ready)
  io.meta_read.valid := io.lsu.req.fire() && !io.nack
  io.tag_read.valid := io.lsu.req.fire() && !io.nack

  val meta_read = io.meta_read.bits
  val tag_read = io.tag_read.bits

  // Tag read for new requests
  meta_read.idx := get_idx(io.lsu.req.bits.addr)
  meta_read.way_en := ~0.U(nWays.W)
  // meta_read.tag := DontCare

  tag_read.idx := get_idx(io.lsu.req.bits.addr)
  tag_read.way_en := ~0.U(nWays.W)

  // Pipeline
  // --------------------------------------------------------------------------------
  // stage 0
  // --------------------------------------------------------------------------------
  // read tag

  val s0_valid = io.lsu.req.fire()
  val s0_req = io.lsu.req.bits
  val s0_fire = s0_valid && s1_ready

  assert(RegNext(!(s0_valid && (s0_req.cmd =/= MemoryOpConstants.M_XRD && s0_req.cmd =/= MemoryOpConstants.M_PFR && s0_req.cmd =/= MemoryOpConstants.M_PFW))), "LoadPipe only accepts load req / softprefetch read or write!")
  dump_pipeline_reqs("LoadPipe s0", s0_valid, s0_req)

  // --------------------------------------------------------------------------------
  // stage 1
  // --------------------------------------------------------------------------------
  // tag match, read data

  val s1_valid = RegInit(false.B)
  val s1_req = RegEnable(s0_req, s0_fire)
  // in stage 1, load unit gets the physical address
  val s1_paddr_dup_lsu = io.lsu.s1_paddr_dup_lsu
  val s1_paddr_dup_dcache = io.lsu.s1_paddr_dup_dcache
  val s1_vaddr = Cat(s1_req.addr(PAddrBits - 1, blockOffBits), io.lsu.s1_paddr_dup_lsu(blockOffBits - 1, 0))
  val s1_bank_oh = UIntToOH(addr_to_dcache_bank(s1_vaddr))
  val s1_nack = RegNext(io.nack)
  val s1_nack_data = !io.banked_data_read.ready
  val s1_fire = s1_valid && s2_ready
  s1_ready := !s1_valid || s1_fire

  when (s0_fire) { s1_valid := true.B }
  .elsewhen (s1_fire) { s1_valid := false.B }

  dump_pipeline_reqs("LoadPipe s1", s1_valid, s1_req)

  // tag check
  val meta_resp = io.meta_resp
  val tag_resp = io.tag_resp.map(r => r(tagBits - 1, 0))
  def wayMap[T <: Data](f: Int => T) = VecInit((0 until nWays).map(f))

  // dcache side tag match
  val s1_tag_eq_way = wayMap((w: Int) => tag_resp(w) === (get_tag(s1_paddr_dup_dcache))).asUInt
  val s1_tag_match_way = wayMap((w: Int) => s1_tag_eq_way(w) && meta_resp(w).coh.isValid()).asUInt
  val s1_tag_match = s1_tag_match_way.orR
  assert(RegNext(!s1_valid || PopCount(s1_tag_match_way) <= 1.U), "tag should not match with more than 1 way")

  val s1_fake_meta = Wire(new Meta)
//  s1_fake_meta.tag := get_tag(s1_paddr_dup_dcache)
  s1_fake_meta.coh := ClientMetadata.onReset
  val s1_fake_tag = get_tag(s1_paddr_dup_dcache)

  // when there are no tag match, we give it a Fake Meta
  // this simplifies our logic in s2 stage
  val s1_hit_meta = Mux(s1_tag_match, Mux1H(s1_tag_match_way, wayMap((w: Int) => meta_resp(w))), s1_fake_meta)
  val s1_hit_coh = s1_hit_meta.coh
  val s1_hit_error = Mux(s1_tag_match, Mux1H(s1_tag_match_way, wayMap((w: Int) => io.error_flag_resp(w))), false.B)

  io.replace_way.set.valid := RegNext(s0_fire)
  io.replace_way.set.bits := get_idx(s1_vaddr)
  val s1_repl_way_en = UIntToOH(io.replace_way.way)
  val s1_repl_tag = Mux1H(s1_repl_way_en, wayMap(w => tag_resp(w)))
  val s1_repl_coh = Mux1H(s1_repl_way_en, wayMap(w => meta_resp(w).coh))

  val s1_need_replacement = !s1_tag_match
  val s1_way_en = Mux(s1_need_replacement, s1_repl_way_en, s1_tag_match_way)
  val s1_coh = Mux(s1_need_replacement, s1_repl_coh, s1_hit_coh)
  val s1_tag = Mux(s1_need_replacement, s1_repl_tag, get_tag(s1_paddr_dup_dcache))

  // data read
  io.banked_data_read.valid := s1_fire && !s1_nack
  io.banked_data_read.bits.addr := s1_vaddr
  io.banked_data_read.bits.way_en := s1_tag_match_way

  // get s1_will_send_miss_req in lpad_s1
  val s1_has_permission = s1_hit_coh.onAccess(s1_req.cmd)._1
  val s1_new_hit_coh = s1_hit_coh.onAccess(s1_req.cmd)._3
  val s1_hit = s1_tag_match && s1_has_permission && s1_hit_coh === s1_new_hit_coh
  val s1_will_send_miss_req = s1_valid && !s1_nack && !s1_nack_data && !s1_hit

  // check ecc error
  val s1_encTag = Mux1H(s1_tag_match_way, wayMap((w: Int) => io.tag_resp(w)))
  val s1_flag_error = Mux(s1_need_replacement, false.B, s1_hit_error) // error reported by exist dcache error bit

  // --------------------------------------------------------------------------------
  // stage 2
  // --------------------------------------------------------------------------------
  // return data

  // val s2_valid = RegEnable(next = s1_valid && !io.lsu.s1_kill, init = false.B, enable = s1_fire)
  val s2_valid = RegInit(false.B)
  val s2_req = RegEnable(s1_req, s1_fire)
  val s2_paddr = RegEnable(s1_paddr_dup_dcache, s1_fire)
  val s2_vaddr = RegEnable(s1_vaddr, s1_fire)
  val s2_bank_oh = RegEnable(s1_bank_oh, s1_fire)
  val s2_bank_oh_dup_0 = RegEnable(s1_bank_oh, s1_fire)
  s2_ready := true.B

  val s2_fire = s2_valid

  when (s1_fire) { s2_valid := !io.lsu.s1_kill }
  .elsewhen(io.lsu.resp.fire()) { s2_valid := false.B }

  dump_pipeline_reqs("LoadPipe s2", s2_valid, s2_req)

  // hit, miss, nack, permission checking
  // dcache side tag match
  val s2_tag_match_way = RegEnable(s1_tag_match_way, s1_fire)
  val s2_tag_match = RegEnable(s1_tag_match, s1_fire)

  // lsu side tag match
  val s2_tag_eq_way_dup_lsu = wayMap((w: Int) => RegNext(tag_resp(w)) === RegNext((get_tag(s1_paddr_dup_lsu)))).asUInt
  val s2_tag_match_way_dup_lsu = wayMap((w: Int) => s2_tag_eq_way_dup_lsu(w) && RegNext(meta_resp(w).coh.isValid())).asUInt
  val s2_tag_match_dup_lsu = s2_tag_match_way_dup_lsu.orR
  val s2_hit_dup_lsu = s2_tag_match_dup_lsu

  io.lsu.s2_hit := s2_hit_dup_lsu

  val s2_hit_meta = RegEnable(s1_hit_meta, s1_fire)
  val s2_hit_coh = RegEnable(s1_hit_coh, s1_fire)
  val s2_has_permission = s2_hit_coh.onAccess(s2_req.cmd)._1 // redundant
  val s2_new_hit_coh = s2_hit_coh.onAccess(s2_req.cmd)._3 // redundant

  val s2_way_en = RegEnable(s1_way_en, s1_fire)
  val s2_repl_coh = RegEnable(s1_repl_coh, s1_fire)
  val s2_repl_tag = RegEnable(s1_repl_tag, s1_fire)
  val s2_encTag = RegEnable(s1_encTag, s1_fire)

  // when req got nacked, upper levels should replay this request
  // nacked or not
  val s2_nack_hit = RegEnable(s1_nack, s1_fire)
  // can no allocate mshr for load miss
  val s2_nack_no_mshr = io.miss_req.valid && !io.miss_req.ready
  // Bank conflict on data arrays
  val s2_nack_data = RegEnable(!io.banked_data_read.ready, s1_fire)
  val s2_nack = s2_nack_hit || s2_nack_no_mshr || s2_nack_data

  val banked_data_resp = io.banked_data_resp
  val s2_bank_addr = addr_to_dcache_bank(s2_paddr)
  val banked_data_resp_word = Mux1H(s2_bank_oh, io.banked_data_resp) // io.banked_data_resp(s2_bank_addr)
  val banked_data_resp_word_dup_0 = Mux1H(s2_bank_oh_dup_0, io.banked_data_resp) // io.banked_data_resp(s2_bank_addr)
  dontTouch(s2_bank_addr)

  val s2_instrtype = s2_req.instrtype

  val s2_tag_error = dcacheParameters.tagCode.decode(s2_encTag).error // error reported by tag ecc check
  val s2_flag_error = RegEnable(s1_flag_error, s1_fire)

  val s2_hit = s2_tag_match && s2_has_permission && s2_hit_coh === s2_new_hit_coh
  assert(!RegNext(s2_valid && (s2_tag_match && !s2_hit)))
  assert(!RegNext(s2_valid && (s2_hit_dup_lsu =/= s2_hit)))

  // only dump these signals when they are actually valid
  dump_pipeline_valids("LoadPipe s2", "s2_hit", s2_valid && s2_hit)
  dump_pipeline_valids("LoadPipe s2", "s2_nack", s2_valid && s2_nack)
  dump_pipeline_valids("LoadPipe s2", "s2_nack_hit", s2_valid && s2_nack_hit)
  dump_pipeline_valids("LoadPipe s2", "s2_nack_no_mshr", s2_valid && s2_nack_no_mshr)

  val s2_can_send_miss_req = RegEnable(s1_will_send_miss_req, s1_fire)

  // send load miss to miss queue
  io.miss_req.valid := s2_valid && s2_can_send_miss_req
  io.miss_req.bits := DontCare
  io.miss_req.bits.source := s2_instrtype
  io.miss_req.bits.cmd := s2_req.cmd
  io.miss_req.bits.addr := get_block_addr(s2_paddr)
  io.miss_req.bits.vaddr := s2_vaddr
  io.miss_req.bits.way_en := s2_way_en
  io.miss_req.bits.req_coh := s2_hit_coh
  io.miss_req.bits.replace_coh := s2_repl_coh
  io.miss_req.bits.replace_tag := s2_repl_tag
  io.miss_req.bits.cancel := io.lsu.s2_kill || s2_tag_error

  // send back response
  val resp = Wire(ValidIO(new DCacheWordResp))
  resp.valid := s2_valid
  resp.bits := DontCare
  // resp.bits.data := s2_word_decoded
  resp.bits.data := banked_data_resp_word.raw_data
  resp.bits.data_dup_0 := banked_data_resp_word_dup_0.raw_data
  // * on miss or nack, upper level should replay request
  // but if we successfully sent the request to miss queue
  // upper level does not need to replay request
  // they can sit in load queue and wait for refill
  //
  // * report a miss if bank conflict is detected
  val real_miss = !s2_hit_dup_lsu
  resp.bits.miss := real_miss || io.bank_conflict_slow
  // load pipe need replay when there is a bank conflict
  resp.bits.replay := resp.bits.miss && (!io.miss_req.fire() || s2_nack) || io.bank_conflict_slow
  resp.bits.tag_error := s2_tag_error // report tag_error in load s2

  XSPerfAccumulate("dcache_read_bank_conflict", io.bank_conflict_slow && s2_valid)

  io.lsu.resp.valid := resp.valid
  io.lsu.resp.bits := resp.bits
  assert(RegNext(!(resp.valid && !io.lsu.resp.ready)), "lsu should be ready in s2")

  when (resp.valid) {
    resp.bits.dump()
  }

  io.lsu.debug_s1_hit_way := s1_tag_match_way
  io.lsu.s1_disable_fast_wakeup := io.disable_ld_fast_wakeup
  io.lsu.s1_bank_conflict := io.bank_conflict_fast
  assert(RegNext(s1_ready && s2_ready), "load pipeline should never be blocked")

  // --------------------------------------------------------------------------------
  // stage 3
  // --------------------------------------------------------------------------------
  // report ecc error

  val s3_valid = RegNext(s2_valid)
  val s3_paddr = RegEnable(s2_paddr, s2_fire)
  val s3_hit = RegEnable(s2_hit, s2_fire)

  val s3_data_error = io.read_error_delayed // banked_data_resp_word.error && !bank_conflict
  val s3_tag_error = RegEnable(s2_tag_error, s2_fire)
  val s3_flag_error = RegEnable(s2_flag_error, s2_fire)
  val s3_error = s3_tag_error || s3_flag_error || s3_data_error

  // error_delayed signal will be used to update uop.exception 1 cycle after load writeback
  resp.bits.error_delayed := s3_error && (s3_hit || s3_tag_error) && s3_valid

  // report tag / data / l2 error (with paddr) to bus error unit
  io.error := 0.U.asTypeOf(new L1CacheErrorInfo())
  io.error.report_to_beu := (s3_tag_error || s3_data_error) && s3_valid
  io.error.paddr := s3_paddr
  io.error.source.tag := s3_tag_error
  io.error.source.data := s3_data_error
  io.error.source.l2 := s3_flag_error
  io.error.opType.load := true.B
  // report tag error / l2 corrupted to CACHE_ERROR csr
  io.error.valid := s3_error && s3_valid

  // update plru, report error in s3

  io.replace_access.valid := RegNext(RegNext(RegNext(io.meta_read.fire()) && s1_valid && !io.lsu.s1_kill) && !s2_nack_no_mshr)
  io.replace_access.bits.set := RegNext(RegNext(get_idx(s1_req.addr)))
  io.replace_access.bits.way := RegNext(RegNext(Mux(s1_tag_match, OHToUInt(s1_tag_match_way), io.replace_way.way)))

  // --------------------------------------------------------------------------------
  // Debug logging functions
  def dump_pipeline_reqs(pipeline_stage_name: String, valid: Bool,
    req: DCacheWordReq ) = {
      when (valid) {
        XSDebug(s"$pipeline_stage_name: ")
        req.dump()
      }
  }

  def dump_pipeline_valids(pipeline_stage_name: String, signal_name: String, valid: Bool) = {
    when (valid) {
      XSDebug(s"$pipeline_stage_name $signal_name\n")
    }
  }

  // performance counters
  XSPerfAccumulate("load_req", io.lsu.req.fire())
  XSPerfAccumulate("load_s1_kill", s1_fire && io.lsu.s1_kill)
  XSPerfAccumulate("load_hit_way", s1_fire && s1_tag_match)
  XSPerfAccumulate("load_replay", io.lsu.resp.fire() && resp.bits.replay)
  XSPerfAccumulate("load_replay_for_data_nack", io.lsu.resp.fire() && resp.bits.replay && s2_nack_data)
  XSPerfAccumulate("load_replay_for_no_mshr", io.lsu.resp.fire() && resp.bits.replay && s2_nack_no_mshr)
  XSPerfAccumulate("load_replay_for_conflict", io.lsu.resp.fire() && resp.bits.replay && io.bank_conflict_slow)
  XSPerfAccumulate("load_hit", io.lsu.resp.fire() && !real_miss)
  XSPerfAccumulate("load_miss", io.lsu.resp.fire() && real_miss)
  XSPerfAccumulate("load_succeed", io.lsu.resp.fire() && !resp.bits.miss && !resp.bits.replay)
  XSPerfAccumulate("load_miss_or_conflict", io.lsu.resp.fire() && resp.bits.miss)
  XSPerfAccumulate("actual_ld_fast_wakeup", s1_fire && s1_tag_match && !io.disable_ld_fast_wakeup)
  XSPerfAccumulate("ideal_ld_fast_wakeup", io.banked_data_read.fire() && s1_tag_match)

  val perfEvents = Seq(
    ("load_req                 ", io.lsu.req.fire()                                               ),
    ("load_replay              ", io.lsu.resp.fire() && resp.bits.replay                          ),
    ("load_replay_for_data_nack", io.lsu.resp.fire() && resp.bits.replay && s2_nack_data          ),
    ("load_replay_for_no_mshr  ", io.lsu.resp.fire() && resp.bits.replay && s2_nack_no_mshr       ),
    ("load_replay_for_conflict ", io.lsu.resp.fire() && resp.bits.replay && io.bank_conflict_slow ),
  )
  generatePerfEvent()
}
