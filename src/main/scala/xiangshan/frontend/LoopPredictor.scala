package xiangshan.frontend

import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chisel3.util._
import utils._
import xiangshan._
import chisel3.experimental.chiselName

trait LoopPredictorParams extends HasXSParameter with HasBPUParameter {
  val nRows = 32
  val cntBits = 32
  val confBits = 2
  val predParallel = 2 // now only double

  val idxLen = log2Up(nRows)
  val tagLen = VAddrBits - (idxLen + instOffsetBits)

  def maxConf = ((1 << confBits) - 1).U(confBits.W)
  def minConf = 0.U

  def dupForLp = dupForIttage

  def getLtbTag(pc: UInt) = pc >> (idxLen + instOffsetBits)
  def getLtbIdx(pc: UInt) = pc(instOffsetBits+idxLen-1,instOffsetBits)
  def initLTBentry(newLTBtag: UInt/*, target: UInt*/) : LoopEntry = {
    val newLTBentry = WireDefault( 0.U.asTypeOf(new LoopEntry) )
    newLTBentry.tag        := newLTBtag(tagLen-1, 0)
    newLTBentry.specCnt    := 1.U
    newLTBentry.totalSpecCnt := 1.U
    newLTBentry
  }
  def increConf(conf: UInt) : UInt = {
    val newConf = Mux( (conf === maxConf), maxConf, (conf + 1.U) )
    newConf
  }
  def decreConf(conf: UInt) : UInt = {
    val newConf = Mux( (conf === minConf), minConf, (conf - 1.U) )
    newConf
  }
  def reviseSpecCnt(rawTotalSpecCnt: UInt, tripCnt: UInt) = 
    Mux(rawTotalSpecCnt % tripCnt === 0.U, tripCnt, (rawTotalSpecCnt % tripCnt))

  def doPred(oldTotalSpecCnt: UInt, tripCnt: UInt, conf: UInt, isDouble: Bool): UInt = {
    val newSpecCnt = WireDefault(oldTotalSpecCnt)
    when(conf =/= minConf) {
      // newSpecCnt := (oldSpecCnt % tripCnt) + 1.U
      newSpecCnt := Mux(isDouble, reviseSpecCnt(oldTotalSpecCnt + 2.U, tripCnt), 
                                  reviseSpecCnt(oldTotalSpecCnt + 1.U, tripCnt))
    }.otherwise {
      newSpecCnt := Mux(isDouble, (oldTotalSpecCnt + 2.U), (oldTotalSpecCnt + 1.U))
    }
    newSpecCnt
  }
  def doRecover(predSpecCnt: Vec[UInt], predExitLoop: Bool, conf: UInt, 
                isDouble: Bool, doublePartIdx: UInt): UInt = {
    val newSpecCnt = WireDefault(0.U)
    when(conf =/= minConf && !predExitLoop) {
      newSpecCnt := 0.U
    }.otherwise {
      when(isDouble && doublePartIdx === 0.U) {
        newSpecCnt := predSpecCnt(0)
      }.otherwise {
        newSpecCnt := predSpecCnt(1)
      }
    }
    newSpecCnt
  }

}

class LoopEntry(implicit p: Parameters) extends XSBundle with LoopPredictorParams {
  val tag        = UInt(tagLen.W)
  val specCnt    = UInt(cntBits.W)
  val tripCnt    = UInt(cntBits.W)
  val conf       = UInt(confBits.W)  
  val totalSpecCnt = UInt(cntBits.W)
  // val debugCnt = UInt(cntBits.W)


  def isConf    = (conf === maxConf)
  def isNotConf = (conf < maxConf) 
}

class LTBrwIO(implicit p: Parameters) extends XSBundle with LoopPredictorParams {
  val readEna    = Input(Bool())
  val readIdx    = Input(UInt(idxLen.W))
  val readEntry  = Output(new LoopEntry)

  val writeEna   = Input(Bool())
  val writeIdx   = Input(UInt(idxLen.W))
  val writeEntry = Input(new LoopEntry)
}

class LTBio(implicit p: Parameters) extends XSBundle with LoopPredictorParams {
  val pred    = new LTBrwIO
  val update  = new LTBrwIO
  val recover = new LTBrwIO
  val predIsDouble = Input(Bool())
}

class LTB(implicit p: Parameters) extends XSModule with LoopPredictorParams {
  val io = IO(new LTBio)

  val ltb = RegInit(VecInit(Seq.fill(nRows)(0.U.asTypeOf(new LoopEntry))))
  
  val predReadUpdateWrite    = io.pred.readEna && io.update.writeEna && 
                               (io.pred.readIdx === io.update.writeIdx)
  val predReadRecoverWrite   = io.pred.readEna && io.recover.writeEna && 
                               (io.pred.readIdx === io.recover.writeIdx)
  val recoverReadUpdateWrite = io.recover.readEna && io.update.writeEna && 
                               (io.recover.readIdx === io.update.writeIdx)

  // pred read
  when(io.pred.readEna) {
    when(predReadUpdateWrite) {
      io.pred.readEntry := io.update.writeEntry
    }.elsewhen(predReadRecoverWrite) {
      io.pred.readEntry := io.recover.writeEntry
    }.otherwise {
      io.pred.readEntry := ltb(io.pred.readIdx)
    }
  }.otherwise {
    io.pred.readEntry := 0.U.asTypeOf(new LoopEntry)
  }

  when(io.recover.readEna) {
    when(recoverReadUpdateWrite) {
      io.recover.readEntry := io.update.writeEntry
    }.otherwise {
      io.recover.readEntry := ltb(io.recover.readIdx)
    }
  }.otherwise {
    io.recover.readEntry := 0.U.asTypeOf(new LoopEntry)
  }

  // update read
  when(io.update.readEna) {
    io.update.readEntry := ltb(io.update.readIdx)
  }.otherwise {
    io.update.readEntry := 0.U.asTypeOf(new LoopEntry)
  }


  val predUpdateWrite    = io.pred.writeEna && io.update.writeEna &&
                           (io.pred.writeIdx === io.update.writeIdx)
  val predRecoverWrite   = io.pred.writeEna && io.recover.writeEna &&
                           (io.pred.writeIdx === io.recover.writeIdx)
  val recoverUpdateWrite = io.recover.writeEna && io.update.writeEna &&
                           (io.recover.writeIdx === io.update.writeIdx)
  // pred write 
  val predWriteEntry = WireDefault(io.pred.writeEntry)
  when(predUpdateWrite) {
    when(io.pred.writeEntry.tag === io.update.writeEntry.tag) {
      predWriteEntry.specCnt := doPred(io.update.writeEntry.specCnt, 
                                       io.update.writeEntry.tripCnt, 
                                       io.update.writeEntry.conf,
                                       io.predIsDouble)
    }.otherwise {
      predWriteEntry.specCnt := io.update.writeEntry.specCnt
    }
    predWriteEntry.tag     := io.update.writeEntry.tag
    predWriteEntry.tripCnt := io.update.writeEntry.tripCnt
    predWriteEntry.conf    := io.update.writeEntry.conf
  }.elsewhen(predRecoverWrite) {
    when(io.pred.writeEntry.tag === io.recover.writeEntry.tag) {
      predWriteEntry.specCnt := doPred(io.recover.writeEntry.specCnt, 
                                       io.recover.writeEntry.tripCnt, 
                                       io.recover.writeEntry.conf, 
                                       io.predIsDouble)
    }.otherwise {
      predWriteEntry.specCnt := io.recover.writeEntry.specCnt
    }
    predWriteEntry.tag     := io.recover.writeEntry.tag
    predWriteEntry.tripCnt := io.recover.writeEntry.tripCnt
    predWriteEntry.conf    := io.recover.writeEntry.conf
  }
  when(io.pred.writeEna) {
    ltb(io.pred.writeIdx) := predWriteEntry
  }

  // recover write
  when(io.recover.writeEna) {
    when(recoverUpdateWrite) {
      ltb(io.recover.writeIdx) := io.update.writeEntry
    }.otherwise {
      ltb(io.recover.writeIdx) := io.recover.writeEntry
    }
  }

  // update write
  when(io.update.writeEna) {
    ltb(io.update.writeIdx) := io.update.writeEntry
  }

}

class LPpredInfo (implicit p: Parameters) extends XSBundle with LoopPredictorParams {
  val lpPred       = Bool()
  val predExitLoop = Bool()
  // val predExitLoop = Vec(predParallel, Bool())
  val predConf     = Bool()
  // val specCnt      = UInt(cntBits.W)
  // val totalSpecCnt = UInt(cntBits.W)
  val specCnt = Vec(predParallel, UInt(cntBits.W))
  val totalSpecCnt = Vec(predParallel, UInt(cntBits.W))
  val isDouble     = Bool()
  // val debugCnt = UInt(cntBits.W)
}

class LPpredIO(implicit p: Parameters) extends XSBundle with LoopPredictorParams {
  val valid      = Input(Bool())
  val pc         = Input(UInt(VAddrBits.W))
  val isDouble   = Input(Bool())
  val lpPredInfo = Output(new LPpredInfo)
  val tripCnt    = Output(UInt(cntBits.W))
}

class LPredirectIO(implicit p: Parameters) extends XSBundle with LoopPredictorParams {
  val valid  = Input(Bool())  
  val pc     = Input(UInt(VAddrBits.W))
  val doublePartIdx = Input(UInt(1.W))
  val lpPredInfo = Input(new LPpredInfo)
}

class LPupdateIO(implicit p: Parameters) extends XSBundle with LoopPredictorParams {
  val valid        = Input(Bool())
  val pc           = Input(UInt(VAddrBits.W))
  val isLoopBranch = Input(Bool())
  val updateTaken  = Input(Vec(predParallel, Bool()))
  val lpPredInfo   = Input(new LPpredInfo)
}

class LoopPredictor(implicit p: Parameters) extends XSModule with LoopPredictorParams {
  val io = IO(new Bundle{
    val pred     = new LPpredIO
    val redirect = new LPredirectIO
    val update   = new LPupdateIO
  })

  val ltb = Module(new LTB)

  // pred
  val predLTBidx = getLtbIdx(io.pred.pc)
  ltb.io.pred.readEna := io.pred.valid
  ltb.io.pred.readIdx := predLTBidx
  val predLTBreadEntry = ltb.io.pred.readEntry

  val predTag = getLtbTag(io.pred.pc)
  val predTagMatch = (predTag === predLTBreadEntry.tag)
  val predLTBwriteEntry = WireDefault(predLTBreadEntry) 
  val predLTBwena = io.pred.valid && predTagMatch
  when(predLTBwena) {
    predLTBwriteEntry.specCnt := doPred(predLTBreadEntry.totalSpecCnt, predLTBreadEntry.tripCnt, 
                                        predLTBreadEntry.conf, io.pred.isDouble)
    predLTBwriteEntry.totalSpecCnt := doPred(predLTBreadEntry.totalSpecCnt, predLTBreadEntry.tripCnt, 
                                             minConf, io.pred.isDouble)
    // predLTBwriteEntry.totalSpecCnt := predLTBreadEntry.totalSpecCnt + 1.U
    // predLTBwriteEntry.debugCnt := predLTBreadEntry.debugCnt + 1.U

    printf("pred  pc: %x; specCnt: %d; exitLoop: %d; tripCnt: %d; conf: %d;" +
      " totalSpecCnt: %d; isDouble: %d; \n", 
    io.pred.pc, predLTBwriteEntry.specCnt, io.pred.lpPredInfo.predExitLoop, 
    predLTBreadEntry.tripCnt, predLTBreadEntry.conf, 
    predLTBwriteEntry.totalSpecCnt, io.pred.isDouble)
  }
 
  ltb.io.pred.writeEna   := predLTBwena
  ltb.io.pred.writeIdx   := predLTBidx
  ltb.io.pred.writeEntry := predLTBwriteEntry  
  ltb.io.predIsDouble    := io.pred.isDouble

  val predCntEq  = (predLTBwriteEntry.specCnt === predLTBwriteEntry.tripCnt)
  // val predCntNeq = (predLTBwriteEntry.specCnt =/= predLTBwriteEntry.tripCnt)
  io.pred.lpPredInfo.lpPred := io.pred.valid
  io.pred.lpPredInfo.predExitLoop := (predCntEq && predTagMatch)
  io.pred.lpPredInfo.predConf     := predLTBreadEntry.isConf
  io.pred.lpPredInfo.specCnt(0)      := doPred(predLTBreadEntry.totalSpecCnt, predLTBreadEntry.tripCnt, 
                                        predLTBreadEntry.conf, false.B)
  io.pred.lpPredInfo.totalSpecCnt(0) := doPred(predLTBreadEntry.totalSpecCnt, predLTBreadEntry.tripCnt, 
                                               minConf, false.B)
  io.pred.lpPredInfo.specCnt(1)      := predLTBwriteEntry.specCnt
  io.pred.lpPredInfo.totalSpecCnt(1) := predLTBwriteEntry.totalSpecCnt
  io.pred.lpPredInfo.isDouble     := io.pred.isDouble
  io.pred.tripCnt := predLTBreadEntry.tripCnt

  // redirect
  val redirectValid = io.redirect.valid && io.redirect.lpPredInfo.lpPred
  val redirectPC = io.redirect.pc
  val redirectLTBidx = getLtbIdx(redirectPC)
  ltb.io.recover.readEna := redirectValid
  ltb.io.recover.readIdx := redirectLTBidx
  val redirectLTBreadEntry = ltb.io.recover.readEntry

  val redirectTag = getLtbTag(redirectPC)
  val redirectTagMatch = (redirectTag === redirectLTBreadEntry.tag)
  val redirectLTBwriteEntry = WireDefault(redirectLTBreadEntry)
  val redirectIsWriteLTB = redirectValid && redirectTagMatch
  when(redirectIsWriteLTB) {
    redirectLTBwriteEntry.specCnt := doRecover(io.redirect.lpPredInfo.specCnt, 
                                               io.redirect.lpPredInfo.predExitLoop, 
                                               redirectLTBreadEntry.conf,
                                               io.redirect.lpPredInfo.isDouble,
                                               io.redirect.doublePartIdx)
    redirectLTBwriteEntry.totalSpecCnt := doRecover(io.redirect.lpPredInfo.totalSpecCnt, 
                                                    false.B, 
                                                    false.B,
                                                    io.redirect.lpPredInfo.isDouble,
                                                    io.redirect.doublePartIdx)

    printf("recover-specCnt  pc: %x; new-spcCnt: %d; new-totalSpecCnt: %d; " +
      "pred-specCnt0: %d; pred-totalSpecCnt0: %d; " +
      "pred-specCnt1: %d; pred-totalSpecCnt1: %d; " +
      "crt-totalSpecCnt: %d; doublePartIdx: %d; isdouble: %d\n", 
    redirectPC, redirectLTBwriteEntry.specCnt, redirectLTBwriteEntry.totalSpecCnt, 
    io.redirect.lpPredInfo.specCnt(0), io.redirect.lpPredInfo.totalSpecCnt(0), 
    io.redirect.lpPredInfo.specCnt(1), io.redirect.lpPredInfo.totalSpecCnt(1), 
    redirectLTBreadEntry.totalSpecCnt, io.redirect.doublePartIdx, 
    io.redirect.lpPredInfo.isDouble)
  }
  ltb.io.recover.writeEna   := redirectIsWriteLTB
  ltb.io.recover.writeIdx   := redirectLTBidx
  ltb.io.recover.writeEntry := redirectLTBwriteEntry


  // update
  val totalSpecCntArray = RegInit(VecInit(Seq.fill(nRows)(0.U(cntBits.W))))

  val updateValid = io.update.valid && io.update.lpPredInfo.lpPred
  val updateTaken0 = io.update.updateTaken(0)
  val updateTaken1 = io.update.updateTaken(1)
  val updateIsDouble = io.update.lpPredInfo.isDouble
  val updateTaken = Mux(updateIsDouble, (updateTaken0 && updateTaken1), updateTaken0)
  val updateNotTaken = Mux(updateIsDouble, (!updateTaken0 || !updateTaken1), !updateTaken0)
  val updateExitLoopDPIdx = Mux((updateIsDouble && !updateTaken1), 1.U, 0.U)
  
  val updateLTBidx = getLtbIdx(io.update.pc)
  ltb.io.update.readEna := updateValid
  ltb.io.update.readIdx := updateLTBidx
  val updateLTBreadEntry = ltb.io.update.readEntry
  // val updtCSCAreadEntry = commitSpecCntArray(updtLTBidx)
  val updateTag = getLtbTag(io.update.pc)
  val updateTagMatch = (updateTag === updateLTBreadEntry.tag)
  val updateLTBwriteEntry = WireDefault(updateLTBreadEntry)

  val updateCnt = (updateValid && updateTagMatch && updateNotTaken)
  val updateIsAllocEntry = (updateValid && io.update.isLoopBranch && !updateTagMatch
                            && updateLTBreadEntry.isNotConf && updateTaken)
  val updateAllocEntry = initLTBentry(updateTag)
  val updateLTBwena = (updateIsAllocEntry || updateCnt)
  val updatePredTotalSpecCnt = WireDefault(0.U(cntBits.W))
  val newTripCnt = WireDefault(0.U(cntBits.W))

  when(updateCnt) {
    when(updateIsDouble) {
      updatePredTotalSpecCnt := io.update.lpPredInfo.totalSpecCnt(updateExitLoopDPIdx)
    }.otherwise {
      updatePredTotalSpecCnt := io.update.lpPredInfo.totalSpecCnt(0)
    }
    totalSpecCntArray(updateLTBidx) := updatePredTotalSpecCnt

    newTripCnt := updatePredTotalSpecCnt - totalSpecCntArray(updateLTBidx)
    updateLTBwriteEntry.tripCnt := newTripCnt
    when(newTripCnt === updateLTBreadEntry.tripCnt) {
      updateLTBwriteEntry.conf := increConf(updateLTBreadEntry.conf)
    }.elsewhen(updateLTBreadEntry.conf =/= maxConf) {
      updateLTBwriteEntry.conf := decreConf(updateLTBreadEntry.conf)
    }

    printf("update  pc: %x; new-tripCnt: %d; new-conf: %d; old-tripCnt: %d; " +
           "pred-totalSpecCnt: %d; old-totalSpecCnt: %d; isDouble: %d " +
           "updateExitLoopDPIdx: %d\n",
           io.update.pc, newTripCnt, updateLTBwriteEntry.conf, 
           updateLTBreadEntry.tripCnt, updatePredTotalSpecCnt, 
           totalSpecCntArray(updateLTBidx), updateIsDouble, 
           updateExitLoopDPIdx)
  }
  when(updateIsAllocEntry) {
    totalSpecCntArray(updateLTBidx) := 0.U
  }

  ltb.io.update.writeEna := updateLTBwena
  ltb.io.update.writeIdx := updateLTBidx
  ltb.io.update.writeEntry := Mux(updateIsAllocEntry, updateAllocEntry, updateLTBwriteEntry)

  when(io.update.valid && updateNotTaken) {
    printf("xs-updt pc: %x; new-tripCnt: %d; new-conf: %d; old-tripCnt: %d; " +
           "pred-totalSpecCnt: %d; old-totalSpecCnt: %d; " +
           "lpPred: %d; updateTagMatch: %d\n",
           io.update.pc, newTripCnt, updateLTBwriteEntry.conf, 
           updateLTBreadEntry.tripCnt, updatePredTotalSpecCnt, 
           totalSpecCntArray(updateLTBidx), 
           io.update.lpPredInfo.lpPred, updateTagMatch)
  }
 
  when(updateIsAllocEntry) {
    printf("allocate pc: %x; idx: %d\n", io.update.pc, updateLTBidx)
  }

}

class LPmeta(implicit p: Parameters) extends XSBundle with LoopPredictorParams {
  val pc = UInt(VAddrBits.W)
  val offsetInPredStream = UInt(log2Ceil(PredictWidth).W)
  val isBypass = Bool()
  val lpPredInfo = new LPpredInfo
}

class XSlpPredIO(implicit p: Parameters) extends XSBundle with LoopPredictorParams {
  val valid          = Input(Bool())
  val pc             = Input(UInt(VAddrBits.W))
  val offsetInPredStream = Input(UInt(log2Ceil(PredictWidth).W))
  val isBypass       = Input(Bool())
  val isDouble       = Input(Bool())
  val meta           = Output(new LPmeta)
  // the signals of xsLPpredInfo
  val isConf         = Output(Bool())
  val remainIterNum  = Output(UInt(cntBits.W))
  // no use
  val isConfExitLoop = Output(Bool())
  val target         = Output(UInt(VAddrBits.W))
  val isInterNumGT2  = Output(Bool())
}

class XSlpUpdateIO(implicit p: Parameters) extends XSBundle with LoopPredictorParams {
  val valid        = Input(Bool())
  val pc           = Input(UInt(VAddrBits.W))
  val isLoopBranch = Input(Bool())
  val updateTaken  = Input(Vec(predParallel, Bool()))
  val meta         = Input(new LPmeta)
}

class XSlpRedirectIO(implicit p: Parameters) extends XSBundle with LoopPredictorParams {
  val valid = Input(Bool())
  val pc    = Input(UInt(VAddrBits.W))
  val doublePartIdx = Input(UInt(1.W))
  val meta  = Input(new LPmeta)
}

class XSLoopPredictor(implicit p: Parameters) extends XSModule with LoopPredictorParams {
  val io = IO(new Bundle{
    val lpEna         = Input(Bool())
    val pred          = new XSlpPredIO
    val update        = new XSlpUpdateIO
    val redirect      = new XSlpRedirectIO
  })

  val lp = Module(new LoopPredictor)

  val lpPredValid = (io.lpEna && io.pred.valid)
  lp.io.pred.valid := lpPredValid
  lp.io.pred.pc    := io.pred.pc
  lp.io.pred.isDouble := io.pred.isDouble

  val lpMeta = WireDefault(0.U.asTypeOf(new LPmeta))
  lpMeta.pc := io.pred.pc
  lpMeta.offsetInPredStream := io.pred.offsetInPredStream
  lpMeta.isBypass := io.pred.isBypass
  lpMeta.lpPredInfo := lp.io.pred.lpPredInfo

  val isConf = lp.io.pred.lpPredInfo.predConf
  val theTripCnt = lp.io.pred.tripCnt
  val theSpecCnt = Mux(io.pred.isDouble, lp.io.pred.lpPredInfo.specCnt(1), 
                                         lp.io.pred.lpPredInfo.specCnt(0) )
  io.pred.meta := lpMeta
  io.pred.isConf := isConf
  io.pred.remainIterNum := Mux(isConf, (theTripCnt - theSpecCnt), 0.U)

  io.pred.isConfExitLoop := false.B
  io.pred.target         := 0.U
  io.pred.isInterNumGT2 := true.B

  when(lpPredValid) {
    printf("xs-pred  pc: %x; conf: %d; remainIterNum: %d; theTripCnt: %d; " +
           "theSpecCnt: %d; isDouble: %d; totalSpecCnt: %d\n", 
           io.pred.pc, io.pred.isConf, io.pred.remainIterNum, theTripCnt, 
           theSpecCnt, io.pred.isDouble, io.pred.meta.lpPredInfo.totalSpecCnt(1))
  }

  lp.io.redirect.valid := io.redirect.valid
  // lp.io.redirect.pc := io.redirect.meta.pc
  lp.io.redirect.pc := io.redirect.pc
  lp.io.redirect.doublePartIdx := io.redirect.doublePartIdx
  lp.io.redirect.lpPredInfo := io.redirect.meta.lpPredInfo
  
  lp.io.update.valid          := (io.lpEna && io.update.valid)
  lp.io.update.pc             := io.update.pc
  lp.io.update.isLoopBranch   := io.update.isLoopBranch
  lp.io.update.updateTaken  := io.update.updateTaken
  lp.io.update.lpPredInfo := io.update.meta.lpPredInfo
}