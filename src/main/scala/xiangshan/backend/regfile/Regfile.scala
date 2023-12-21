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

package xiangshan.backend.regfile

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import xiangshan._
import xiangshan.backend.datapath.DataConfig.{DataConfig, FpData, FpRegSrcDataSet, IntData, IntRegSrcDataSet, VecData, VecRegSrcDataSet, VfRegSrcDataSet}
import xiangshan.backend.exu.ExeUnitParams

class RfReadPort(dataWidth: Int, addrWidth: Int) extends Bundle {
  val addr = Input(UInt(addrWidth.W))
  val data = Output(UInt(dataWidth.W))
}

class RfWritePort(dataWidth: Int, addrWidth: Int) extends Bundle {
  val wen = Input(Bool())
  val addr = Input(UInt(addrWidth.W))
  val data = Input(UInt(dataWidth.W))
}

class RfReadPortWithConfig(val rfReadDataCfg: DataConfig, addrWidth: Int) extends Bundle {
  val addr: UInt = Input(UInt(addrWidth.W))
  val srcType: UInt = Input(UInt(3.W))

  def readInt: Boolean = IntRegSrcDataSet.contains(rfReadDataCfg)
  def readFp : Boolean = FpRegSrcDataSet .contains(rfReadDataCfg)
  def readVec: Boolean = VecRegSrcDataSet.contains(rfReadDataCfg)
  def readVf : Boolean = VfRegSrcDataSet .contains(rfReadDataCfg)
}

class RfWritePortWithConfig(val rfWriteDataCfg: DataConfig, addrWidth: Int) extends Bundle {
  val wen = Input(Bool())
  val addr = Input(UInt(addrWidth.W))
  val data = Input(UInt(rfWriteDataCfg.dataWidth.W))
  val intWen = Input(Bool())
  val fpWen = Input(Bool())
  val vecWen = Input(Bool())
  def writeInt: Boolean = rfWriteDataCfg.isInstanceOf[IntData]
  def writeFp : Boolean = rfWriteDataCfg.isInstanceOf[FpData]
  def writeVec: Boolean = rfWriteDataCfg.isInstanceOf[VecData]
}

class Regfile
(
  name: String,
  numPregs: Int,
  numReadPorts: Int,
  numWritePorts: Int,
  hasZero: Boolean,
  len: Int,
  width: Int,
) extends Module {
  val io = IO(new Bundle() {
    val readPorts = Vec(numReadPorts, new RfReadPort(len, width))
    val writePorts = Vec(numWritePorts, new RfWritePort(len, width))
    val debug_rports = Vec(65, new RfReadPort(len, width))
  })

  println(name + ": size:" + numPregs + " read: " + numReadPorts + " write: " + numWritePorts)

  val mem = Reg(Vec(numPregs, UInt(len.W)))
  for (r <- io.readPorts) {
    val rdata = if (hasZero) Mux(r.addr === 0.U, 0.U, mem(r.addr)) else mem(r.addr)
    r.data := RegNext(rdata)
  }
  for (w <- io.writePorts) {
    when(w.wen) {
      mem(w.addr) := w.data
    }
  }

  for (rport <- io.debug_rports) {
    val zero_rdata = Mux(rport.addr === 0.U, 0.U, mem(rport.addr))
    rport.data := (if (hasZero) zero_rdata else mem(rport.addr))
  }
}

object Regfile {
  // non-return version
  def apply(
    name         : String,
    numEntries   : Int,
    raddr        : Seq[UInt],
    rdata        : Vec[UInt],
    wen          : Seq[Bool],
    waddr        : Seq[UInt],
    wdata        : Seq[UInt],
    hasZero      : Boolean,
    withReset    : Boolean,
    debugReadAddr: Option[Seq[UInt]],
    debugReadData: Option[Vec[UInt]]
  )(implicit p: Parameters): Unit = {
    val numReadPorts = raddr.length
    val numWritePorts = wen.length
    require(wen.length == waddr.length, s"wen.length ${wen.length} != waddr.length ${waddr.length}")
    require(wen.length == wdata.length, s"wen.length ${wen.length} != wdata.length ${wdata.length}")
    val dataBits = wdata.map(_.getWidth).min
    require(wdata.map(_.getWidth).min == wdata.map(_.getWidth).max, s"dataBits != $dataBits")
    val addrBits = waddr.map(_.getWidth).min
    require(waddr.map(_.getWidth).min == waddr.map(_.getWidth).max, s"addrBits != $addrBits")

    val regfile = Module(new Regfile(name, numEntries, numReadPorts, numWritePorts, hasZero, dataBits, addrBits))
    rdata := regfile.io.readPorts.zip(raddr).map { case (rport, addr) =>
      rport.addr := addr
      rport.data
    }

    regfile.io.writePorts.zip(wen).zip(waddr).zip(wdata).foreach{ case (((wport, en), addr), data) =>
      wport.wen := en
      wport.addr := addr
      wport.data := data
    }
    if (withReset) {
      val numResetCycles = math.ceil(numEntries / numWritePorts).toInt
      val resetCounter = RegInit(numResetCycles.U)
      val resetWaddr = RegInit(VecInit((0 until numWritePorts).map(_.U(log2Up(numEntries + 1).W))))
      val inReset = resetCounter =/= 0.U
      when (inReset) {
        resetCounter := resetCounter - 1.U
        resetWaddr := VecInit(resetWaddr.map(_ + numWritePorts.U))
      }
      when (!inReset) {
        resetWaddr.map(_ := 0.U)
      }
      for ((wport, i) <- regfile.io.writePorts.zipWithIndex) {
        wport.wen := inReset || wen(i)
        wport.addr := Mux(inReset, resetWaddr(i), waddr(i))
        wport.data := wdata(i)
      }
    }

    require(debugReadAddr.nonEmpty == debugReadData.nonEmpty, "Both debug addr and data bundles should be empty or not")
    regfile.io.debug_rports := DontCare
    if (debugReadAddr.nonEmpty && debugReadData.nonEmpty) {
      debugReadData.get := VecInit(regfile.io.debug_rports.zip(debugReadAddr.get).map { case (rport, addr) =>
        rport.addr := addr
        rport.data
      })
    }
  }

  def apply(
    name         : String,
    numEntries   : Int,
    numBank      : Int,
    raddr        : Seq[Seq[UInt]],
    rdata        : Seq[Vec[UInt]],
    wen          : Seq[Seq[Bool]], // bank, port
    waddr        : Seq[Seq[UInt]], // bank, port
    wdata        : Seq[Seq[UInt]], // bank, port
    hasZero      : Boolean,
    withReset    : Boolean,
    debugReadAddr: Option[Seq[UInt]],
    debugReadData: Option[Vec[UInt]],
  )(implicit p: Parameters): Unit = {
    require(raddr.size == numBank, s"raddr.size ${raddr.size} != numBank $numBank")
    require(wen.size == numBank, s"wen.size ${wen.size} != numBank $numBank")
    require(waddr.size == numBank, s"waddr.size ${waddr.size} != numBank $numBank")
    require(wdata.size == numBank, s"wdata.size ${wdata.size} != numBank $numBank")
    require(wen.head.size == waddr.head.size, s"wen.head.size ${wen.head.size} != waddr.head.size ${waddr.head.size}")
    require(wen.head.size == wdata.head.size, s"wen.head.size ${wen.head.size} != wdata.head.size ${wdata.head.size}")

    val bankBitsWidth = log2Ceil(numBank)
    val bankedDebugRAddr = debugReadAddr.map(_.map(x => (x >> bankBitsWidth.U).asUInt))
    val bankedDebugRData: Seq[Option[Vec[UInt]]] =
      Seq.fill(numBank)(
        if (debugReadData.isDefined) {
          Some(Wire((debugReadData.get.cloneType)))
        } else { None }
      )

    // Each bank has an independent regfile with partial entries
    (0 until numBank).foreach(bankIdx => {
      val bankedNumEntries = numEntries / numBank
      require(raddr(bankIdx).head.getWidth == log2Ceil(bankedNumEntries), s"raddr($bankIdx).head.getWidth ${raddr(bankIdx).head.getWidth} != ${log2Ceil(bankedNumEntries)}")
      require(waddr(bankIdx).head.getWidth == log2Ceil(bankedNumEntries), s"waddr($bankIdx).head.getWidth ${waddr(bankIdx).head.getWidth} != ${log2Ceil(bankedNumEntries)}")
      if (debugReadAddr.isDefined) {
        require(debugReadAddr.get.head.getWidth == log2Ceil(numEntries), s"debugReadAddr($bankIdx).head.getWidth ${debugReadAddr.get.head.getWidth} != log2Ceil($numEntries)")
      }

      apply(
        name = name + s"_bank${bankIdx}",
        numEntries = bankedNumEntries,
        raddr = raddr(bankIdx),
        rdata = rdata(bankIdx),
        wen = wen(bankIdx),
        waddr = waddr(bankIdx),
        wdata = wdata(bankIdx),
        hasZero = hasZero && (bankIdx == 0),
        withReset = withReset,
        bankedDebugRAddr,
        bankedDebugRData(bankIdx)
      )
    })

    // manually connect debug read data
    if (debugReadData.isDefined) {
      val debugData = debugReadData.get
      val debugAddr = debugReadAddr.get
      val bankedDebugRfData = bankedDebugRData.map(_.get)

      debugData.zipWithIndex.foreach {
        case (port, portIdx) =>
          val bankedOH = (0 until numBank).map{ bankIdx =>
            if (numBank == 1) true.B
            else debugAddr(portIdx)(bankBitsWidth-1, 0) === bankIdx.U
            // NOTE: RegNext inside regfile, so RegNext here. Ugly Code
          }
          val bankRfData = bankedDebugRfData.map(_.apply(portIdx))

          assert(PopCount(bankedOH) === 1.U, s"debugReadAddr($portIdx) should be in one bank")
          port := Mux1H(bankedOH, bankRfData)
      }
    }
  }
}

object IntRegFile {
  // non-return version
  def apply(
    name         : String,
    numEntries   : Int,
    numBank      : Int,
    raddr        : Seq[Seq[UInt]],
    rdata        : Seq[Vec[UInt]],
    wen          : Seq[Seq[Bool]],
    waddr        : Seq[Seq[UInt]],
    wdata        : Seq[Seq[UInt]],
    debugReadAddr: Option[Seq[UInt]],
    debugReadData: Option[Vec[UInt]],
    withReset    : Boolean = false,
  )(implicit p: Parameters): Unit = {
    require(raddr.size == numBank)
    Regfile(
      name, numEntries, numBank, raddr, rdata, wen, waddr, wdata,
      hasZero = true, withReset, debugReadAddr, debugReadData)
  }
}

object VfRegFile {
  // non-return version
  def apply(
    name         : String,
    numEntries   : Int,
    splitNum     : Int,
    numBank      : Int,
    raddr        : Seq[Seq[UInt]], // bank, port
    rdata        : Seq[Vec[UInt]], // bank, port
    wen          : Seq[Seq[Seq[Bool]]], // split, bank, port
    waddr        : Seq[Seq[UInt]], // bank, port
    wdata        : Seq[Seq[UInt]], // bank, port
    debugReadAddr: Option[Seq[UInt]],
    debugReadData: Option[Vec[UInt]],
    withReset    : Boolean = false,
  )(implicit p: Parameters): Unit = {
    require(splitNum >= 1, "splitNum should be no less than 1")
    require(splitNum == wen.length, s"splitNum $splitNum should be equal to length of wen vec ${wen.length}")
    if (splitNum == 1) {
      Regfile(
        name, numEntries, numBank, raddr, rdata, wen.head, waddr, wdata,
        hasZero = false, withReset, debugReadAddr, debugReadData)
    } else {
      val dataWidth = 64 // TODO: dataWidth = wdataWidth / splitNum
      val numReadPorts = raddr.length
      require(splitNum > 1 && wdata.head.head.getWidth == dataWidth * splitNum)
      // NOTE: split -> numBank -> portVec. May DIFFFERENT with other place
      val wdataVec = Wire(Vec(splitNum, Vec(numBank, Vec(wdata.head.length, UInt(dataWidth.W)))))
      val rdataVec = Wire(Vec(splitNum, Vec(numBank, Vec(raddr.head.length, UInt(dataWidth.W)))))
      val debugRDataVec: Option[Vec[Vec[UInt]]] = debugReadData.map(x => Wire(Vec(splitNum, Vec(x.length, UInt(dataWidth.W)))))
      for (i <- 0 until splitNum) {
//        wdataVec(i) := wdata.map(_ ((i + 1) * dataWidth - 1, i * dataWidth))
        wdataVec(i).zip(wdata).map{ case (sp, all) =>
          sp.zip(all).map{ case (s, a) =>
            s := a((i + 1) * dataWidth - 1, i * dataWidth) }
        }
        Regfile(
          name + s"Part${i}", numEntries, numBank, raddr, rdataVec(i), wen(i), waddr, wdataVec(i),
          hasZero = false, withReset, debugReadAddr, debugRDataVec.map(_(i))
        )
      }
      rdata.indices.map{ bankIdx =>
        rdata(bankIdx).indices.map { portIdx =>
          rdata(bankIdx)(portIdx) := Cat(rdataVec.map(_ (bankIdx)).map(_ (portIdx)).reverse)
        }
      }

      if (debugReadData.nonEmpty) {
        for (i <- 0 until debugReadData.get.length) {
          debugReadData.get(i) := Cat(debugRDataVec.get.map(_ (i)).reverse)
        }
      }
    }
  }
}