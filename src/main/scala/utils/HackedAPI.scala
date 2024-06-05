package utils

import scala.reflect.macros.blackbox.Context
import scala.language.experimental.macros

import chisel3._
import chisel3.util._

object HackedAPI {
  object HackedRegEnable {
    def apply[T <: Data](next: T, enable: Bool): T = {
      checkAndPrint(next, enable)

      chisel3.util.RegEnable(next, enable)
    }

    def apply[T <: Data](next: T, init: T, enable: Bool): T = {
      checkAndPrint(next, enable)

      chisel3.util.RegEnable(next, init, enable)
    }

    def getLocation: String = {
      val stackTrace = Thread.currentThread().getStackTrace

      s"${stackTrace.drop(4).take(10).mkString("[HackedRegEnable]   ", "\n[HackedRegEnable]   ", "\n")}" ++
      "[HackedRegEnable]   ...\n" ++
      s"${stackTrace.takeRight(10).mkString("[HackedRegEnable]   ", "\n[HackedRegEnable]   ", "\n")}"
    }

    def checkAndPrint[T <: Data](next: T, enable: Bool): Unit = {
      next match {
        case bool: Bool if (bool == enable) =>
          println(s"[HackedRegEnable] arg `next' is the same as arg `enable'. HackedRegEnable call stack is as follows:\n$getLocation")
        case validio: Valid[_] =>
          println(s"[HackedRegEnable] arg `next' is Valid[Data].")
          if (validio.valid == enable) {
            println(s"[HackedRegEnable] valid in arg `next' is the same as arg `enable'.")
          }
          println(s"[HackedRegEnable] call stack is as follows:\n$getLocation")
        case decoupledio: DecoupledIO[_] =>
          println(s"[HackedRegEnable] arg `next' is DecoupledIO[Data]")
          if (decoupledio.valid == enable) {
            println(s"[HackedRegEnable] valid in arg `next' is the same as arg `enable'.")
          }
          println(s"[HackedRegEnable] call stack is as follows:\n$getLocation")
        case record: Record =>
          for (data <- record.getElements) {
            if (data == enable) {
              println(s"[HackedRegEnable] arg `next' is Record and arg `enable' appears in arg `next', HackedRegEnable call stack is as follows:\n$getLocation")
            }
          }
        case _ =>
      }
    }
  }
}
