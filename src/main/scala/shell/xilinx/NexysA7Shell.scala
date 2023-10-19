// See LICENSE for license details.
package sifive.fpgashells.shell.xilinx.nexysa7shell

import Chisel._
import chisel3.{Input, Output, RawModule, withClockAndReset}
import chisel3.experimental.{attach, Analog}

import freechips.rocketchip.config._
import freechips.rocketchip.devices.debug._
import freechips.rocketchip.util.{SyncResetSynchronizerShiftReg, ElaborationArtefacts, HeterogeneousBag}

import sifive.blocks.devices.gpio._
import sifive.blocks.devices.spi._
import sifive.blocks.devices.uart._

import sifive.fpgashells.devices.xilinx.digilentnexysa7mig._
import sifive.fpgashells.ip.xilinx._

import sifive.fpgashells.clocks._

//-------------------------------------------------------------------------
// Arty 100T Memory Interface Generator
//-------------------------------------------------------------------------

trait HasDDR2 { this: NexysA7Shell =>
  // import different migs in 1 shell, need to differentiate MemoryXilinxDDRKey
  require(!p.lift(MemoryDigilentDDRKey).isEmpty)
  val ddr = IO(new DigilentNexysA7MIGPads(p(MemoryDigilentDDRKey)))
  
  def connectMIG(dut: HasMemoryDigilentNexysA7MIGModuleImp): Unit = {
    // Clock & Reset
    dut.digilentnexysa7mig.sys_clk_i := clk200.asUInt
    mig_clock                        := dut.digilentnexysa7mig.ui_clk
    mig_sys_reset                    := dut.digilentnexysa7mig.ui_clk_sync_rst
    dut.digilentnexysa7mig.aresetn   := mig_resetn
    dut.digilentnexysa7mig.sys_rst   := sys_reset

    ddr <> dut.digilentnexysa7mig
  }
}

abstract class NexysA7Shell(implicit val p: Parameters) extends RawModule {

  //-----------------------------------------------------------------------
  // Interface
  //-----------------------------------------------------------------------
  
  // 100Mhz sysclk
  val clock_100            = IO(Input(Clock()))

  // active high reset
  val resetn               = IO(Input(Bool()))

  // UART
  val uart_tx              = IO(Output(Bool()))
  val uart_rx              = IO(Input(Bool()))
  val uart_rts             = IO(Output(Bool()))
  val uart_cts             = IO(Input(Bool()))

  // SDIO
  val sdio_poweroff        = IO(Output(Bool()))
  val sdio_clk             = IO(Output(Bool()))
  val sdio_cmd             = IO(Analog(1.W))
  val sdio_dat             = IO(Analog(4.W))

  //-----------------------------------------------------------------------
  // Wire declrations
  //-----------------------------------------------------------------------

  val sys_reset       = Wire(Bool())

  val dut_clock       = Wire(Clock())
  val dut_reset       = Wire(Bool())
  val dut_resetn      = Wire(Bool())

  val dut_ndreset     = Wire(Bool())

  val sd_spi_sck      = Wire(Bool())
  val sd_spi_cs       = Wire(Bool())
  val sd_spi_dq_i     = Wire(Vec(4, Bool()))
  val sd_spi_dq_o     = Wire(Vec(4, Bool()))

  val reset           = Wire(Bool())

  val mig_mmcm_locked = Wire(Bool())
  val mig_sys_reset   = Wire(Bool())

  val mig_clock       = Wire(Clock())
  val mig_reset       = Wire(Bool())
  val mig_resetn      = Wire(Bool())

  //-----------------------------------------------------------------------
  // System reset
  //-----------------------------------------------------------------------

  sys_reset := reset

  //-----------------------------------------------------------------------
  // Clock Generator
  //-----------------------------------------------------------------------

  //65MHz and multiples
  val nexysa7_sys_clock_mmcm = Module(new mmcm())

  nexysa7_sys_clock_mmcm.io.clk_in1 := clock_100
  val clk50   = nexysa7_sys_clock_mmcm.io.clk_out1  // 50 Mhz
  val clk200  = nexysa7_sys_clock_mmcm.io.clk_out2  // 200 Mhz
  val clk100  = nexysa7_sys_clock_mmcm.io.clk_out3  // 100 Mhz
  nexysa7_sys_clock_mmcm.io.resetn  := resetn

  // DUT clock and reset
  reset := !nexysa7_sys_clock_mmcm.io.locked
  dut_clock := clk50
  dut_reset := sys_reset
  mig_reset := sys_reset
  mig_resetn := !mig_reset

  mig_clock            := clk200
  mig_mmcm_locked      := UInt("b1")
 
  //-----------------------------------------------------------------------
  // UART
  //-----------------------------------------------------------------------

  uart_rts  := !false.B

  def connectUART(dut: HasPeripheryUARTModuleImp): Unit = dut.uart.headOption.foreach(connectUART)

  def connectUART(uart: UARTPortIO): Unit = {
    uart.rxd := SyncResetSynchronizerShiftReg(uart_rx, 2, init = Bool(true), name=Some("uart_rxd_sync"))
    uart_tx  := uart.txd
  }

  //-----------------------------------------------------------------------
  // SPI
  //-----------------------------------------------------------------------

  def connectSPI(dut: HasPeripherySPIModuleImp): Unit = dut.spi.headOption.foreach(connectSPI)

  def connectSPI(spi: SPIPortIO): Unit = {
    // SPI
    sd_spi_sck := spi.sck
    sd_spi_cs  := spi.cs(0)

    spi.dq.zipWithIndex.foreach {
      case(pin, idx) =>
        sd_spi_dq_o(idx) := pin.o
        pin.i            := sd_spi_dq_i(idx)
    }

    //-------------------------------------------------------------------
    // SDIO <> SPI Bridge
    //-------------------------------------------------------------------

    val ip_sdio_spi = Module(new sdio_spi_bridge())

    ip_sdio_spi.io.clk   := dut_clock
    ip_sdio_spi.io.reset := dut_reset

    // SDIO
    attach(sdio_dat, ip_sdio_spi.io.sd_dat)
    attach(sdio_cmd, ip_sdio_spi.io.sd_cmd)
    sdio_clk := ip_sdio_spi.io.spi_sck
    sdio_poweroff := sys_reset

    // SPI
    ip_sdio_spi.io.spi_sck  := sd_spi_sck
    ip_sdio_spi.io.spi_cs   := sd_spi_cs
    sd_spi_dq_i             := ip_sdio_spi.io.spi_dq_i.asBools
    ip_sdio_spi.io.spi_dq_o := sd_spi_dq_o.asUInt
  }

  ElaborationArtefacts.add("old-shell.vivado.tcl",
    """set obj [current_fileset -constrset]
      |add_files -quiet -norecurse -fileset $obj [file join $boarddir tcl ios.tcl]
      |add_files -quiet -norecurse -fileset $obj [file join $boarddir tcl clocks.tcl]
      |""".stripMargin)
}
