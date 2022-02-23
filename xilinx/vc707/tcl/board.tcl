# See LICENSE for license details.
set name {vc707}
set part_fpga {xc7vx485tffg1761-2}
if {[regexp -all 2021 [version -short]]} {
    set part_board {xilinx.com:vc707:part0:1.4} 
} else {
    set part_board {xilinx.com:vc707:part0:1.3}
}
