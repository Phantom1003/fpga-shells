# See LICENSE for license details.

create_ip -vendor xilinx.com -library ip -name clk_wiz -module_name mmcm -dir $ipdir -force
set_property -dict [list \
        CONFIG.PRIMITIVE {PLL} \
        CONFIG.RESET_TYPE {ACTIVE_LOW} \
        CONFIG.CLKOUT1_USED {true} \
        CONFIG.CLKOUT2_USED {true} \
        CONFIG.CLKOUT3_USED {true} \
        CONFIG.CLKOUT1_REQUESTED_OUT_FREQ {50.000} \
        CONFIG.CLKOUT2_REQUESTED_OUT_FREQ {200.000} \
        CONFIG.CLKOUT3_REQUESTED_OUT_FREQ {50.000} \
        ] [get_ips mmcm]
