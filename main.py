import machine
import utime

import ips2550

IPS_ADDR = 24
I2C_FREQ = 100_000

def print_current_config(ips: ips2550.IPS):
    print(f"Supply Voltage            {ips.get_vdd()} V")
    print(f"Output Mode               {"Differential" if ips.get_output_mode() == ips2550.OM_DIFFERENTIAL else "Single Ended"}")
    print(f"Automatic Gain Control    {ips.get_automatic_gain_control()}")
    print(
        f"Master Gain Code          {ips.get_master_gain_code()} ({ips.get_master_gain()}x)"
    )
    print(f"Master Gain Boost (2x)    {ips.get_master_gain_boost()}")
    print(
        f"Fine Gain 1               {ips.get_fine_gain_1_code()} ({ips.get_fine_gain_1()}x)"
    )
    print(
        f"Fine Gain 2               {ips.get_fine_gain_2_code()} ({ips.get_fine_gain_2()}x)"
    )
    print(
        f"Offset 1                  {ips.get_offset_sign_1() * ips.get_offset_code_1()} ({ips.get_offset_1_perc()}*Vtx)"
    )
    print(
        f"Offset 2                  {ips.get_offset_sign_2() * ips.get_offset_code_2()} ({ips.get_offset_2_perc()}*Vtx)"
    )
    print(f"TX Current Bias           {ips.get_tx_current_bias_uA()} uA")
    print(f"TX Frequency              {ips.get_tx_frequency()/1e6:0.2f} MHz")

ips = ips2550.IPS(
    0,
    sda=machine.Pin(16),
    scl=machine.Pin(17),
    i2c_addr=IPS_ADDR,
    freq=I2C_FREQ,
    rx1=machine.Pin(26),
    rx2=machine.Pin(27),
    ref=machine.Pin(28),
)


ips.set_output_mode(ips2550.OM_DIFFERENTIAL)
ips.set_voltage(ips2550.VDD_3V3)
ips.set_current_bias(0xFF)
ips.set_automatic_gain_control(False)
ips.set_master_gain_boost(True)
ips.set_master_gain_code(50)


print_current_config(ips)


# while(True):
#     freq = ips.get_tx_frequency()
#     rx1 = ips.get_rx1()
#     rx2 = ips.get_rx2()

#     print(f"{freq/1e6:0.2f}\t{rx1:0.3f}\t{rx2:0.3f}")
#     utime.sleep_ms(100)
