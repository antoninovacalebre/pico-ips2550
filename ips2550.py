import machine
import utime
import math

CONFIG_WAIT_MS = 10


def read_adc_voltage(adc: machine.ADC, vdd: float = 3.3):
    return adc.read_u16() / 65536 * vdd


def most_significant_one(n: int) -> int:
    i = 1
    while (n >> i) != 0:
        i += 1
    return i


def least_significant_one(n: int) -> int:
    if n == 0:
        return 0
    i = 0
    while ((n >> i) & 0b1) != 1:
        i += 1
    return i


def crc(word: int, polynomial: int, filler: int = 0) -> int:
    g = most_significant_one(polynomial)
    n = g - 1
    word = (word << n) | filler
    while (word >> n) != 0:
        first_one = most_significant_one(word)
        xor_mask = ~(0xFF << g) << (first_one - g)
        xor_val = polynomial << (first_one - g)
        word = ((word & xor_mask) ^ xor_val) | (word & ~xor_mask)

    return word & ~(0xFF << g)


VDD_3V3 = 0
VDD_5V0 = 1

OM_DIFFERENTIAL = 0
OM_SINGLE_ENDED = 1

# fmt: off
GAIN_FACTORS = [2.0, 2.1, 2.18, 2.29, 2.38, 2.5, 2.59, 2.72, 2.83, 2.97,
                3.09, 3.24, 3.36, 3.53, 3.67, 3.85, 4.0, 4.2, 4.36, 4.58,
                4.76, 4.99, 5.19, 5.45, 5.66, 5.94, 6.17, 6.48, 6.73, 7.06,
                7.34, 7.7, 8.0, 8.4, 8.72, 9.16, 9.51, 9.99, 10.38, 10.89,
                11.31, 11.88, 12.34, 12.96, 13.46, 14.13, 14.67, 15.41, 16.0, 16.8,
                17.45, 18.32, 19.02, 19.98, 20.75, 21.79, 22.62, 23.76, 24.68, 25.91,
                26.91, 28.26, 29.34, 30.81, 32.0, 33.6, 34.9, 36.64, 38.05, 39.95,
                41.5, 43.58, 45.25, 47.51, 49.36, 51.83, 53.82, 56.52, 58.69, 61.62,
                64.0, 67.2, 69.79, 73.28, 76.1, 79.9, 83.01, 87.16, 90.5, 95.02,
                98.72, 103.66, 107.65, 113.03, 117.38, 123.24]
# fmt: on


class IPS:
    _i2c_addr: int
    _i2c: machine.I2C
    _rx1: machine.ADC
    _rx2: machine.ADC
    _ref: machine.ADC

    def __init__(
        self,
        i2c_id: int,
        sda: machine.Pin,
        scl: machine.Pin,
        i2c_addr: int,
        freq: float,
        rx1: machine.Pin,
        rx2: machine.Pin,
        ref: machine.Pin,
    ):
        self._i2c = machine.I2C(i2c_id, scl=scl, sda=sda, freq=freq)
        self._rx1 = machine.ADC(rx1)
        self._rx2 = machine.ADC(rx2)
        self._ref = machine.ADC(ref)
        self._i2c_addr = i2c_addr

    def read_register(self, reg_addr):
        data = self._i2c.readfrom_mem(self._i2c_addr, reg_addr, 2)
        data = (data[0] << 8) | data[1]
        crc_bits = data & 0b111
        reg = data >> 5

        message = ((reg << 5) & 0xFF00) | (reg & 0b111)
        if crc(message, 0b1011, crc_bits) != 0:
            raise AssertionError("CRC check failed on read")

        return reg

    def read_register_masked(self, reg_addr, mask):
        reg = self.read_register(reg_addr)
        return (reg & mask) >> least_significant_one(mask)

    def write_register(self, reg_addr, value):
        crc_in = (
            ((reg_addr & 0x007F) << 17) | ((value & 0x07F8) << 5) | (value & 0x0007)
        )
        crc_bits = crc(crc_in, 0b1011)
        message = (value << 5) | 0x18 | crc_bits
        self._i2c.writeto_mem(self._i2c_addr, reg_addr, message.to_bytes(2, "big"))

    def write_register_masked(self, reg_addr, value, mask):
        reg = self.read_register(reg_addr)
        updated_reg = (reg & ~mask) | (value & mask)
        self.write_register(reg_addr, updated_reg)

    def set_sub_addr(self, msn):
        self.write_register_masked(0x40, msn << 4, 0x00F0)
        self.write_register_masked(0x00, msn << 4, 0x00F0)
        utime.sleep_ms(CONFIG_WAIT_MS)
        print("The value will change at next power-up.")

    def set_voltage(self, vdd: int):
        if vdd != VDD_3V3 and vdd != VDD_5V0:
            raise RuntimeError
        self.write_register_masked(0x41, vdd, 0b1)
        self.write_register_masked(0x01, vdd, 0b1)
        utime.sleep_ms(CONFIG_WAIT_MS)

    def set_automatic_gain_control(self, enabled: bool):
        val = 0 if enabled else 1
        self.write_register_masked(0x40, val << 9, 0b1 << 9)
        self.write_register_masked(0x00, val << 9, 0b1 << 9)
        utime.sleep_ms(CONFIG_WAIT_MS)

    def set_master_gain_boost(self, enabled: bool):
        val = 1 if enabled else 0
        self.write_register_masked(0x42, val << 7, 0b1 << 7)
        self.write_register_masked(0x02, val << 7, 0b1 << 7)
        utime.sleep_ms(CONFIG_WAIT_MS)

    def set_master_gain_code(self, code: int):
        if code > 95 or code < 0:
            raise RuntimeError
        self.write_register_masked(0x42, code, 0x007F)
        self.write_register_masked(0x02, code, 0x007F)
        utime.sleep_ms(CONFIG_WAIT_MS)

    def set_fine_gain_1(self, code: int):
        if code > 0x7F or code < 0:
            raise RuntimeError
        self.write_register_masked(0x43, code, 0x007F)
        self.write_register_masked(0x03, code, 0x007F)
        utime.sleep_ms(CONFIG_WAIT_MS)

    def set_fine_gain_2(self, code: int):
        if code > 0x7F or code < 0:
            raise RuntimeError
        self.write_register_masked(0x45, code, 0x007F)
        self.write_register_masked(0x05, code, 0x007F)
        utime.sleep_ms(CONFIG_WAIT_MS)

    def set_offset_1(self, sign: int, code: int):
        if code > 0x7F or code < 0:
            raise RuntimeError
        sign_code = (1 if sign < 0 else 0) << 7
        self.write_register_masked(0x44, sign_code | code, 0x00FF)
        self.write_register_masked(0x04, sign_code | code, 0x00FF)
        utime.sleep_ms(CONFIG_WAIT_MS)

    def set_offset_2(self, sign: int, code: int):
        if code > 0x7F or code < 0:
            raise RuntimeError
        sign_code = (1 if sign < 0 else 0) << 7
        self.write_register_masked(0x46, sign_code | code, 0x00FF)
        self.write_register_masked(0x06, sign_code | code, 0x00FF)
        utime.sleep_ms(CONFIG_WAIT_MS)

    def set_current_bias(self, code: int):
        if code < 0 or code > 0xFF:
            raise RuntimeError
        self.write_register_masked(0x47, code, 0x00FF)
        self.write_register_masked(0x07, code, 0x00FF)
        utime.sleep_ms(CONFIG_WAIT_MS)

    def set_output_mode(self, mode: int):
        if mode != OM_DIFFERENTIAL and mode != OM_SINGLE_ENDED:
            raise RuntimeError
        self.write_register_masked(0x40, mode << 1, 0b1 << 1)
        self.write_register_masked(0x00, mode << 1, 0b1 << 1)
        utime.sleep_ms(CONFIG_WAIT_MS)

    def get_tx_frequency(self) -> float:
        return self.read_register_masked(0x6E, 0x07FF) * 20000.0

    def get_vdd(self) -> float:
        code = self.read_register_masked(0x01, 0b1)
        return VDD_3V3 if code == 0 else VDD_5V0

    def get_output_mode(self) -> int:
        code = self.read_register_masked(0x00, 0b1 << 1)
        return OM_DIFFERENTIAL if code == 0 else OM_SINGLE_ENDED

    def get_automatic_gain_control(self) -> bool:
        return self.read_register_masked(0x00, 0b1 << 9) == 0

    def get_master_gain_code(self) -> int:
        return self.read_register_masked(0x02, 0x007F)

    def get_master_gain(self) -> float:
        return GAIN_FACTORS[self.get_master_gain_code()]

    def get_master_gain_boost(self) -> bool:
        return self.read_register_masked(0x02, 0b1 << 7) == 1

    def get_fine_gain_1_code(self) -> int:
        return self.read_register_masked(0x03, 0x007F)

    def get_fine_gain_2_code(self) -> int:
        return self.read_register_masked(0x05, 0x007F)

    def get_fine_gain_1(self) -> float:
        return 1.0 + self.get_fine_gain_1_code() * 0.125 / 100.0

    def get_fine_gain_2(self) -> float:
        return 1.0 + self.get_fine_gain_2_code() * 0.125 / 100.0

    def get_offset_sign_1(self) -> int:
        return 1 if self.read_register_masked(0x04, 0b1 << 7) == 0 else -1

    def get_offset_sign_2(self) -> int:
        return 1 if self.read_register_masked(0x06, 0b1 << 7) == 0 else -1

    def get_offset_code_1(self) -> int:
        return self.read_register_masked(0x04, 0x007F)

    def get_offset_code_2(self) -> int:
        return self.read_register_masked(0x06, 0x007F)

    def get_offset_1_perc(self) -> float:
        return self.get_offset_sign_1() * self.get_offset_code_1() * 0.0015 / 100.0

    def get_offset_2_perc(self) -> float:
        return self.get_offset_sign_2() * self.get_offset_code_2() * 0.0015 / 100.0

    def get_tx_current_bias_uA(self) -> float:
        code = self.read_register_masked(0x07, 0x00FF)
        multiplier = 2 ** ((code >> 6) * 2)
        base = code & 0x003F
        return multiplier * base * 0.5

    def get_rx1(self) -> float:
        return read_adc_voltage(self._rx1) - read_adc_voltage(self._ref)

    def get_rx2(self) -> float:
        return read_adc_voltage(self._rx2) - read_adc_voltage(self._ref)

    def get_rx1_avg(self, nsamples: int = 10, delay_ms: int = 25) -> float:
        rx = 0.0
        for i in range(nsamples):
            rx += self.get_rx1()
            utime.sleep_ms(delay_ms)
        rx /= nsamples
        return rx

    def get_rx2_avg(self, nsamples: int = 10, delay_ms: int = 25) -> float:
        rx = 0.0
        for i in range(nsamples):
            rx += self.get_rx2()
            utime.sleep_ms(delay_ms)
        rx /= nsamples
        return rx

    def estimate_vtx_rms(self) -> float:
        vtx = 0.0

        starting_offset_sign = self.get_offset_sign_1()
        starting_offset = self.get_offset_code_1()
        gain = self.get_master_gain()
        if self.get_master_gain_boost():
            gain *= 2

        # set offset to min (-0x7F)
        self.set_offset_1(-1, 0x7F)
        rx1_n = self.get_rx1_avg()

        # set offset to max (0x7F)
        self.set_offset_1(1, 0x7F)
        rx1_p = self.get_rx1_avg()

        # go back to original offset
        self.set_offset_1(starting_offset_sign, starting_offset)

        # rx1_p = gain * (v + op*Vtx_rms)
        # rx1_n = gain * (v + on*Vtx_rms)
        # rx1_p - rx1_n = gain * (op - on) * Vtx_rms
        # -> Vtx_rms = (rx1_p - rx1_n) / (gain * (op - on))

        on = -0x7F * 0.000015
        op = 0x7F * 0.000015
        drx = rx1_p - rx1_n

        vtx = drx / (gain * (op - on))

        return abs(vtx)

    def estimate_vtx(self) -> float:
        return self.estimate_vtx_rms() * math.sqrt(2)

    def estimate_vtx_pp(self) -> float:
        return self.estimate_vtx() * 2.0


if __name__ == "__main__":
    print("This is a library file, not an executable.")
