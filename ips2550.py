import machine
import utime


def read_adc_voltage(adc: machine.ADC, vdd: float = 3.3):
    return adc.read_u16() / 65536 * vdd


def bit_length(n: int) -> int:
    i = 1
    while (n >> i) != 0:
        i += 1
    return i


def crc(word: int, polynomial: int, filler: int = 0) -> int:
    g = bit_length(polynomial)
    n = g - 1
    word = (word << n) | filler
    while (word >> n) != 0:
        first_one = bit_length(word)
        xor_mask = ~(0xFF << g) << (first_one - g)
        xor_val = polynomial << (first_one - g)
        word = ((word & xor_mask) ^ xor_val) | (word & ~xor_mask)

    return word & ~(0xFF << g)


def get_bits_in_word(word, bits_list):
    res = 0x00
    for bit in bits_list:
        res = res << 1
        res |= (word & (1 << bit)) >> bit
    return res


VDD_3V3 = 0
VDD_5V0 = 1

OM_DIFFERENTIAL = 0
OM_SINGLE_ENDED = 1

GAIN_FACTORS = [
    2.0,
    2.1,
    2.18,
    2.29,
    2.38,
    2.5,
    2.59,
    2.72,
    2.83,
    2.97,
    3.09,
    3.24,
    3.36,
    3.53,
    3.67,
    3.85,
    4.0,
    4.2,
    4.36,
    4.58,
    4.76,
    4.99,
    5.19,
    5.45,
    5.66,
    5.94,
    6.17,
    6.48,
    6.73,
    7.06,
    7.34,
    7.7,
    8.0,
    8.4,
    8.72,
    9.16,
    9.51,
    9.99,
    10.38,
    10.89,
    11.31,
    11.88,
    12.34,
    12.96,
    13.46,
    14.13,
    14.67,
    15.41,
    16.0,
    16.8,
    17.45,
    18.32,
    19.02,
    19.98,
    20.75,
    21.79,
    22.62,
    23.76,
    24.68,
    25.91,
    26.91,
    28.26,
    29.34,
    30.81,
    32.0,
    33.6,
    34.9,
    36.64,
    38.05,
    39.95,
    41.5,
    43.58,
    45.25,
    47.51,
    49.36,
    51.83,
    53.82,
    56.52,
    58.69,
    61.62,
    64.0,
    67.2,
    69.79,
    73.28,
    76.1,
    79.9,
    83.01,
    87.16,
    90.5,
    95.02,
    98.72,
    103.66,
    107.65,
    113.03,
    117.38,
    123.24,
]


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

    def read_reg(self, reg_addr):
        data = self._i2c.readfrom_mem(self._i2c_addr, reg_addr, 2)
        data = (data[0] << 8) | data[1]
        crc_bits = data & 0b111
        reg = data >> 5

        message = ((reg << 5) & 0xFF00) | (reg & 0b111)
        if crc(message, 0b1011, crc_bits) != 0:
            raise AssertionError("CRC check failed on read")

        return reg

    def write_reg(self, reg_addr, value):
        crc_in = (
            (((reg_addr & 0b0000_0000_0111_1111) << 1) << 16)
            | (((value & 0b0000_0111_1111_1000) >> 3) << 8)
            | (value & 0b0000_0000_0000_0111)
        )
        crc_bits = crc(crc_in, 0b1011)
        message = int((value << 5) | 0b11000 | crc_bits)
        self._i2c.writeto_mem(self._i2c_addr, reg_addr, message.to_bytes(2, "big"))

    def write_register_bits(self, reg_addr, bit_list, value_list):
        register = self.read_reg(reg_addr)
        mask_clear = 0
        for bit in bit_list:
            mask_clear = mask_clear | (1 << bit)
        newword = register & ~mask_clear
        mask_set = 0
        for bit, value in zip(bit_list, value_list):
            mask_set = mask_set | (value << bit)
        newword = newword | mask_set
        self.write_reg(reg_addr, newword)

    def set_voltage(self, vdd: int):
        if vdd != VDD_3V3 and vdd != VDD_5V0:
            raise RuntimeError
        self.write_register_bits(0x41, [0], [vdd])
        utime.sleep_ms(50)
        self.write_register_bits(0x01, [0], [vdd])
        utime.sleep_ms(50)

    def set_automatic_gain_control(self, enabled: bool):
        bit_val = 0 if enabled else 1
        self.write_register_bits(0x00, [9], [bit_val])
        utime.sleep_ms(50)
        self.write_register_bits(0x40, [9], [bit_val])
        utime.sleep_ms(50)

    def set_master_gain_boost(self, enabled: bool):
        bit_val = 1 if enabled else 0
        self.write_register_bits(0x02, [7], [bit_val])
        utime.sleep_ms(50)
        self.write_register_bits(0x42, [7], [bit_val])
        utime.sleep_ms(50)

    def set_master_gain_code(self, code: int):
        if code > 95 or code < 0:
            raise RuntimeError
        bit_val = [int(a) for a in bin(code)[2:]]
        bit_val = (7 - len(bit_val)) * [0] + bit_val
        self.write_register_bits(0x42, list(reversed(range(7))), bit_val)
        utime.sleep_ms(50)
        self.write_register_bits(0x02, list(reversed(range(7))), bit_val)
        utime.sleep_ms(50)

    def set_fine_gain_1(self, code: int):
        if code > 0x7F or code < 0:
            raise RuntimeError
        bit_val = [int(a) for a in bin(code)[2:]]
        bit_val = (7 - len(bit_val)) * [0] + bit_val
        self.write_register_bits(0x03, list(reversed(range(7))), bit_val)
        utime.sleep_ms(50)
        self.write_register_bits(0x43, list(reversed(range(7))), bit_val)
        utime.sleep_ms(50)

    def set_fine_gain_2(self, code: int):
        if code > 0x7F or code < 0:
            raise RuntimeError
        bit_val = [int(a) for a in bin(code)[2:]]
        bit_val = (7 - len(bit_val)) * [0] + bit_val
        self.write_register_bits(0x05, list(reversed(range(7))), bit_val)
        utime.sleep_ms(50)
        self.write_register_bits(0x45, list(reversed(range(7))), bit_val)
        utime.sleep_ms(50)

    def set_offset_1(self, sign: int, code: int):
        if code > 0x7F or code < 0:
            raise RuntimeError
        sign = 0 if sign < 0 else 1
        bit_val = [int(a) for a in bin(code)[2:]]
        bit_val = [sign] + (7 - len(bit_val)) * [0] + bit_val
        self.write_register_bits(0x04, list(reversed(range(8))), bit_val)
        utime.sleep_ms(50)
        self.write_register_bits(0x44, list(reversed(range(8))), bit_val)
        utime.sleep_ms(50)

    def set_offset_2(self, sign: int, code: int):
        if code > 0x7F or code < 0:
            raise RuntimeError
        sign = 0 if sign < 0 else 1
        bit_val = [int(a) for a in bin(code)[2:]]
        bit_val = [sign] + (7 - len(bit_val)) * [0] + bit_val
        self.write_register_bits(0x06, list(reversed(range(8))), bit_val)
        utime.sleep_ms(50)
        self.write_register_bits(0x46, list(reversed(range(8))), bit_val)
        utime.sleep_ms(50)

    def set_current_bias(self, code: int):
        if code < 0 or code > 0xFF:
            raise RuntimeError
        bit_val = [int(a) for a in bin(code)[2:]]
        bit_val = (8 - len(bit_val)) * [0] + bit_val
        self.write_register_bits(0x07, list(reversed(range(8))), bit_val)
        utime.sleep_ms(50)
        self.write_register_bits(0x47, list(reversed(range(8))), bit_val)
        utime.sleep_ms(50)

    def set_output_mode(self, mode: int):
        if mode != OM_DIFFERENTIAL and mode != OM_SINGLE_ENDED:
            raise RuntimeError

        self.write_register_bits(0x00, [1], [mode])
        utime.sleep_ms(50)
        self.write_register_bits(0x40, [1], [mode])
        utime.sleep_ms(50)

    def get_tx_frequency(self) -> float:
        tx_cnt_reg = self.read_reg(0x6E)
        return get_bits_in_word(tx_cnt_reg, list(reversed(range(11)))) * 20_000.0

    def get_vdd(self) -> float:
        reg = self.read_reg(0x01)
        return 5.0 if get_bits_in_word(reg, [0]) == VDD_5V0 else 3.3

    def get_output_mode(self) -> int:
        reg = self.read_reg(0x00)
        return get_bits_in_word(reg, [1])

    def get_automatic_gain_control(self) -> bool:
        reg = self.read_reg(0x00)
        return get_bits_in_word(reg, [9]) == 0

    def get_master_gain_code(self) -> int:
        reg = self.read_reg(0x02)
        return get_bits_in_word(reg, list(reversed(range(7))))

    def get_master_gain(self) -> float:
        reg = self.read_reg(0x02)
        return GAIN_FACTORS[get_bits_in_word(reg, list(reversed(range(7))))]

    def get_master_gain_boost(self) -> bool:
        reg = self.read_reg(0x02)
        return get_bits_in_word(reg, [7]) == 1

    def get_fine_gain_1_code(self) -> int:
        reg = self.read_reg(0x03)
        return get_bits_in_word(reg, list(reversed(range(7))))

    def get_fine_gain_2_code(self) -> int:
        reg = self.read_reg(0x05)
        return get_bits_in_word(reg, list(reversed(range(7))))

    def get_fine_gain_1(self) -> float:
        return 1.0 + self.get_fine_gain_1_code() * 0.125 / 100.0 * 2.0

    def get_fine_gain_2(self) -> float:
        return 1.0 + self.get_fine_gain_2_code() * 0.125 / 100.0 * 2.0

    def get_offset_sign_1(self) -> int:
        reg = self.read_reg(0x04)
        return 1 if get_bits_in_word(reg, [7]) == 0 else -1

    def get_offset_sign_2(self) -> int:
        reg = self.read_reg(0x06)
        return 1 if get_bits_in_word(reg, [7]) == 0 else -1

    def get_offset_code_1(self) -> int:
        reg = self.read_reg(0x04)
        return get_bits_in_word(reg, list(reversed(range(7))))

    def get_offset_code_2(self) -> int:
        reg = self.read_reg(0x06)
        return get_bits_in_word(reg, list(reversed(range(7))))

    def get_offset_1_perc(self) -> float:
        return self.get_offset_sign_1() * self.get_offset_code_1() * 4 * 0.0015 / 100.0

    def get_offset_2_perc(self) -> float:
        return self.get_offset_sign_2() * self.get_offset_code_2() * 4 * 0.0015 / 100.0

    def get_tx_current_bias_uA(self) -> float:
        reg = self.read_reg(0x07)
        code = get_bits_in_word(reg, list(reversed(range(8))))
        mul = code >> 6
        base = code & 0x3F
        return 2 ** (mul * 2) * 31.5 * base / 0x3F

    def get_rx1(self) -> float:
        return read_adc_voltage(self._rx1) - read_adc_voltage(self._ref)

    def get_rx2(self) -> float:
        return read_adc_voltage(self._rx2) - read_adc_voltage(self._ref)


if __name__ == "__main__":
    print("This is a library file, not an executable.")
