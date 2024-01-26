#!/usr/bin/env python3
import logging
import time
import math
import os
from time import sleep

timer = [
    {
        "arr": 0
    }
] * 4

#As is from sunfounder
class _Basic_class(object):
    _class_name = '_Basic_class'
    DEBUG_LEVELS = {'debug': logging.DEBUG,
              'info': logging.INFO,
              'warning': logging.WARNING,
              'error': logging.ERROR,
              'critical': logging.CRITICAL,
              }
    DEBUG_NAMES = ['critical', 'error', 'warning', 'info', 'debug']

    def __init__(self):
        self._debug_level = 0
        self.logger = logging.getLogger(self._class_name)
        self.ch = logging.StreamHandler()
        form = "%(asctime)s	[%(levelname)s]	%(message)s"
        self.formatter = logging.Formatter(form)
        self.ch.setFormatter(self.formatter)
        self.logger.addHandler(self.ch)
        self._debug    = self.logger.debug
        self._info     = self.logger.info
        self._warning  = self.logger.warning
        self._error    = self.logger.error
        self._critical = self.logger.critical

    @property
    def debug(self):
        return self._debug_level

    @debug.setter
    def debug(self, debug):
        if debug in range(5):
            self._debug_level = self.DEBUG_NAMES[debug]
        elif debug in self.DEBUG_NAMES:
            self._debug_level = debug
        else:
            raise ValueError('Debug value must be 0(critical), 1(error), 2(warning), 3(info) or 4(debug), not \"{0}\".'.format(debug))  
        self.logger.setLevel(self.DEBUG_LEVELS[self._debug_level])
        self.ch.setLevel(self.DEBUG_LEVELS[self._debug_level])
        self._debug('Set logging level to [%s]' % self._debug_level)

    def run_command(self, cmd):
        import subprocess
        p = subprocess.Popen(
            cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        result = p.stdout.read().decode('utf-8')
        status = p.poll()
        # print(result)
        # print(status)
        return status, result

    def map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

#Modified with ChatGPT
class Pin(_Basic_class):
    OUT = "OUT"
    IN = "IN"
    IRQ_FALLING = "IRQ_FALLING"
    IRQ_RISING = "IRQ_RISING"
    IRQ_RISING_FALLING = "IRQ_RISING_FALLING"
    PULL_UP = "PULL_UP"
    PULL_DOWN = "PULL_DOWN"
    PULL_NONE = "PULL_NONE"

    def __init__(self, *value):
        self._pin = None
        self._mode = None
        self._pull = None
        self._board_name = None

        if len(value) > 0:
            pin = value[0]
        if len(value) > 1:
            mode = value[1]
        else:
            mode = None
        if len(value) > 2:
            setup = value[2]
        else:
            setup = None

        if isinstance(pin, str):
            self._board_name = pin
        elif isinstance(pin, int):
            self._pin = pin
        else:
            self._error('Pin should be either a string or an integer, not %s' % type(pin))

        self._value = 0
        self.init(mode, pull=setup)

    def init(self, mode, pull=None):
        self._pull = pull
        self._mode = mode

    def value(self, *value):
        if len(value) == 0:
            if self._mode in [None, self.OUT]:
                self.mode(self.IN)
            result = self._value
            self._debug("read pin %s: %s" % (self._pin, result))
            return result
        else:
            value = value[0]
            if self._mode in [None, self.IN]:
                self.mode(self.OUT)
            self._value = value
            return value

    def on(self):
        return self.value(1)

    def off(self):
        return self.value(0)

    def high(self):
        return self.on()

    def low(self):
        return self.off()

    def mode(self, *value):
        if len(value) == 0:
            return (self._mode, self._pull)
        else:
            self._mode = value[0]
            if len(value) == 1:
                pass
            elif len(value) == 2:
                self._pull = value[1]

    def pull(self, *value):
        return self._pull

    def irq(self, handler=None, trigger=None, bouncetime=200):
        # For simplicity, do nothing in the debug version
        pass

    def name(self):
        return "DebugGPIO%s" % self._pin

    def names(self):
        return [self.name, self._board_name]

    class cpu:
        GPIO17 = 17
        GPIO18 = 18
        GPIO27 = 27
        GPIO22 = 22
        GPIO23 = 23
        GPIO24 = 24
        GPIO25 = 25
        GPIO26 = 26
        GPIO4 = 4
        GPIO5 = 5
        GPIO6 = 6
        GPIO12 = 12
        GPIO13 = 13
        GPIO19 = 19
        GPIO16 = 16
        GPIO26 = 26
        GPIO20 = 20
        GPIO21 = 21

        def __init__(self):
            pass


#As is from sunfounder
def _retry_wrapper(func):
    def wrapper(self, *arg, **kwargs):
        for i in range(self.RETRY):
            try:
                return func(self, *arg, **kwargs)
            except OSError:
                self._debug("OSError: %s" % func.__name__)
                continue
        else:
            return False
    return wrapper

#Made using ChatGPT
class DummySMBus:
    def write_byte(self, addr, data):
        #print(f"DummySMBus: Write byte - Address: 0x{addr:02X}, Data: 0x{data:02X}")
        return 0  # Return value for simulation

    def write_byte_data(self, addr, reg, data):
        #print(f"DummySMBus: Write byte data - Address: 0x{addr:02X}, Register: 0x{reg:02X}, Data: 0x{data:02X}")
        return 0  # Return value for simulation

    def write_word_data(self, addr, reg, data):
        #print(f"DummySMBus: Write word data - Address: 0x{addr:02X}, Register: 0x{reg:02X}, Data: 0x{data:04X}")
        return 0  # Return value for simulation

    def write_i2c_block_data(self, addr, reg, data):
        #print(f"DummySMBus: Write i2c block data - Address: 0x{addr:02X}, Register: 0x{reg:02X}, Data: {data}")
        return 0  # Return value for simulation

    def read_byte(self, addr):
        #print(f"DummySMBus: Read byte - Address: 0x{addr:02X}")
        return 0xFF  # Return value for simulation

    def read_i2c_block_data(self, addr, reg, num):
        #print(f"DummySMBus: Read i2c block data - Address: 0x{addr:02X}, Register: 0x{reg:02X}, Num: {num}")
        return [0xFF] * num  # Return value for simulation

#Modified slightly to use the ChatGPT-generated DummySMBus class
class I2C(_Basic_class):
    MASTER = 0
    SLAVE  = 1
    RETRY = 5

    def __init__(self, *args, **kargs):     # *args表示位置参数（形式参数），可无，； **kargs表示默认值参数，可无。
        super().__init__()
        self._bus = 1
        self._smbus = DummySMBus()

    @_retry_wrapper
    def _i2c_write_byte(self, addr, data):   # i2C 写系列函数
        #self._debug("_i2c_write_byte: [0x{:02X}] [0x{:02X}]".format(addr, data))
        result = self._smbus.write_byte(addr, data)
        return result
            
    @_retry_wrapper
    def _i2c_write_byte_data(self, addr, reg, data):
        #self._debug("_i2c_write_byte_data: [0x{:02X}] [0x{:02X}] [0x{:02X}]".format(addr, reg, data))
        return self._smbus.write_byte_data(addr, reg, data)
    
    @_retry_wrapper
    def _i2c_write_word_data(self, addr, reg, data):
        #self._debug("_i2c_write_word_data: [0x{:02X}] [0x{:02X}] [0x{:04X}]".format(addr, reg, data))
        return self._smbus.write_word_data(addr, reg, data)
    
    @_retry_wrapper
    def _i2c_write_i2c_block_data(self, addr, reg, data):
        #self._debug("_i2c_write_i2c_block_data: [0x{:02X}] [0x{:02X}] {}".format(addr, reg, data))
        return self._smbus.write_i2c_block_data(addr, reg, data)
    
    @_retry_wrapper
    def _i2c_read_byte(self, addr):   # i2C 读系列函数
        #self._debug("_i2c_read_byte: [0x{:02X}]".format(addr))
        return self._smbus.read_byte(addr)

    @_retry_wrapper
    def _i2c_read_i2c_block_data(self, addr, reg, num):
        #self._debug("_i2c_read_i2c_block_data: [0x{:02X}] [0x{:02X}] [{}]".format(addr, reg, num))
        return self._smbus.read_i2c_block_data(addr, reg, num)

    @_retry_wrapper
    def is_ready(self, addr):
        addresses = self.scan()
        if addr in addresses:
            return True
        else:
            return False

    def scan(self):                             # 查看有哪些i2c设备
        cmd = "i2cdetect -y %s" % self._bus
        _, output = self.run_command(cmd)          # 调用basic中的方法，在linux中运行cmd指令，并返回运行后的内容
        
        outputs = output.split('\n')[1:]        # 以回车符为分隔符，分割第二行之后的所有行
        #self._debug("outputs")
        addresses = []
        for tmp_addresses in outputs:
            if tmp_addresses == "":
                continue
            tmp_addresses = tmp_addresses.split(':')[1]
            tmp_addresses = tmp_addresses.strip().split(' ')    # strip函数是删除字符串两端的字符，split函数是分隔符
            for address in tmp_addresses:
                if address != '--':
                    addresses.append(int(address, 16))
        #self._debug("Conneceted i2c device: %s"%addresses)                   # append以列表的方式添加address到addresses中
        return addresses


    def send(self, send, addr, timeout=0):                      # 发送数据，addr为从机地址，send为数据
        if isinstance(send, bytearray):
            data_all = list(send)
        elif isinstance(send, int):
            data_all = []
            d = "{:X}".format(send)
            d = "{}{}".format("0" if len(d)%2 == 1 else "", d)  # format是将()中的内容对应填入{}中，（）中的第一个参数是一个三目运算符，if条件成立则为“0”，不成立则为“”(空的意思)，第二个参数是d，此行代码意思为，当字符串为奇数位时，在字符串最强面添加‘0’，否则，不添加， 方便以下函数的应用
            # print(d)
            for i in range(len(d)-2, -1, -2):       # 从字符串最后开始取，每次取2位
                tmp = int(d[i:i+2], 16)             # 将两位字符转化为16进制
                # print(tmp)
                data_all.append(tmp)                # 添加到data_all数组中
            data_all.reverse()
        elif isinstance(send, list):
            data_all = send
        else:
            raise ValueError("send data must be int, list, or bytearray, not {}".format(type(send)))

        if len(data_all) == 1:                      # 如果data_all只有一组数
            data = data_all[0]
            # print("i2c write: [0x%02X] to 0x%02X"%(data, addr))
            self._i2c_write_byte(addr, data)
        elif len(data_all) == 2:                    # 如果data_all只有两组数
            reg = data_all[0]
            data = data_all[1]
            self._i2c_write_byte_data(addr, reg, data)
        elif len(data_all) == 3:                    # 如果data_all只有三组数
            reg = data_all[0]
            data = (data_all[2] << 8) + data_all[1]
            self._i2c_write_word_data(addr, reg, data)
        else:
            reg = data_all[0]
            data = list(data_all[1:])
            self._i2c_write_i2c_block_data(addr, reg, data)

    def recv(self, recv, addr=0x00, timeout=0):     # 接收数据
        if isinstance(recv, int):                   # 将recv转化为二进制数
            result = bytearray(recv)
        elif isinstance(recv, bytearray):
            result = recv
        else:
            return False
        for i in range(len(result)):
            result[i] = self._i2c_read_byte(addr)
        return result

    def mem_write(self, data, addr, memaddr, timeout=5000, addr_size=8): #memaddr match to chn
        if isinstance(data, bytearray):
            data_all = list(data)
        elif isinstance(data, list):
            data_all = data
        elif isinstance(data, int):
            data_all = []
            data = "%x"%data
            if len(data) % 2 == 1:
                data = "0" + data
            # print(data)
            for i in range(0, len(data), 2):
                # print(data[i:i+2])
                data_all.append(int(data[i:i+2], 16))
        else:
            raise ValueError("memery write require arguement of bytearray, list, int less than 0xFF")
        # print(data_all)
        self._i2c_write_i2c_block_data(addr, memaddr, data_all)


    @_retry_wrapper 
    def mem_read(self, data, addr, memaddr, timeout=5000, addr_size=8):     # 读取数据
        if isinstance(data, int):
            num = data
        elif isinstance(data, bytearray):
            num = len(data)
        else:
            return False
        result = bytearray(self._i2c_read_i2c_block_data(addr, memaddr, num))
        return result
    
    def readfrom_mem_into(self, addr, memaddr, buf):
        buf = self.mem_read(len(buf), addr, memaddr)
        return buf
    
    def writeto_mem(self, addr, memaddr, data):
        self.mem_write(data, addr, memaddr)

#As is from sunfounder
class ADC(I2C):
    ADDR=0x14                   # 扩展板的地址为0x14

    def __init__(self, chn):    # 参数，通道数，树莓派扩展板上有8个adc通道分别为"A0, A1, A2, A3, A4, A5, A6, A7"
        super().__init__()
        if isinstance(chn, str):
            if chn.startswith("A"):     # 判断穿境来的参数是否为A开头，如果是，取A后面的数字出来
                chn = int(chn[1:])
            else:
                raise ValueError("ADC channel should be between [A0, A7], not {0}".format(chn))
        if chn < 0 or chn > 7:          # 判断取出来的数字是否在0~7的范围内
            self._error('Incorrect channel range')
        chn = 7 - chn
        self.chn = chn | 0x10           # 给从机地址
        self.reg = 0x40 + self.chn
        # self.bus = smbus.DummySMBus(1)
        
    def read(self):                     # adc通道读取数---写一次数据，读取两次数据 （读取的数据范围是0~4095）
        self._debug("Write 0x%02X to 0x%02X"%(self.chn, self.ADDR))
        # self.bus.write_byte(self.ADDR, self.chn)      # 写入数据
        self.send([self.chn, 0, 0], self.ADDR)

        self._debug("Read from 0x%02X"%(self.ADDR))
        # value_h = self.bus.read_byte(self.ADDR)
        value_h = self.recv(1, self.ADDR)[0]            # 读取数据

        self._debug("Read from 0x%02X"%(self.ADDR))
        # value_l = self.bus.read_byte(self.ADDR)
        value_l = self.recv(1, self.ADDR)[0]            # 读取数据（读两次）

        value = (value_h << 8) + value_l
        self._debug("Read value: %s"%value)
        return value

    def read_voltage(self):                             # 将读取的数据转化为电压值（0~3.3V）
        return self.read*3.3/4095

#Modified slightly to use the ChatGPT-generated DummySMBus class
class PWM(I2C):
    REG_CHN = 0x20
    REG_FRE = 0x30
    REG_PSC = 0x40
    REG_ARR = 0x44

    ADDR = 0x14

    CLOCK = 72000000

    def __init__(self, channel, debug="critical"):
        super().__init__()
        if isinstance(channel, str):
            if channel.startswith("P"):
                channel = int(channel[1:])
                if channel > 14:
                    raise ValueError("channel must be in range of 0-14")
            else:
                raise ValueError("PWM channel should be between [P0, P11], not {0}".format(channel))
        try:
            self.send(0x2C, self.ADDR)
            self.send(0, self.ADDR)
            self.send(0, self.ADDR)
        except IOError:
            self.ADDR = 0x15

        self.debug = debug
        self._debug("PWM address: {:02X}".format(self.ADDR))
        self.channel = channel
        self.timer = int(channel/4)
        self.bus = DummySMBus()
        self._pulse_width = 0
        self._freq = 50
        self.freq(50)

    def i2c_write(self, reg, value):
        value_h = value >> 8
        value_l = value & 0xff
        self._debug("i2c write: [0x%02X, 0x%02X, 0x%02X, 0x%02X]"%(self.ADDR, reg, value_h, value_l))
        # print("i2c write: [0x%02X, 0x%02X, 0x%02X] to 0x%02X"%(reg, value_h, value_l, self.ADDR))
        self.send([reg, value_h, value_l], self.ADDR)

    def freq(self, *freq):
        if len(freq) == 0:
            return self._freq
        else:
            self._freq = int(freq[0])
            # [prescaler,arr] list
            result_ap = []
            # accuracy list
            result_acy = []
            # middle value for equal arr prescaler
            st = int(math.sqrt(self.CLOCK/self._freq))
            # get -5 value as start
            st -= 5
            # prevent negetive value
            if st <= 0:
                st = 1
            for psc in range(st,st+10):
                arr = int(self.CLOCK/self._freq/psc)
                result_ap.append([psc, arr])
                result_acy.append(abs(self._freq-self.CLOCK/psc/arr))
            i = result_acy.index(min(result_acy))
            psc = result_ap[i][0]
            arr = result_ap[i][1]
            self._debug("prescaler: %s, period: %s"%(psc, arr))
            self.prescaler(psc)
            self.period(arr)

    def prescaler(self, *prescaler):
        if len(prescaler) == 0:
            return self._prescaler
        else:
            self._prescaler = int(prescaler[0]) - 1
            reg = self.REG_PSC + self.timer
            self._debug("Set prescaler to: %s"%self._prescaler)
            self.i2c_write(reg, self._prescaler)

    def period(self, *arr):
        global timer
        if len(arr) == 0:
            return timer[self.timer]["arr"]
        else:
            timer[self.timer]["arr"] = int(arr[0]) - 1
            reg = self.REG_ARR + self.timer
            self._debug("Set arr to: %s"%timer[self.timer]["arr"])
            self.i2c_write(reg, timer[self.timer]["arr"])

    def pulse_width(self, *pulse_width):
        if len(pulse_width) == 0:
            return self._pulse_width
        else:
            self._pulse_width = int(pulse_width[0])
            reg = self.REG_CHN + self.channel
            self.i2c_write(reg, self._pulse_width)

    def pulse_width_percent(self, *pulse_width_percent):
        global timer
        if len(pulse_width_percent) == 0:
            return self._pulse_width_percent
        else:
            self._pulse_width_percent = pulse_width_percent[0]
            temp = self._pulse_width_percent / 100.0
            # print(temp)
            pulse_width = temp * timer[self.timer]["arr"]
            self.pulse_width(pulse_width)

class Servo(_Basic_class):
    MAX_PW = 2500
    MIN_PW = 500
    _freq = 50
    def __init__(self, pwm):
        super().__init__()
        self.pwm = PWM(pwm)
        self.pwm.period(4095)
        prescaler = int(float(self.pwm.CLOCK) /self.pwm._freq/self.pwm.period())
        self.pwm.prescaler(prescaler)
        # self.angle(90)

    # angle ranges -90 to 90 degrees
    def angle(self, angle):
        if not (isinstance(angle, int) or isinstance(angle, float)):
            raise ValueError("Angle value should be int or float value, not %s"%type(angle))
        if angle < -90:
            angle = -90
        if angle > 90:
            angle = 90
        High_level_time = self.map(angle, -90, 90, self.MIN_PW, self.MAX_PW)
        self._debug("High_level_time: %f" % High_level_time)
        pwr =  High_level_time / 20000
        self._debug("pulse width rate: %f" % pwr)
        value = int(pwr*self.pwm.period())
        self._debug("pulse width value: %d" % value)
        self.pwm.pulse_width(value)

    # pwm_value ranges MIN_PW 500 to MAX_PW 2500 degrees
    def set_pwm(self,pwm_value):
        if pwm_value > self.MAX_PW:
            pwm_value =  self.MAX_PW 
        if pwm_value < self.MIN_PW:
            pwm_value = self.MIN_PW

        self.pwm.pulse_width(pwm_value)

class fileDB(object):
    """A file based database.

    A file based database, read and write arguements in the specific file.
    """
    def __init__(self, db:str, mode:str=None, owner:str=None):  
        pass


    def file_check_create(self, file_path:str, mode:str=None, owner:str=None):
        pass
    
    def get(self, name, default_value=None):
        return default_value
    
    def set(self, name, value):
        pass


class Ultrasonic():
    def __init__(self, trig, echo, timeout=0.02):
        self.trig = trig
        self.echo = echo
        self.timeout = timeout

    def _read(self):
        self.trig.low()
        time.sleep(0.01)
        self.trig.high()
        time.sleep(0.00001)
        self.trig.low()
        pulse_end = 0
        pulse_start = 0
        timeout_start = time.time()
        while self.echo.value()==0:
            pulse_start = time.time()
            if pulse_start - timeout_start > self.timeout:
                return -1
        while self.echo.value()==1:
            pulse_end = time.time()
            if pulse_end - timeout_start > self.timeout:
                return -1
        during = pulse_end - pulse_start
        cm = round(during * 340 / 2 * 100, 2)
        return cm

    def read(self, times=10):
        for i in range(times):
            a = self._read()
            if a != -1:
                return a
        return -1

class Grayscale_Module(object):

    REFERENCE_DEFAULT = [1000]*3

    def __init__(self, pin0, pin1, pin2, reference=None):
        
        if isinstance(pin0,str):
            self.chn_0 = ADC(pin0)
            self.chn_1 = ADC(pin1)
            self.chn_2 = ADC(pin2)
        else:
            self.chn_0 = ADC('A0')
            self.chn_1 = ADC('A1')
            self.chn_2 = ADC('A2')
            
        if reference is None:
            self.reference = self.REFERENCE_DEFAULT
        else:
            self.set_reference(reference)

    def set_reference(self, reference):
        if isinstance(reference, int) or isinstance(reference, float):
            self.reference = [reference] * 3
        elif isinstance(reference, list) and len(reference) != 3:
            self.reference = reference
        else:
            raise TypeError("reference parameter must be \'int\', \'float\', or 1*3 list.")

    def get_line_status(self,fl_list):

        if fl_list[0] > self.reference[0] and fl_list[1] > self.reference[1] and fl_list[2] > self.reference[2]:
            return 'stop'
            
        elif fl_list[1] <= self.reference[1]:
            return 'forward'
        
        elif fl_list[0] <= self.reference[0]:
            return 'right'

        elif fl_list[2] <= self.reference[2]:
            return 'left'

    def get_grayscale_data(self):
        adc_value_list = []
        adc_value_list.append(self.chn_0.read())
        adc_value_list.append(self.chn_1.read())
        adc_value_list.append(self.chn_2.read())
        return adc_value_list
    
    def read(self):
        return self.get_grayscale_data()

def run_command(cmd):
    import subprocess
    p = subprocess.Popen(
        cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    result = p.stdout.read().decode('utf-8')
    status = p.poll()
    return status, result



def reset_mcu():
    mcu_reset = Pin("MCURST")
    mcu_reset.off()
    time.sleep(0.001)
    mcu_reset.on() 
    time.sleep(0.01)  