from enum import Enum


class L6470StatusMask(Enum):
    HIZ = 0x0001
    BUSY = 0x0002
    SW_F = 0x0004
    SW_EVN = 0x0008
    DIR = 0x0010
    NOTPERF_CMD = 0x0080
    WRONG_CMD = 0x0100
    UVLO = 0x0200
    TH_WRN = 0x0400
    TH_SD = 0x0800
    OCD = 0x1000
    STEP_LOSS_A = 0x2000
    STEP_LOSS_B = 0x4000
    SCK_MOD = 0x8000
    MOT_STATUS = 0x0096


class L6480StatusMask(Enum):
    HIZ = 0x0001
    BUSY = 0x0002
    SW_F = 0x0004
    SW_EVN = 0x0008
    DIR = 0x0010
    WRONG_CMD = 0x0080
    SCK_MOD = 0x0100
    UVLO = 0x0200
    UVLO_ADC = 0x0400
    OCD = 0x2000
    TH_STATUS = 0x3072
    STEP_LOSS_A = 0x4000
    STEP_LOSS_B = 0x8000
    MOT_STATUS = 0x0096


class ActiveType(Enum):
    active_low = 0
    active_high = 1
    none = 2


class Level(Enum):
    critical = 0
    warning = 1
    info = 2
    none = 3


class SimpleBit:

    style = {
        Level.critical: 'background-color: red;border-width: 2px;border-radius: 0px;border-color: red;font: bold 14px;padding: 6px;color: black;',
        Level.warning: 'background-color: yellow;border-width: 2px;border-radius: 0px;border-color: green;font: bold 14px;padding: 6px;color: black;',
        Level.info: 'background-color: green;border-width: 2px;border-radius: 0px;border-color: green;font: bold 14px;padding: 6px;color: black;',
        Level.none: ''
    }

    def __init__(self, name, bit_number, active_type, level):
        self.name = name
        self.number = bit_number
        self.active_type = active_type
        self.level = level
        self.last_status = None

    def active(self, status):
        if self.active_type == ActiveType.active_high:
            return status & self.mask
        if self.active_type == ActiveType.active_low:
            return not status & self.mask
        else:
            return False

    def update_view(self, view, status):
        current_status = self.active(status)

        if current_status == self.last_status:
            return

        if current_status:
            view.setStyleSheet(self.style[self.level])
        else:
            view.setStyleSheet(self.style[Level.none])

    @property
    def mask(self):
        return 1 << self.number


class ThermalStatus:

    possible_status = ['Normal', 'Warning', 'Bridge Shutdown', 'Device Shutdown']

    def __init__(self, name, small_bit_number):
        self.name = name
        self.number = small_bit_number

    def status(self, status):
        less_significant_bit = status & (1 << self.number)
        more_significant_bit = status & (1 << self.number + 1)

        if less_significant_bit and more_significant_bit:
            return self.possible_status[3]
        elif less_significant_bit:
            return self.possible_status[2]
        elif more_significant_bit:
            return self.possible_status[1]
        else:
            return self.possible_status[0]

    def update_view(self, view, status):
        view.setText(self.status(status))

    @property
    def mask(self):
        return (1 << self.number) | (1 << self.number+1)


class MotorStatus:
    possible_status = ['Stopped', 'Acceleration', 'Deceleration', 'Constant Speed']

    def __init__(self, name, small_bit_number):
        self.name = name
        self.number = small_bit_number

    def status(self, status):
        less_significant_bit = status & (1 << self.number)
        more_significant_bit = status & (1 << self.number + 1)

        if less_significant_bit and more_significant_bit:
            return self.possible_status[3]
        elif less_significant_bit:
            return self.possible_status[2]
        elif more_significant_bit:
            return self.possible_status[1]
        else:
            return self.possible_status[0]

    def update_view(self, view, status):
        view.setText(self.status(status))

    @property
    def mask(self):
        return (1 << self.number) | (1 << self.number+1)


class EmptyStatusMonitor:

    def __init__(self):
        self.bit_handlers = []
        self.views = {}
        self.set_registers()

    def set_registers(self):
        pass

    def set_view(self, mask, qt_view):
        self.views[mask.value] = qt_view

    def update_status(self, status):
        for handler in self.bit_handlers:
            self.update_view(handler, handler.mask, status)

    def update_view(self, handler, mask, status):
        view = self.views.get(mask)

        if view:
            handler.update_view(view, status)


class L6470Status(EmptyStatusMonitor):

    def __init__(self):
        EmptyStatusMonitor.__init__(self)

    def set_registers(self):
        self.bit_handlers.append(SimpleBit('SCK_MOD', 15, ActiveType.active_high, Level.info))
        self.bit_handlers.append(SimpleBit('STEP_LOSS_B', 14, ActiveType.active_low, Level.warning))
        self.bit_handlers.append(SimpleBit('STEP_LOSS_A', 13, ActiveType.active_low, Level.warning))
        self.bit_handlers.append(SimpleBit('OCD', 12, ActiveType.active_low, Level.warning))
        self.bit_handlers.append(SimpleBit('TH_SD', 11, ActiveType.active_low, Level.warning))
        self.bit_handlers.append(SimpleBit('TH_WRN', 10, ActiveType.active_low, Level.critical))
        self.bit_handlers.append(SimpleBit('UVLO', 9, ActiveType.active_low, Level.warning))
        self.bit_handlers.append(SimpleBit('WRONG_CMD', 8, ActiveType.active_high, Level.warning))
        self.bit_handlers.append(SimpleBit('NOTPERF_CMD', 7, ActiveType.active_high, Level.warning))
        self.bit_handlers.append(SimpleBit('DIR', 4, ActiveType.active_high, Level.info))
        self.bit_handlers.append(SimpleBit('SW_EVN', 3, ActiveType.active_high, Level.warning))
        self.bit_handlers.append(SimpleBit('SW_F', 2, ActiveType.active_high, Level.warning))
        self.bit_handlers.append(SimpleBit('BUSY', 1, ActiveType.active_low, Level.info))
        self.bit_handlers.append(SimpleBit('HiZ', 0, ActiveType.active_high, Level.info))
        
        self.bit_handlers.append(MotorStatus('MOT_STATUS', 5))
    

class L6480Status(EmptyStatusMonitor):

    def __init__(self):
        EmptyStatusMonitor.__init__(self)

    def set_registers(self):
        self.bit_handlers.append(SimpleBit('STEP_LOSS_B', 15, ActiveType.active_low, Level.warning))
        self.bit_handlers.append(SimpleBit('STEP_LOSS_A', 14, ActiveType.active_low, Level.warning))
        self.bit_handlers.append(SimpleBit('OCD', 13, ActiveType.active_low, Level.warning))
        self.bit_handlers.append(SimpleBit('UVLO_ADC', 10, ActiveType.active_low, Level.warning))
        self.bit_handlers.append(SimpleBit('UVLO', 9, ActiveType.active_low, Level.warning))
        self.bit_handlers.append(SimpleBit('STCK_MOD', 8, ActiveType.active_high, Level.warning))
        self.bit_handlers.append(SimpleBit('CMD_ERROR', 7, ActiveType.active_high, Level.warning))
        self.bit_handlers.append(SimpleBit('DIR', 4, ActiveType.active_high, Level.info))
        self.bit_handlers.append(SimpleBit('SW_EVN', 3, ActiveType.active_high, Level.warning))
        self.bit_handlers.append(SimpleBit('SW_F', 2, ActiveType.active_high, Level.warning))
        self.bit_handlers.append(SimpleBit('BUSY', 1, ActiveType.active_low, Level.info))
        self.bit_handlers.append(SimpleBit('HiZ', 0, ActiveType.active_high, Level.info))

        self.bit_handlers.append(MotorStatus('MOT_STATUS', 5))
