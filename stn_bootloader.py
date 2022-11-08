from serial import Serial
from crccheck.crc import CrcXmodem
import struct
import time
import math
from tqdm import tqdm
from enum import Enum, IntEnum, auto
import binascii

class ReceiverState(Enum):
    STX1 = auto()
    STX2 = auto()
    COMMAND = auto()
    LENGTH = auto()
    DATA = auto()
    CHECKSUM1 = auto()
    CHECKSUM2 = auto()
    ETX = auto()
    COMPLETE = auto()
    INCOMPLETE = auto()

class FrameValue(IntEnum):
    STX = 0x55
    ETX = 0x04
    DLE = 0x05

class ErrorValue(IntEnum):
    CRC_MISMATCH = 0x01
    PACKET_TOO_LONG = 0x02
    UNKNOWN_COMMAND = 0x10
    INVALID_FORMAT = 0x11
    SEQUENCE = 0x30
    AUTHENTICATION = 0x50
    PROGRAMMING = 0x80
    VERIFICATION = 0x90
    FIRMWARE_VERSION = 0xA4

class FrameReceiver:
    def __init__(self):
        self.curr_state = ReceiverState.STX1
        self.raw_buffer = bytearray()
        self.buffer = []
        self.status = 0x00
        self.command = 0x00
        self.error = 0x00
        self.length = 0x00
        self.checksum = 0x00
        self.crc = CrcXmodem()
        self.dle = False

    def __repr__(self):
        print()
        print(f"curr_state = {self.curr_state}")
        print(f"raw_buffer = {binascii.hexlify(self.raw_buffer).upper()}")
        print(f"status = 0b{self.status:02b}")
        print(f"command = 0x{self.command:02X}")
        print(f"error = {self.error.name}")
        print(f"length = {self.length}")
        print(f"checksum = 0x{self.checksum:04X}")
        print(f"crc = {self.crc.check_result}")
        print(f"dle = {self.dle}")

    def __str__(self):
        self.__repr__()

    def consume(self, byte):
        self.raw_buffer.append(byte[0])
        byte = int.from_bytes(byte, "big")

        def byte_unstuff(data):
            if data == FrameValue.DLE and not self.dle:
                self.dle = True
                return None
            else:
                self.dle = False
                return data

        match self.curr_state:
            case ReceiverState.STX1:
                if byte == FrameValue.STX:
                    self.curr_state = ReceiverState.STX2
                else:
                    self.curr_state = ReceiverState.INCOMPLETE

            case ReceiverState.STX2:
                if byte == FrameValue.STX:
                    self.curr_state = ReceiverState.COMMAND
                else:
                    self.curr_state = ReceiverState.INCOMPLETE

            case ReceiverState.COMMAND:
                byte = byte_unstuff(byte)
                if not byte == None:
                    self.crc.process([byte])
                    self.status = (byte & 0xC0) >> 6
                    self.command = byte & 0x3F
                    self.curr_state = ReceiverState.LENGTH
                else:
                    self.curr_state = ReceiverState.INCOMPLETE

            case ReceiverState.LENGTH:
                byte = byte_unstuff(byte)
                if not byte == None:
                    self.crc.process([byte])
                    self.length = byte
                    if self.length > 0:
                        self.curr_state = ReceiverState.DATA
                    else:
                        self.curr_state = ReceiverState.CHECKSUM1
                else:
                    self.curr_state = ReceiverState.INCOMPLETE

            case ReceiverState.DATA:
                byte = byte_unstuff(byte)
                if not byte == None:
                    self.crc.process([byte])
                    self.buffer.append(byte)
                    if len(self.buffer) >= self.length:
                        self.curr_state = ReceiverState.CHECKSUM1
                else:
                    self.curr_state = ReceiverState.INCOMPLETE

            case ReceiverState.CHECKSUM1:
                byte = byte_unstuff(byte)
                if not byte == None:
                    self.checksum = byte << 8
                    self.curr_state = ReceiverState.CHECKSUM2
                else:
                    self.curr_state = ReceiverState.INCOMPLETE

            case ReceiverState.CHECKSUM2:
                byte = byte_unstuff(byte)
                if not byte == None:
                    self.checksum |= byte
                    self.curr_state = ReceiverState.ETX
                else:
                    self.curr_state = ReceiverState.INCOMPLETE

            case ReceiverState.ETX:
                if byte == FrameValue.ETX:
                    if self.is_nack():
                        self.error = ErrorValue(struct.unpack("B", bytearray(self.buffer))[0])
                    self.curr_state = ReceiverState.COMPLETE
                else:
                    self.curr_state = ReceiverState.INCOMPLETE

            case ReceiverState.COMPLETE:
                pass

            case ReceiverState.INCOMPLETE:
                pass

    def is_done(self):
        return (self.curr_state == ReceiverState.COMPLETE) or (self.curr_state == ReceiverState.INCOMPLETE)

    def is_ack(self):
        return self.status == 1

    def is_nack(self):
        return self.status == 2

    def is_valid(self, expected_cmd):
        return self.is_done() and self.crc.final() == self.checksum and self.status == 1 and self.command == expected_cmd

class StnUpdater:
    def __init__(self, port):
        self.port = port

    def send_frame(self, data):
        packet = bytearray()

        packet += bytes([FrameValue.STX, FrameValue.STX])

        for byte in data:
            if byte in [0x55, 0x04, 0x05]:
                packet += bytes([0x05])
            packet.append(byte)

        packet += bytes([0x04])

        self.port.write(packet)

    def recv_frame(self, timeout):
        curr_time = time.time()
        next_time = curr_time + timeout

        receiver = FrameReceiver()

        while curr_time < next_time and not receiver.is_done():
            self.port.timeout = next_time - curr_time
            data = self.port.read()

            if data:
                next_time = curr_time + timeout
                receiver.consume(data)

            curr_time = time.time()

        return receiver


    def send_command(self, cmd, data):
        packet = bytearray()

        packet += struct.pack("B", cmd)

        packet += struct.pack(">H", len(data or []))
        
        if data:
            packet.extend(data)

        crc = CrcXmodem()
        crc.process(bytearray(packet))
        packet += struct.pack(">H", crc.final())

        self.send_frame(packet)

    def send_raw(self, data):
        self.port.write(str.encode(data))

    def recv_response(self, timeout=0.2, resend_retry=3):
        response = self.recv_frame(timeout)

        if not response.is_done():
            for _ in range(resend_retry):
                self.send_command(0x01, None)
                response = self.recv_frame(0.2)
                if response.is_done():
                    break

        return response

    def transmit(self, cmd, data, timeout=0.2):
        for _ in range(5):
            self.send_command(cmd, data)
            response = self.recv_response(timeout)

            if response.is_valid(cmd):
                break

        return response

    def connect(self):
        self.send_command(0x03, None)
        response = self.recv_response(resend_retry=0)

        if not response.is_done():
            self.send_raw("?\r")

            self.wait_for_prompt()

            self.send_raw("ATZ\r")
            time.sleep(0.05)

            self.port.flush()

            for _ in range(0, 5):
                self.send_command(0x03, None)
                response = self.recv_response(timeout=0.05, resend_retry=0)
                if (response.is_valid(0x03)):
                    return True

        return response.is_valid(0x03) 

    def device_id(self):
        response = self.transmit(0x07, None)

        if not response.is_valid(0x07):
            return None

        return struct.unpack(">H", bytearray(response.buffer))[0]

    def version(self):
        response = self.transmit(0x06, None)

        if not response.is_valid(0x06):
            return None

        return struct.unpack("BB", bytearray(response.buffer))

    def start_upload(self, image_size):
        response = self.transmit(0x30, [((image_size >> 16) & 0xFF), ((image_size >> 8) & 0xFF), (image_size & 0xFF),  0x01])

        if not response.is_valid(0x30):
            return None

        return struct.unpack(">H", bytearray(response.buffer))[0]

    def status(self):
        response = self.transmit(0x0F, None)

        if not response.is_valid(0x0F):
            return None

        return struct.unpack("B", bytearray(response.buffer))[0]

    def send_chunk(self, chunk_num, data):
        zz = [(chunk_num >> 8) & 0xFF, chunk_num & 0xFF]
        zz.extend(data)

        for i in range(5):
            response = self.transmit(0x31, zz, timeout=1)

            if response.is_valid(0x31):
                break

            elif response.is_nack() and response.error == ErrorValue.CRC_MISMATCH:
                pass

            else:
                raise Exception(response)

        if not response.is_valid(0x31):
                raise Exception(response)

        return struct.unpack(">H", bytearray(response.buffer))[0]

    def reset(self):
        self.send_command(0x02, None)
        self.recv_response()

    def wait_for_prompt(self, timeout=1):
        self.port.timeout = timeout
        while True:
            byte = self.port.read()
            if byte == b"":
                return False
            if byte == b">":
                return True

class FirmwareImageType(IntEnum):
    Normal = 0x00
    TolerateErrors = 0x01
    Validation = 0x10

class FirmwareImageDescriptor:
    def __init__(self, data):
        self.image_type, _, self.next_idx, self.error_idx, self.image_offset, self.image_size = struct.unpack(">BBBBLL", data)

class FirmwareImage:
    def __init__(self, firmware):
        self.firmware = firmware

        magic, version, dev_id_len = struct.unpack("6s2sb", firmware[:9])
        if not magic == b"STNFWv" or not version == b"05":
            raise Exception("MAGIC OR VERSION")

        self.dev_ids = struct.unpack(">{}H".format(dev_id_len), firmware[9:9 + (dev_id_len * 2)])
        
        desc_count = firmware[11]

        self.descriptors = []

        if desc_count > 0:
            for i in range(desc_count):
                start = 12 + (i * 12)
                data = firmware[start:start+12]
                self.descriptors.append(FirmwareImageDescriptor(data))
        else:
            image_type = 0x00
            next_idx = 0xFF
            error_idx = 0x00
            image_offset = 12
            image_size = len(firmware) - 12
            self.descriptors.append(FirmwareImageDescriptor(struct.pack(">BBBBLL", image_type, 0, next_idx, error_idx, image_offset, image_size)))

def batch(list, size):
    for i in range(0, len(list), size):
        yield list[i: i + size]

def upload_firmware(config):
    updater = StnUpdater(Serial(config["port"], baudrate=config["firmware_baudrate"], rtscts=config["flowcontrol"]))

    if not updater.connect():
        raise Exception("CONNECT")

    updater.port.baudrate = config["updater_baudrate"]

    with open(config["firmware_path"], "rb") as binary:
        firmware = bytearray(binary.read())

    firmware_image = FirmwareImage(firmware)

    if not updater.device_id() in firmware_image.dev_ids:
        raise Exception("DEVICE ID")

    image_idx = 0
    failed = False

    while True:
        descriptor = firmware_image.descriptors[image_idx]
        firmware_data = firmware_image.firmware[descriptor.image_offset : descriptor.image_offset + descriptor.image_size]

        if not updater.start_upload(len(firmware_data)):
            raise Exception("START UPLOAD")

        failed = False

        chunks = enumerate(batch(firmware_data, config["chunk_size"]))
        for idx, chunk in tqdm(chunks, total=math.ceil(len(firmware_data)/config["chunk_size"]), unit="chunk"):
            chunk = list(chunk)
            written = updater.send_chunk(idx, chunk)

            if written != idx:
                for _ in range(3):
                    written = updater.send_chunk(idx, chunk)
                    if written == idx:
                        break
                if written != idx:
                    failed = True
                    break

        if descriptor.next_idx != 0xFF:
            match descriptor.image_type:
                case FirmwareImageType.Normal.value:
                    image_idx = descriptor.next_idx
                case FirmwareImageType.TolerateErrors.value:
                    pass
                case FirmwareImageType.Validation.value:
                    if failed:
                        image_idx = descriptor.error_idx
                    else:
                        image_idx = descriptor.next_idx
                    failed = False
        else:
            break

        if failed:
            break
            
    if updater.status() != 1 or failed:
        print("Upload Failed")
    else:
        updater.reset()
        print("Upload Successful")

    updater.port.close()

config = {
    "firmware_path": "C:\\path\\to\\firmware.bin",
    "port": "COM1",
    "firmware_baudrate": 9600,
    "updater_baudrate": 1000000,
    "flowcontrol": False,
    "chunk_size": 1024
}

upload_firmware(config)