from serial import Serial
from crccheck.crc import CrcXmodem
import struct
import time
import math
from tqdm import tqdm
from enum import Enum, IntEnum, auto

class ReceiverState(Enum):
    STX1 = auto()
    STX2 = auto()
    COMMAND = auto()
    LENGTH = auto()
    DATA = auto()
    CHECKSUM1 = auto()
    CHECKSUM2 = auto()
    ETX = auto()
    SUCCESS = auto()
    FAILED = auto()

class FrameValue(IntEnum):
    STX = 0x55
    ETX = 0x04
    DLE = 0x05

class FrameReceiver:
    def __init__(self):
        self.curr_state = ReceiverState.STX1
        self.raw_buffer = bytearray()
        self.buffer = []
        self.ack = 0x00
        self.command = 0x00
        self.error = 0x00
        self.length = 0x00
        self.checksum = 0x00
        self.crc = CrcXmodem()
        self.dle = False

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

            case ReceiverState.STX2:
                if byte == FrameValue.STX:
                    self.curr_state = ReceiverState.COMMAND
                else:
                    self.curr_state = ReceiverState.COMMAND

            case ReceiverState.COMMAND:
                byte = byte_unstuff(byte)
                if not byte == None:
                    self.crc.process([byte])
                    self.ack = (byte & 0xC0) >> 6
                    self.command = byte & 0x3F
                    self.curr_state = ReceiverState.LENGTH

            case ReceiverState.LENGTH:
                byte = byte_unstuff(byte)
                if not byte == None:
                    self.crc.process([byte])
                    self.length = byte
                    if self.length > 0:
                        self.curr_state = ReceiverState.DATA
                    else:
                        self.curr_state = ReceiverState.CHECKSUM1

            case ReceiverState.DATA:
                byte = byte_unstuff(byte)
                if not byte == None:
                    self.crc.process([byte])
                    self.buffer.append(byte)
                    if len(self.buffer) >= self.length:
                        self.curr_state = ReceiverState.CHECKSUM1

            case ReceiverState.CHECKSUM1:
                byte = byte_unstuff(byte)
                if not byte == None:
                    self.checksum = byte << 8
                    self.curr_state = ReceiverState.CHECKSUM2

            case ReceiverState.CHECKSUM2:
                byte = byte_unstuff(byte)
                if not byte == None:
                    self.checksum |= byte
                    self.curr_state = ReceiverState.ETX

            case ReceiverState.ETX:
                if byte == FrameValue.ETX:
                    if self.is_nack():
                        self.error = struct.unpack("B", bytearray(self.buffer))[0]
                    self.curr_state = ReceiverState.SUCCESS
                else:
                    self.curr_state = ReceiverState.FAILED

            case ReceiverState.SUCCESS:
                pass

            case ReceiverState.FAILED:
                pass

    def is_done(self):
        return (self.curr_state == ReceiverState.SUCCESS) or (self.curr_state == ReceiverState.FAILED)

    def is_ack(self):
        return self.ack == 1

    def is_nack(self):
        return self.ack == 2

    def is_valid(self, expected_cmd):
        return self.is_done() and self.crc.final() == self.checksum and self.ack == 1 and self.command == expected_cmd

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

            elif response.is_nack() and response.buffer[0] == 0x01:
                pass

            else:
                print("Failed chunk")
                attrs = vars(response)
                raise Exception(', '.join("%s: %s" % item for item in attrs.items()))



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

def batch(list, size):
    for i in range(0, len(list), size):
        yield list[i: i + size]

def upload_firmware(config):
    updater = StnUpdater(Serial(config["port"], baudrate=config["firmware_baudrate"]))

    if not updater.connect():
        raise Exception("CONNECT")

    updater.port.baudrate = config["updater_baudrate"]

    with open(config["firmware_path"], "rb") as binary:
        firmware = bytearray(binary.read())
        
    magic, version, dev_id_len = struct.unpack("6s2sb", firmware[:9])
    if not magic == b"STNFWv" or not version == b"05":
        raise Exception("MAGIC OR VERSION")

    dev_ids = struct.unpack(">{}H".format(dev_id_len), firmware[9:9 + (dev_id_len * 2)])

    if dev_ids[0] != updater.device_id():
        raise Exception("DEVICE ID")

    image = firmware[12:]
    if not updater.start_upload(len(image)):
        raise Exception("START UPLOAD")

    chunks = enumerate(batch(image, config["chunk_size"]))
    for idx, chunk in tqdm(chunks, total=math.ceil(len(image)/config["chunk_size"]), unit="chunk"):
        chunk = list(chunk)
        written = updater.send_chunk(idx, chunk)

        if written != idx:
            for _ in range(3):
                written = updater.send_chunk(idx, chunk)
                if written == idx:
                    break
            if written != idx:
                raise Exception("CHUNK FAILURE")
    
    if updater.status() == 1:
        updater.reset()
        print("Upload Successful")
    else:
        print("Upload Failed")

    updater.port.close()

config = {
    "firmware_path": "C:\\path\\to\\firmware.bin",
    "port": "COM1",
    "firmware_baudrate": 9600,
    "updater_baudrate": 1000000,
    "chunk_size": 1024
}

upload_firmware(config)