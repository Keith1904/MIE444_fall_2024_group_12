import socket
import time
from datetime import datetime
import serial



class RadioOperator:
    '''This class communicates with the Bluetooth module.'''
    
    def __init__(
        self,
        HOST,
        PORT_TX,
        PORT_RX,
        BAUDRATE,
        PORT_SERIAL,
        TIMEOUT_SERIAL,
        FRAMESTART,
        FRAMEEND,
        CMD_DELIMITER,
        SIMULATE,
        TRANSMIT_PAUSE):
        
        self.HOST = HOST
        self.PORT_TX = PORT_TX
        self.PORT_RX = PORT_RX
        self.BAUDRATE = BAUDRATE
        self.PORT_SERIAL = PORT_SERIAL
        self.TIMEOUT_SERIAL = TIMEOUT_SERIAL
        self.FRAMESTART = FRAMESTART
        self.FRAMEEND = FRAMEEND
        self.CMD_DELIMITER = CMD_DELIMITER
        self.SIMULATE = SIMULATE
        self.TRANSMIT_PAUSE = TRANSMIT_PAUSE
        self.line_busy = False
        try:
            self.SER = serial.Serial(self.PORT_SERIAL, self.BAUDRATE, timeout=self.TIMEOUT_SERIAL)
        except serial.SerialException:
            print(f'Serial connection was refused.\nEnsure {PORT_SERIAL} is the correct port and nothing else is connected to it.')
            
    def broadcast(self, data):
        while self.line_busy:
            time.sleep(0.1)
        self.line_busy = True
        packet_tx = self.packetize(data)
        self.transmit(packet_tx)
        response =  self.receive()
        self.line_busy = False
        return response
 
    # Wrapper functions
    def transmit(self, data):
        '''Selects whether to use serial or tcp for transmitting.'''
        if self.SIMULATE:
            self.transmit_tcp(data)
        else:
            self.transmit_serial(data)
        time.sleep(self.TRANSMIT_PAUSE)

    def receive(self):
        '''Selects whether to use serial or tcp for receiving.'''
        if self.SIMULATE:
            return self.receive_tcp()
        else:
            return self.receive_serial()

    # TCP communication functions
    def transmit_tcp(self, data):
        '''Send a command over the TCP connection.'''
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                s.connect((self.HOST, self.PORT_TX))
                s.send(data.encode('ascii'))
            except (ConnectionRefusedError, ConnectionResetError):
                print('Tx Connection was refused or reset.')
            except TimeoutError:
                print('Tx socket timed out.')
            except EOFError:
                print('\nKeyboardInterrupt triggered. Closing...')

    def receive_tcp(self):
        '''Receive a reply over the TCP connection.'''
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s2:
            try:
                s2.connect((self.HOST, self.PORT_RX))
                response_raw = s2.recv(1024).decode('ascii')
                if response_raw:
                    # return the data received as well as the current time
                    return [self.depacketize(response_raw), datetime.now().strftime("%H:%M:%S")]
                else:
                    return [[False], datetime.now().strftime("%H:%M:%S")]
            except (ConnectionRefusedError, ConnectionResetError):
                print('Rx connection was refused or reset.')
            except TimeoutError:
                print('Response not received from robot.')

    # Serial communication functions
    def transmit_serial(self, data):
        '''Transmit a command over a serial connection.'''
        self.clear_serial()
        self.SER.write(data.encode('ascii'))

    def receive_serial(self):
        '''Receive a reply over a serial connection.'''

        start_time = time.time()
        response_raw = ''
        while time.time() < start_time + self.TIMEOUT_SERIAL:
            if self.SER.in_waiting:
                response_char = self.SER.read().decode('ascii')
                if response_char == self.FRAMEEND:
                    response_raw += response_char
                    break
                else:
                    response_raw += response_char

        print(f'Raw response was: {response_raw}')

        # If response received, return it
        if response_raw:
            return [self.depacketize(response_raw), datetime.now().strftime("%H:%M:%S")]
        else:
            return [[False], datetime.now().strftime("%H:%M:%S")]

    def clear_serial(self, delay_time: float = 0):
        '''Wait some time (delay_time) and then clear the serial buffer.'''
        if self.SER.in_waiting:
            time.sleep(delay_time)
            print(f'Clearing Serial... Dumped: {self.SER.read(self.SER.in_waiting)}')

    # Packetization and validation functions
    def depacketize(self, data_raw: str):
        '''
        Take a raw string received and verify that it's a complete packet, returning just the data messages in a list.
        '''

        # Locate start and end framing characters
        start = data_raw.find(self.FRAMESTART)
        end = data_raw.find(self.FRAMEEND)

        # Check that the start and end framing characters are present, then return commands as a list
        if (start >= 0 and end >= start):
            data = data_raw[start+1:end].replace(f'{self.FRAMEEND}{self.FRAMESTART}', ',').split(',')
            cmd_list = [item.split(':', 1) for item in data]

            # Make sure this list is formatted in the expected manner
            for cmd_single in cmd_list:
                match len(cmd_single):
                    case 0:
                        cmd_single.append('')
                        cmd_single.append('')
                    case 1:
                        cmd_single.append('')
                    case 2:
                        pass
                    case _:
                        cmd_single = cmd_single[0:2]

            return cmd_list
        else:
            return [[False, '']]

    def packetize(self, data: str):
        '''
        Take a message that is to be sent to the command script and packetize it with start and end framing.
        '''

        # Check to make sure that a packet doesn't include any forbidden characters (0x01, 0x02, 0x03, 0x04)
        forbidden = [self.FRAMESTART, self.FRAMEEND, '\n']
        check_fail = any(char in data for char in forbidden)

        if not check_fail:
            return self.FRAMESTART + data + self.FRAMEEND

        return False

    def response_string(self, cmds: str, responses_list: list):
        '''
        Build a string that shows the responses to the transmitted commands that can be displayed easily.
        '''
        # Validate that the command ids of the responses match those that were sent
        cmd_list = [item.split(':')[0] for item in cmds.split(',')]
        valid = self.validate_responses(cmd_list, responses_list)

        # Build the response string
        out_string = ''
        sgn = ''
        chk = ''
        for item in zip(cmd_list, responses_list, valid):
            if item[2]:
                sgn = '='
                chk = '✓'
            else:
                sgn = '!='
                chk = 'X'

            out_string = out_string + (f'cmd {item[0]} {sgn} {item[1][0]} {chk}, response "{item[1][1]}"\n')

        return out_string

    def validate_responses(self, cmd_list: list, responses_list: list):
        '''
        Validate that the list of commands and received responses have the same command id's. Takes a
        list of commands and list of responses as inputs, and returns a list of true and false values
        indicating whether each id matches.
        '''
        valid = []
        for pair in zip(cmd_list, responses_list):
            if pair[1]:
                if pair[0] == pair[1][0]:
                    valid.append(True)
                else:
                    valid.append(False)
        return valid