from serial import Serial


class GCodeSerial:
    def __init__(self, port, baudrate):
        self.serial = Serial(port, baudrate)

        if self.serial.is_open:
            self.serial.close()
        self.serial.open()

    def send_command(self, cmd):
        cmd.strip('\n')
        cmd+='\n'
        self.serial.write(cmd.encode())

    def send_commands(self, cmd_list):
        for cmd in cmd_list:
            self.send_command(cmd)

    def __del__(self):
        self.serial.close()