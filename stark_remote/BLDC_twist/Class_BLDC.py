import serial, time #, sys, struct
import serial.rs485


# ------------------------------------------------------------------
# 1) 프로토콜 상수 ― dict 로 정리
# ------------------------------------------------------------------
BAUDHEX_TO_BPS: dict[int, int] = {
    0x00: 110,       0x01: 300,       0x02: 600,       0x03: 1_200,
    0x04: 2_400,     0x05: 4_800,     0x06: 9_600,     0x07: 14_400,
    0x08: 19_200,    0x09: 28_800,    0x0A: 38_400,    0x0B: 57_600,
    0x0C: 76_800,    0x0D: 115_200,   0x0E: 230_400,   0x0F: 250_000,
    0x10: 500_000,   0x11: 1_000_000,
}

BAUDBPS_TO_HEX = {_bps: _hex for _hex, _bps in BAUDHEX_TO_BPS.items()}

CMD_MODE: dict[str, list] = {
    "POSITION_SPEED"       : [0x01, 0x07, [1, 1, 1, 2, 2]],
    "POSITION_TIME"        : [0x02, 0x06, [1, 1, 1, 2, 1]],
    "SPEED_TIME"           : [0x03, 0x06, [1, 1, 1, 2, 1]],
    "SET_POSITION_PID"     : [0x04, 0x06, [1, 1, 1, 1, 1, 1]],
    "SET_SPEED_PID"        : [0x05, 0x06, [1, 1, 1, 1, 1, 1]],
    "SET_ID"               : [0x06, 0x03, [1, 1, 1]],
    "SET_BAUDRATE"         : [0x07, 0x03, [1, 1, 1]],
    "SET_RESP_TIME"        : [0x08, 0x03, [1, 1, 1]],
    "SET_RATED_SPEED"      : [0x09, 0x04, [1, 1, 2]],
    "SET_RESOLUTION"       : [0x0A, 0x04, [1, 1, 2]],
    "SET_REDUCTION_RATIO"  : [0x0B, 0x04, [1, 1, 2]],
    "SET_CONTROL_ONOFF"    : [0x0C, 0x03, [1, 1, 1]],
    "SET_POSITION_MODE"    : [0x0D, 0x03, [1, 1, 1]],
    "SET_DIRECTION"        : [0x0E, 0x03, [1, 1, 1]],
    "POSITION_INIT"        : [0x0F, 0x02, [1, 1]],
    "FACTORY_RESET"        : [0x10, 0x02, [1, 1]],
}

FEEDBACK_MODE: dict[str, list] = {
    "ping"         : [0xA0, 0x02, [1, 1]],  
    "position"     : [0xA1, 0x08, [1, 1, 1, 2, 2, 1]],
    "speed"        : [0xA2, 0x08, [1, 1, 1, 2, 2, 1]],  
    "pos_pid"      : [0xA3, 0x06, [1, 1, 1, 1, 1, 1]],
    "spd_pid"      : [0xA4, 0x06, [1, 1, 1, 1, 1, 1]],  
    "resp_time"    : [0xA5, 0x03, [1, 1, 1]],
    "rated_rpm"    : [0xA6, 0x04, [1, 1, 2]],  
    "resolution"   : [0xA7, 0x04, [1, 1, 2]],
    "reduction"    : [0xA8, 0x04, [1, 1, 2]],  
    "ctrl_onoff"   : [0xA9, 0x03, [1, 1, 1]],
    "pos_mode"     : [0xAA, 0x03, [1, 1, 1]],  
    "direction"    : [0xAB, 0x03, [1, 1, 1]],
    "fw_version"   : [0xCD, 0x03, [1, 1, 1]],
}

FEEDBACK_NUM_TO_STR: dict[int, str] = {
    0xD0    :   "ping",  
    0xD1    :   "position",
    0xD2    :   "speed",  
    0xD3    :   "pos_pid",
    0xD4    :   "spd_pid",  
    0xD5    :   "resp_time",
    0xD6    :   "rated_rpm",  
    0xD7    :   "resolution",
    0xD8    :   "reduction",  
    0xD9    :   "ctrl_onoff",
    0xDA    :   "pos_mode",  
    0xDB    :   "direction",
    0xFD    :   "fw_version",
}


class BLDC_MotorControl:
    def __init__(self, port_dir: str = "/dev/ttyACM0", baudrate_bps: int = 9_600, timeout: float = 0.07, 
                 print_OutPut: bool = True, 
                 motor_id: int = 0x00, motor_cw_is_moving_forward: bool = True):
        self.port = port_dir
        self.baudrate = baudrate_bps
        self.serial_timeout = timeout
        self.motor_ID = motor_id
        self.printing_OutPut = print_OutPut
        self.connection_info = self.connect_serial()

        if self.printing_OutPut:
            for feedback_name, check_feedback_items in FEEDBACK_MODE.items():
                self.cprint(feedback_name)
                self.cprint(self.ask_feedback(check_feedback_items))
                self.cprint()
                time.sleep(0.01)
            self.cprint(f"==================== {self.port}, Motor ID: {self.motor_ID:>3} ====================\n\n")

        # 모터 드라이버 매뉴얼 상 초기값으로 설정
        self.response_time          = 0x01      # 100㎲
        self.rated_speed            = 0x0BB8    # 3000 RPM
        # self.resolution             = 0x0004    # 4 (pulse or pole)
        # self.reduction_ratio        = 0x000A    # 1:1
        self.control_is_off         = 0x00      # On
        self.ctrl_direction_is_cw   = 0x00      # CCW
        self.control_Rside          = motor_cw_is_moving_forward  # True면 모터는 우측륜, False면 좌측륜

        # 모터 드라이버와 모터에 맞게 파인튜닝한 값으로 설정
        self.reduction_ratio        = 280
        self.resolution             = 4

        self.speed_PID              = [50, 0, 0]

        # 모터 드라이버에 파인튜닝 값 적용
        p_ratio = self.set_reduction_ratio(ratio_tenths=self.reduction_ratio)
        p_resol = self.set_resolution(resolution=self.resolution)
        p_sPID  = self.set_spd_PID_ctrl(ctrl_spd_P=self.speed_PID[0], ctrl_spd_I=self.speed_PID[1], ctrl_spd_D=self.speed_PID[2])

        self.cprint(f"Reduction Ratio: {p_ratio}\nResolution: {p_resol}\nSpeed PID:{p_sPID}")
        
        # 모터 제어값을 초기값으로 설정
        self.present_POS        = 0x0000    # 0.01˚
        self.present_SPD        = 0x0000    # 0.1 RPM

        # 함수 반복 실행 시 탈출용 변수
        self.repeat_function = 0
        

    def connect_serial(self):
        try:
            self.con_ser = serial.rs485.RS485(
                port=self.port,
                baudrate=self.baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=self.serial_timeout
            )
            self.con_ser.rs485_mode = serial.rs485.RS485Settings(
            rts_level_for_tx=True, rts_level_for_rx=False,
            delay_before_tx=0.0,  delay_before_rx=0.01
            )
            return self.con_ser

        except serial.SerialException as e:
            print(f"Failed to make connection with {self.port}: {e}")
    

    # -------------------- 기본 명령 유틸리티 함수 --------------------
    @staticmethod
    def _checksum(motor_id: int, length_of_data: int, data_without_checksum: bytes):
        bytes_of_ID_and_Data_Len = bytes([motor_id, length_of_data])
        # self.cprint(bytes_of_ID_and_Data_Len, data_without_checksum)
        return (~sum(bytes_of_ID_and_Data_Len+data_without_checksum) & 0xFF)
    
    @staticmethod
    def _2BytesTO_two1Byte(_2bytes_data):
        if type(_2bytes_data) == bytes:
            _1st_byte = int.from_bytes(_2bytes_data[0:1], byteorder="big")
            _2nd_byte = int.from_bytes(_2bytes_data[1:2], byteorder="big")
        elif type(_2bytes_data) == int:
            _1st_byte = ((_2bytes_data >> 8) & 0xFF)
            _2nd_byte = (_2bytes_data & 0xFF)
        else:
            raise ValueError("_2bytes_data type Error")
        return _1st_byte, _2nd_byte
    
    @staticmethod
    def _make_TxData(ID: int, Len_of_Data: int, DATA: list[int, int]):
        Data_before_mainDATA = (0xFF, 0xFE, ID, Len_of_Data)
        return_DATA = Data_before_mainDATA + tuple(DATA)
        return bytes(return_DATA)
    
    @staticmethod
    def _read_RxData(raw_DATA: bytes, division_list: list):
        division_Len = len(division_list)
        cumulative_Div_Len = 2 # From CheckSum
        structured_DATA = []
        for index in range(division_Len):
            temp_DATA = int.from_bytes(raw_DATA[cumulative_Div_Len:cumulative_Div_Len+division_list[index]], byteorder="big")
            structured_DATA.append(temp_DATA)
            cumulative_Div_Len += division_list[index]
        # self.cprint(structured_DATA)
        if structured_DATA[1] in FEEDBACK_NUM_TO_STR:
            structured_DATA[1] = FEEDBACK_NUM_TO_STR[structured_DATA[1]]
        return structured_DATA
    
    def Transmit_data(self, Tx_data):
        # self.con_ser.reset_input_buffer()
        self.con_ser.write(Tx_data)
        # self.con_ser.flush()
        # time.sleep(0.01)

    def Receive_data(self, Rx_data_length: int, Rx_Data_division_list: list = None):
        check_count = 0
        while True:
            check_count += 1
            header_check = self.con_ser.read(2)
            if header_check == b'\xFF\xFE':
                break
            if len(header_check) == 0:
                if self.printing_OutPut:
                    print(f"Receiving Header Packet Error ({self.port}, {self.motor_ID}, {header_check})")  # 타임아웃
                if check_count >= 5:
                    return f"Receiving Header Packet Error 5 times. Latest data: {header_check}"
        
        raw_Rx_data = self.con_ser.read(size=(2+Rx_data_length)) # Header(2Bytes) has been already read. So read DATA with ID and Data Length.
        # self.cprint("???", raw_Rx_data)
        if Rx_Data_division_list:
            return self._read_RxData(raw_DATA=raw_Rx_data, division_list=Rx_Data_division_list)
        else:
            return raw_Rx_data
    
    def TxRx_data(self, Tx_data, Rx_data_length: int, Rx_Data_division_list: list):
        self.Transmit_data(Tx_data=Tx_data)
        return self.Receive_data(Rx_data_length=Rx_data_length, Rx_Data_division_list=Rx_Data_division_list)

    def cprint(self, _msg = None):
        if self.printing_OutPut:
            if _msg != None:
                print(_msg)
            else:
                print()
    

    # -------------------- 피드백 수신 명령 함수 --------------------
    def ask_feedback(self, feedback_mode_chosen: list):
        Length_of_Data = 0x02
        Mode = feedback_mode_chosen[0]

        Data_for_checksum = bytes([Mode])
        # self.cprint(Data_for_checksum)
        CheckSum = self._checksum(motor_id=self.motor_ID, length_of_data=Length_of_Data, data_without_checksum=Data_for_checksum)
        # self.cprint(CheckSum)
        main_Data = [CheckSum, Mode]

        making_Tx_data = self._make_TxData(ID=self.motor_ID, Len_of_Data=Length_of_Data, DATA=main_Data)
        # self.cprint(making_Tx_data)
        self.Transmit_data(Tx_data=making_Tx_data)
        return self.Receive_data(Rx_data_length=feedback_mode_chosen[1], Rx_Data_division_list=feedback_mode_chosen[2])

    def check_ping(self, ping_motor_ID: int):
        Length_of_Data = 0x02
        Mode = 0xA0

        Data_for_checksum = bytes([Mode])
        CheckSum = self._checksum(motor_id=ping_motor_ID, length_of_data=Length_of_Data, data_without_checksum=Data_for_checksum)
        main_Data = [CheckSum, Mode]

        making_Tx_data = self._make_TxData(ID=ping_motor_ID, Len_of_Data=Length_of_Data, DATA=main_Data)
        # self.cprint(making_Tx_data)
        self.Transmit_data(Tx_data=making_Tx_data)
        return self.Receive_data(Rx_data_length=0x02, Rx_Data_division_list=FEEDBACK_MODE["ping"][2])
    

    # -------------------- 제어 송신 명령 함수 --------------------(위치 제어 관련 함수 미작성)
    def set_baudrate(self, bps: int):
        """
        매뉴얼 상 07번 송신 기능

        통신 속도(baud rate) 설정
        """
        if bps not in BAUDBPS_TO_HEX:
            raise ValueError("지원하지 않는 BaudRate")
        baudrate_HEXvalue = BAUDBPS_TO_HEX[bps]

        Length_of_Data = CMD_MODE["SET_BAUDRATE"][1]
        Mode = CMD_MODE["SET_BAUDRATE"][0]

        Data_for_checksum = bytes([Mode, baudrate_HEXvalue])
        CheckSum = self._checksum(motor_id=self.motor_ID, length_of_data=Length_of_Data, data_without_checksum=Data_for_checksum)
        main_Data = [CheckSum, Mode, baudrate_HEXvalue]

        making_Tx_data = self._make_TxData(ID=self.motor_ID, Len_of_Data=Length_of_Data, DATA=main_Data)
        self.Transmit_data(Tx_data=making_Tx_data)

        temp_BackUp_baudrate = self.baudrate
        self.baudrate = bps
        self.connect_serial()
        check_ID_by_TxRx_Ping = self.check_ping()
        if check_ID_by_TxRx_Ping[1] == 208:
            return True
        else:
            self.repeat_function += 1
            if self.repeat_function >= 10:
                raise ValueError("Baudrate setting Error")
            else:
                self.cprint(f"Attepting set the Baudrate to Initial value.")
                self.set_baudrate(new_ID=temp_BackUp_baudrate)
                self.repeat_function = 0
    
    '''
    def set_motor_id(self, new_ID: int):
        """
        매뉴얼 상 06번 송신 기능

        드라이버 ID 설정
        """
        Length_of_Data = CMD_MODE["SET_ID"][1]
        Mode = CMD_MODE["SET_ID"][0]

        Data_for_checksum = bytes([Mode, new_ID])
        CheckSum = self._checksum(motor_id=self.motor_ID, length_of_data=Length_of_Data, data_without_checksum=Data_for_checksum)
        main_Data = [CheckSum, Mode, new_ID]

        making_Tx_data = self._make_TxData(ID=self.motor_ID, Len_of_Data=Length_of_Data, DATA=main_Data)
        self.Transmit_data(Tx_data=making_Tx_data)

        temp_BackUp_ID = self.motor_ID
        self.motor_ID = new_ID
        check_ID_by_TxRx_Ping = self.check_ping()
        if check_ID_by_TxRx_Ping[1] == 208:
            return True
        else:
            self.repeat_function += 1
            if self.repeat_function >= 10:
                raise ValueError("ID setting Error")
            else:
                self.cprint(f"Attepting set the ID to Initial value.")
                self.set_motor_id(new_ID=temp_BackUp_ID)
                self.repeat_function = 0
    '''

    def set_control_onoff(self, control_enable: bool):
        """
        매뉴얼 상 12번 송신 기능

        모터 제어 ON/OFF (모터 드라이브 활성/비활성)
        """
        if control_enable:
            ctrl_data = 0x00
        else:
            ctrl_data = 0x01

        Length_of_Data = CMD_MODE["SET_CONTROL_ONOFF"][1]
        Mode = CMD_MODE["SET_CONTROL_ONOFF"][0]

        Data_for_checksum = bytes([Mode, ctrl_data])
        CheckSum = self._checksum(motor_id=self.motor_ID, length_of_data=Length_of_Data, data_without_checksum=Data_for_checksum)
        main_Data = [CheckSum, Mode, ctrl_data]

        making_Tx_data = self._make_TxData(ID=self.motor_ID, Len_of_Data=Length_of_Data, DATA=main_Data)
        self.Transmit_data(Tx_data=making_Tx_data)
        return self.ask_feedback(feedback_mode_chosen=FEEDBACK_MODE["ctrl_onoff"])
    
    def set_spd_PID_ctrl(self, ctrl_spd_P: int, ctrl_spd_I: int, ctrl_spd_D: int, ctrl_spd_CURRENT_tenths: int = 80):
        """
        매뉴얼 상 05번 송신 기능

        모터 속도 PID 설정(속도 제어 정격 전류는 8A[암페어],)
        """
        Length_of_Data = CMD_MODE["SET_SPEED_PID"][1]
        Mode = CMD_MODE["SET_SPEED_PID"][0]

        Data_for_checksum = bytes([Mode, ctrl_spd_P, ctrl_spd_I, ctrl_spd_D, ctrl_spd_CURRENT_tenths])
        CheckSum = self._checksum(motor_id=self.motor_ID, length_of_data=Length_of_Data, data_without_checksum=Data_for_checksum)
        main_Data = [CheckSum, Mode, ctrl_spd_P, ctrl_spd_I, ctrl_spd_D, ctrl_spd_CURRENT_tenths]

        making_Tx_data = self._make_TxData(ID=self.motor_ID, Len_of_Data=Length_of_Data, DATA=main_Data)
        self.Transmit_data(Tx_data=making_Tx_data)
        return self.ask_feedback(feedback_mode_chosen=FEEDBACK_MODE["spd_pid"])
    
    def set_rated_speed_rpm(self, motor_rated_RPM_tenths: int):
        """
        매뉴얼 상 09번 송신 기능

        모터 정격속도 설정
        """
        temp_rated1, temp_rated2 = self._2BytesTO_two1Byte(motor_rated_RPM_tenths)

        Length_of_Data = CMD_MODE["SET_RATED_SPEED"][1]
        Mode = CMD_MODE["SET_RATED_SPEED"][0]

        Data_for_checksum = bytes([Mode, temp_rated1, temp_rated2])
        CheckSum = self._checksum(motor_id=self.motor_ID, length_of_data=Length_of_Data, data_without_checksum=Data_for_checksum)
        main_Data = [CheckSum, Mode, temp_rated1, temp_rated2]

        making_Tx_data = self._make_TxData(ID=self.motor_ID, Len_of_Data=Length_of_Data, DATA=main_Data)
        self.Transmit_data(Tx_data=making_Tx_data)
        return self.ask_feedback(feedback_mode_chosen=FEEDBACK_MODE["rated_rpm"])
    
    def set_reduction_ratio(self, ratio_tenths: int):
        """
        매뉴얼 상 11번 송신 기능
        
        감속비 설정 - 단위: 0.1 ratio [범위: 1~65533]

        예시)
        1 : 1  → ratio_tenths = 10   (0x000A)

        2.5 : 1  → ratio_tenths = 25   (0x0019)
        """
        FineTuning_param_of_Ratio_Value = 1 # 7*6*(55/60)
        Real_ratio_tenths = round(ratio_tenths * FineTuning_param_of_Ratio_Value)
        temp_ratio1, temp_ratio2 = self._2BytesTO_two1Byte(Real_ratio_tenths)

        Length_of_Data = CMD_MODE["SET_REDUCTION_RATIO"][1]
        Mode = CMD_MODE["SET_REDUCTION_RATIO"][0]

        Data_for_checksum = bytes([Mode, temp_ratio1, temp_ratio2])
        CheckSum = self._checksum(motor_id=self.motor_ID, length_of_data=Length_of_Data, data_without_checksum=Data_for_checksum)
        main_Data = [CheckSum, Mode, temp_ratio1, temp_ratio2]

        making_Tx_data = self._make_TxData(ID=self.motor_ID, Len_of_Data=Length_of_Data, DATA=main_Data)
        self.Transmit_data(Tx_data=making_Tx_data)
        return self.ask_feedback(feedback_mode_chosen=FEEDBACK_MODE["reduction"])
    
    def set_resolution(self, resolution: int):
        """
        매뉴얼 상 10번 송신 기능
        
        센서/엔코더 분해능 설정 - 단위: Pulse or Pole [범위: 1~65533(0xFFFD)]

        예시) 1000 → 0x03E8 로 변환됨
        """
        temp_res1, temp_res2 = self._2BytesTO_two1Byte(resolution)

        Length_of_Data = CMD_MODE["SET_RESOLUTION"][1]
        Mode = CMD_MODE["SET_RESOLUTION"][0]

        Data_for_checksum = bytes([Mode, temp_res1, temp_res2])
        CheckSum = self._checksum(motor_id=self.motor_ID, length_of_data=Length_of_Data, data_without_checksum=Data_for_checksum)
        main_Data = [CheckSum, Mode, temp_res1, temp_res2]

        making_Tx_data = self._make_TxData(ID=self.motor_ID, Len_of_Data=Length_of_Data, DATA=main_Data)
        self.Transmit_data(Tx_data=making_Tx_data)
        return self.ask_feedback(feedback_mode_chosen=FEEDBACK_MODE["resolution"])

    def set_speed_per_time(self, GoForward_true: bool, target_Speed_RPM_tenths: int, Accel_time_0p1s: int = 1):
        """
        매뉴얼 상 03번 송신 기능
        
        RPM 값을 target_Speed_rpm에 입력, 모터 가속 시간(0.1s 단위)을 Accel_time_0p1s에 입력
        """
        if GoForward_true:
            Clockwise_true = 0x01 if self.control_Rside else 0x00
        else:
            Clockwise_true = 0x00 if self.control_Rside else 0x01
        
        FineTuning_param_of_Speed_Value = 1 #15.3/60
        Real_Speed_tenths = round(target_Speed_RPM_tenths * FineTuning_param_of_Speed_Value)
        temp_spd1, temp_spd2 = self._2BytesTO_two1Byte(Real_Speed_tenths)

        Length_of_Data = CMD_MODE["SPEED_TIME"][1]
        Mode = CMD_MODE["SPEED_TIME"][0]

        Data_for_checksum = bytes([Mode, Clockwise_true, temp_spd1, temp_spd2, Accel_time_0p1s])
        # self.cprint(Data_for_checksum)
        CheckSum = self._checksum(motor_id=self.motor_ID, length_of_data=Length_of_Data, data_without_checksum=Data_for_checksum)
        # self.cprint(CheckSum)
        main_Data = [CheckSum, Mode, Clockwise_true, temp_spd1, temp_spd2, Accel_time_0p1s]

        making_Tx_data = self._make_TxData(ID=self.motor_ID, Len_of_Data=Length_of_Data, DATA=main_Data)
        # self.cprint(making_Tx_data)
        self.Transmit_data(Tx_data=making_Tx_data)
        return self.ask_feedback(feedback_mode_chosen=FEEDBACK_MODE["speed"])
    
    def Brake(self):
        """
        바퀴 회전 정지 / 속도 0 RPM(전진 상태 유지) 입력
        """
        return self.set_speed_per_time(GoForward_true=True, target_Speed_rpm=0x0000, Accel_time_0p1s=1)
    
    def factory_reset(self, target_motor_ID_in_Driver: int):
        """
        매뉴얼 상 16번 송신 기능
        
        공장 초기화
        """
        Length_of_Data = CMD_MODE["FACTORY_RESET"][1]
        Mode = CMD_MODE["FACTORY_RESET"][0]

        Data_for_checksum = bytes([Mode])
        CheckSum = self._checksum(motor_id=target_motor_ID_in_Driver, length_of_data=Length_of_Data, data_without_checksum=Data_for_checksum)
        main_Data = [CheckSum, Mode]

        making_Tx_data = self._make_TxData(ID=target_motor_ID_in_Driver, Len_of_Data=Length_of_Data, DATA=main_Data)
        self.Transmit_data(Tx_data=making_Tx_data)
        self.cprint(f"Factory Reset. {self.port}")
    

    # -------------------- 제어 명령 함수 응용 --------------------
    def Turn_OFF(self):
        self.Brake()
        self.set_control_onoff(control_enable=False)
    
    def Turn_ON(self):
        self.set_control_onoff(control_enable=True)


    # -------------------- GPT가 만든 함수 --------------------
    def GPT_scan_ids(self, start=0x00, end=0x1F, pause=0.01):
        found = []
        old = self.motor_ID
        for i in range(start, end+1):
            self.motor_ID = i
            rep = self.check_ping()          # 당신 코드의 ping (A0h 요청)
            # 파서가 [mode, ...] 형태면 mode==0xD0 또는 문자열 "ping" 확인
            ok = (isinstance(rep, list) and ((rep[1] == 'ping') or (rep[0] == 0xD0)))
            if ok:
                found.append(i)
            time.sleep(pause)
        self.motor_ID = old
        return found
    
    def GPT_set_motor_ID(self, new_ID: int, verify_range=(0x00, 0xEF)):
        # 1) 브로드캐스트로 SET_ID 송신
        Length = CMD_MODE["SET_ID"][1]     # 0x03
        Mode   = CMD_MODE["SET_ID"][0]     # 0x06
        data   = bytes([Mode, new_ID])
        csum   = self._checksum(0xFF, Length, data)
        tx     = self._make_TxData(ID=0xFF, Len_of_Data=Length, DATA=[csum, Mode, new_ID])
        self.con_ser.reset_input_buffer()
        self.Transmit_data(tx)
        time.sleep(0.05)   # 슬레이브 적용 여유

        # 2) 스캔으로 새 ID 확인
        found = self.scan_ids(start=verify_range[0], end=verify_range[1])
        if new_ID in found:
            self.motor_ID = new_ID
            return True

        # 3) 못 찾으면 한 번 더 시도(또는 사용자에게 전원 재기동 후 재스캔 권고)
        time.sleep(0.2)
        found = self.scan_ids(start=verify_range[0], end=verify_range[1])
        if new_ID in found:
            self.motor_ID = new_ID
            return True
        raise ValueError("ID setting Error: device not found at new_ID")
    
    # -------------------- 수작업 송신 함수 --------------------
    def send_setting(self, All_DATA_withoutCheckSum_by_Each1Byte: list[int, int]):
        Header_1    = All_DATA_withoutCheckSum_by_Each1Byte[0]
        Header_2    = All_DATA_withoutCheckSum_by_Each1Byte[1]
        _motor_ID   = All_DATA_withoutCheckSum_by_Each1Byte[2]
        _Data_Size  = All_DATA_withoutCheckSum_by_Each1Byte[3]
        # _Check_Sum  = 0x00
        _Mode       = All_DATA_withoutCheckSum_by_Each1Byte[4]
        _left_data  = All_DATA_withoutCheckSum_by_Each1Byte[5:]

        Data_for_checksum = bytes([Header_1, Header_2, _motor_ID, _Data_Size, _Mode, _left_data])
        _Check_Sum = self._checksum(motor_id=_motor_ID, length_of_data=_Data_Size, data_without_checksum=Data_for_checksum)
        main_Data = [_Check_Sum, _Mode, _left_data]

        making_Tx_data = self._make_TxData(ID=_motor_ID, Len_of_Data=_Data_Size, DATA=main_Data)
        self.TxRx_data(Tx_data=making_Tx_data, Rx_data_length=_Data_Size)

