from os import stat
from crc16_python import crc16_str_swap
import logging
from utils import toHex ,Hexcon


class COMMAND:
    Max_Min_Temp = '14'
    ACQUIRE_DEVICE_INF = '01'
    ACQUIRE_GIMBAL_MOUTION='19'
    AUTO_FOCUS = '04'
    CENTER = '08'
    MANUAL_ZOOM = '05'
    PHOTO_VIDEO_HDR = '0c'
    ACQUIRE_GIMBAL_ATT = '0d'
    RANGE_FİNDER = '15'
    BOX_TEMP = '13'
    POINT_TEMP = '12'
    INF_COLOR_MAP = '1A'
    COLOR_MAP = '1B'
    IMAGE_MOD = '10'
    IMAGE_MOD_CHANGE = '11'
    TargetAngle = '0E'    
    Thermal_Raw_data= '34'
    Thermal_Map= '35'    
    Range_finder_params_get = '31'
    Range_finder_params_send = '32'    
    Thermal_Gain_Send = '38'
    Thermal_Gain_Get= '37'    
    Thermal_Params_Send = '3C'
    Thermal_Params_Get = '3B'
    
    
    GIMBAL_ROT = '07'



class DeviceMsg:
    seq=0
    code_board_ver=''
    gimbal_firmware_ver=''
    zoom_firmware_ver=''
    id=''

class AutoFocusMsg:
    seq=0
    success=False

class CenterMsg:
    seq=0
    success=False

class MoutionMode:
    seq=0
    gimbal_moution_mode=''

class ManualZoomMsg:
    seq=0
    level=-1

class GimbalSpeedMsg:
    seq=0
    success=False

class AttitdueMsg:
    seq=    0
    stamp=  0 # seconds
    yaw=    ''
    pitch=  ''
    roll=   ''
    yaw_speed=  0.0 # deg/s
    pitch_speed=0.0
    roll_speed= 0.0

class TarRotationmsg:
    seq =0
    yaw = ''
    pitch = ''
    roll = ''

class TemperatureMsg:
    seq = 0
    temp_max = ''
    temp_min = ''
    temp_max_x = ''
    temp_max_y = ''
    temp_min_x = ''
    temp_min_y = ''

class RangeFinderMsg:
    seq =0
    Range_value = ''

class BoxTemperatureMsg:
    seq = 0
    startx = ''
    starty = ''
    endx = ''
    endy =''
    temp_max = ''
    temp_min = ''
    temp_max_x = ''
    temp_max_y = ''
    temp_min_x = ''
    temp_min_y = ''

class PointTemperatureMsg:
    seq = 0
    temp_point_x = ''
    temp_point_y = ''
    temprature = ''
    
class ColorMapMSg:
    seq = 0
    pseudo_color = ''
    target_pseudo_color = ''

class ImageModMsg:
    seq = 0
    vdisp_mode =''
    target_vdisp_mode= ''  
    
class ThermalRawData:
    seq = 0
    mode =''   
       
class ThermalTempMAP:
    seq = 0
    ack =''
   
class ThermalGain:
    seq = 0
    gain_status ='' 
       
class ThermalParams:
    seq = 0
    Distance = ''
    Target_emission_rate = ''
    Humidity = ''
    Atmospheric_Temperature = ''
    Reflection_Temperature = ''
    ack = ''
    
class RangeFinderParams:
    seq = 0
    laser_state =''
    ack = ''
    
#############################################
class SIYIMESSAGE:
    
    """
    Structure of SIYI camera messages
    """
    def __init__(self, debug=False) -> None:
        self._debug= debug # print debug messages
        if self._debug:
            d_level = logging.DEBUG
        else:
            d_level = logging.INFO
        LOG_FORMAT='[%(levelname)s] %(asctime)s [SIYIMessage::%(funcName)s] :\t%(message)s'
        logging.basicConfig(format=LOG_FORMAT, level=d_level)
        self._logger = logging.getLogger(self.__class__.__name__)

        self.HEADER='5566'# STX, 2 bytes
        self._ctr ='01'        

        self._seq= 0

        self._cmd_id='00' # 1 byte
        
        self._data_len = 0
        
        # String of data byes (in hex)
        self._data=''

        self._crc16='0000' # low byte (2 characters) on the left!
    
    def incrementSEQ(self, val):   
        if not isinstance(val, int):
            self._logger.warning("Sequence value is not integer. Returning zero")
            return "0000"
        if val> 65535:
            self._logger.warning("Sequence value is greater than 65535. Resetting to zero")
            self._seq = 0
            return "0000"
        if val<0:
            self._logger.warning("Sequence value is negative. Resetting to zero")
            return "0000"

        seq = val+1
        self._seq = seq

        seq_hex = hex(seq)
        seq_hex = seq_hex[2:] # remove '0x'
        if len(seq_hex)==3:
            seq_hex = '0'+seq_hex
        elif len(seq_hex)==1:
            seq_hex = '000'+seq_hex
        elif len(seq_hex)==2:
            seq_str = '00'+seq_hex
        else:
            seq='0000'
        
        low_b = seq_hex[-2:]
        high_b = seq_hex[0:2]
        seq_str = low_b+high_b

        return seq_str

    def computeDataLen(self, data):
        if not isinstance(data, str):
            self._logger.error("Data is not of type string")
            return "0000"
        # We expect number of chartacters to be even (each byte is represented by two cahrs e.g. '0A')
        if (len(data)%2) != 0:
            data = '0'+data # Pad 0 from the left, as sometimes it's ignored!
        L = int(len(data)/2)
        len_hex = hex(L)
        len_hex = len_hex[2:] # remove '0x'
        if len(len_hex)==3:
            len_hex = '0'+len_hex
        elif len(len_hex)==1:
            len_hex = '000'+len_hex
        elif len(len_hex)==2:
            len_hex = '00'+len_hex
        else:
            len_hex='0000'
        
        low_b = len_hex[-2:]
        high_b = len_hex[0:2]
        len_str = low_b + high_b
        return len_str

    def decodeMsg(self, msg):
        data = None
        
        if not isinstance(msg, str):
            self._logger.error("Input message is not a string")
            return data

        # 10 bytes: STX+CTRL+Data_len+SEQ+CMD_ID+CRC16
        #            2 + 1  +    2   + 2 +   1  + 2
        MINIMUM_DATA_LENGTH=10*2
        if len(msg)<MINIMUM_DATA_LENGTH:
            self._logger.error("No data to decode")
            return data

        
        # Now we got minimum amount of data. Check if we have enough
        # Data length, bytes are reversed, according to SIYI SDK
        low_b = msg[6:8] # low byte
        high_b = msg[8:10] # high byte
        data_len = high_b+low_b
        data_len = int('0x'+data_len, base=16)
        char_len = data_len*2 # number of characters. Each byte is represented by two characters in hex, e.g. '0A'= 2 chars

        # check crc16, if msg is OK!
        msg_crc=msg[-4:] # last 4 characters
        payload=msg[:-4]
        expected_crc=crc16_str_swap(payload)
        if expected_crc!=msg_crc:
            self._logger.error("CRC16 is not valid. Got %s. Expected %s. Message might be corrupted!", msg_crc, expected_crc)
            return data
        
        # Sequence
        low_b = msg[10:12] # low byte
        high_b = msg[12:14] # high byte
        seq_hex = high_b+low_b
        seq = int('0x'+seq_hex, base=16)
        
        # CMD ID
        cmd_id = msg[14:16]
        
        # DATA
        if data_len>0:
            data = msg[16:16+char_len]
        else:
            data=''
        
        self._data = data
        self._data_len = data_len
        self._cmd_id = cmd_id
        
        return data, data_len, cmd_id, seq

    def encodeMsg(self, data, cmd_id):
        """
        Encodes a msg according to SDK protocol

        Returns
        --
        [str] Encoded msg. Empty string if crc16 is not successful
        """
        seq = self.incrementSEQ(self._seq)
        data_len = self.computeDataLen(data)
        
        # msg_front = self.HEADER+self._ctr+data_len+seq+cmd_id+data
        msg_front = self.HEADER+self._ctr+data_len+'0000'+cmd_id+data
        crc = crc16_str_swap(msg_front)
        if crc is not None:
            msg = msg_front+crc
            self._logger.debug("Encoded msg: %s", msg)
            return msg
        else:
            self._logger.error("Could not encode message. crc16 is None")
            return ''

    ########################################################
    #               Message definitions                    #
    ########################################################
    
    def firmwareVerMsg(self):
        """
        Returns message string of the Acqsuire Firmware Version msg
        """
        data=""
        cmd_id = COMMAND.ACQUIRE_DEVICE_INF
        return self.encodeMsg(data, cmd_id)
    
    def hwIdMsg(self):
        """
        Returns message string for the Acquire Hardware ID
        """
        data=""
        cmd_id = COMMAND.ACQUIRE_DEVICE_INF
        return self.encodeMsg(data, cmd_id)

    def MoutionMsg(self):
        """
        Gimbal moiton mode information
        """
        data=""
        cmd_id = COMMAND.ACQUIRE_GIMBAL_MOUTION
        return self.encodeMsg(data, cmd_id)

    def autoFocusMsg(self):
        """
        Auto focus msg
        """
        data="01"
        cmd_id = COMMAND.AUTO_FOCUS
        return self.encodeMsg(data, cmd_id)

    def centerMsg(self):
        """
        Center gimbal msg
        """
        data="01"
        cmd_id = COMMAND.CENTER
        return self.encodeMsg(data, cmd_id)

    def zoomInMsg(self):
        """
        Zoom in Msg
        """
        data="01"
        cmd_id = COMMAND.MANUAL_ZOOM
        return self.encodeMsg(data, cmd_id)

    def zoomOutMsg(self):
        """
        Zoom out Msg
        """
        data="ff"
        cmd_id = COMMAND.MANUAL_ZOOM
        return self.encodeMsg(data, cmd_id)

    def stopZoomMsg(self):
        """
        Stop Zoom Msg
        """
        data="00"
        cmd_id = COMMAND.MANUAL_ZOOM
        return self.encodeMsg(data, cmd_id)

    def takePhotoMsg(self):
        data="00"
        cmd_id = COMMAND.PHOTO_VIDEO_HDR
        return self.encodeMsg(data, cmd_id)

    def recordMsg(self):
        data="02"
        cmd_id = COMMAND.PHOTO_VIDEO_HDR
        return self.encodeMsg(data, cmd_id)

    def lockModeMsg(self):
        data="03"
        cmd_id = COMMAND.PHOTO_VIDEO_HDR
        return self.encodeMsg(data, cmd_id)

    def followModeMsg(self):
        data="04"
        cmd_id = COMMAND.PHOTO_VIDEO_HDR
        return self.encodeMsg(data, cmd_id)
    
    def fpvModeMsg(self):
        data="05"
        cmd_id = COMMAND.PHOTO_VIDEO_HDR
        return self.encodeMsg(data, cmd_id)

    def gimbalAttMsg(self):
        """
        Acquire Gimbal Attiude msg
        """
        data=""
        cmd_id = COMMAND.ACQUIRE_GIMBAL_ATT
        return self.encodeMsg(data, cmd_id)

    def AllTempMsg(self):

        data="01"
        cmd_id = COMMAND.Max_Min_Temp
        return self.encodeMsg(data, cmd_id)

    def RangeFinderMsg(self):
        data=""
        cmd_id = COMMAND.RANGE_FİNDER
        return self.encodeMsg(data, cmd_id)

    def PointTempMsg(self,pointx,pointy):
        data = Hexcon(pointx)+ Hexcon(pointy)+"01"
        cmd_id = COMMAND.POINT_TEMP
        return self.encodeMsg(data, cmd_id)

    def BoxTempMsg(self,startx,starty,endx,endy):
        data = Hexcon(startx)+ Hexcon(starty)+ Hexcon(endx)+ Hexcon(endy) +"01"
        cmd_id = COMMAND.BOX_TEMP
        return self.encodeMsg(data, cmd_id)

    def InfColorMapMsg(self):
        data = ""
        cmd_id = COMMAND.INF_COLOR_MAP
        return self.encodeMsg(data, cmd_id)
      
    def ColorMapMsg(self,color):
        data = toHex(color,8)
        cmd_id = COMMAND.COLOR_MAP
        return self.encodeMsg(data, cmd_id)

    def InfImageModMsg(self):
        data = ""
        cmd_id = COMMAND.IMAGE_MOD
        return self.encodeMsg(data, cmd_id)

    def ImageModMsg(self,mode):
        data = toHex(mode,8)
        cmd_id = COMMAND.IMAGE_MOD_CHANGE
        return self.encodeMsg(data, cmd_id)
    
    def gimbalTargetMsg(self,yaw,pitch):
        data = Hexcon(yaw)+ Hexcon(pitch)
        cmd_id = COMMAND.TargetAngle
        return self.encodeMsg(data,cmd_id)
   
    def RangefinderStatusMsg(self):
        data = ""
        cmd_id = COMMAND.Range_finder_params_get
        return self.encodeMsg(data, cmd_id)

    def RangefinderStatusSendMsg(self,state):
        data = toHex(state,8)
        cmd_id = COMMAND.Range_finder_params_send
        return self.encodeMsg(data, cmd_id)   
   
    def ThermalGainMsg(self):
        data = ""
        cmd_id = COMMAND.Thermal_Gain_Get
        return self.encodeMsg(data, cmd_id)
    
    def ThermalGainSendMsg(self,gain):
        data = toHex(gain,8)
        cmd_id = COMMAND.Thermal_Gain_Send
        return self.encodeMsg(data, cmd_id)

    def ThermalRAWDataMsg(self,mode):
        data = toHex(mode,8)
        cmd_id = COMMAND.Thermal_Raw_data
        return self.encodeMsg(data, cmd_id)   
   
    def ThermalMAPMsg(self):
        data = ""
        cmd_id = COMMAND.Thermal_Map
        return self.encodeMsg(data, cmd_id)

    def ThermalParamsGetMsg(self):
        data = ""
        cmd_id = COMMAND.Thermal_Params_Get
        return self.encodeMsg(data, cmd_id)

    def ThermalParamsSendMsg(self,Dist,Ems,Hum,Ta,Tu):
        data = Hexcon(Dist)+Hexcon(Ems)+Hexcon(Hum)+Hexcon(Ta)+Hexcon(Tu)
        cmd_id = COMMAND.Thermal_Params_Send
        return self.encodeMsg(data, cmd_id)   
   
      
        
    def gimbalSpeedMsg(self, yaw_speed, pitch_speed):
        if yaw_speed>100:
            yaw_speed=100
        if yaw_speed<-100:
            yaw_speed=-100

        if pitch_speed>100:
            pitch_speed=100
        if pitch_speed<-100:
            pitch_speed=-100

        data1=toHex(yaw_speed, 8)
        data2=toHex(pitch_speed, 8)
        data=data1+data2
        cmd_id = COMMAND.GIMBAL_ROT
        return self.encodeMsg(data, cmd_id)

    

