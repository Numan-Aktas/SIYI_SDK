import socket
from siyi_message import *
from time import sleep, time
import logging
from utils import  toInt
import threading


class SIYISDK:
    def __init__(self, server_ip="192.168.144.25", port=37260, debug=False):
        """
        
        Params
        --
        - server_ip [str] IP address of the camera
        - port: [int] UDP port of the camera
        """
        self._debug= debug # print debug messages
        if self._debug:
            d_level = logging.DEBUG
        else:
            d_level = logging.INFO
        LOG_FORMAT=' [%(levelname)s] %(asctime)s [SIYISDK::%(funcName)s] :\t%(message)s'
        logging.basicConfig(format=LOG_FORMAT, level=d_level)
        self._logger = logging.getLogger(self.__class__.__name__)

        # Message sent to the camera
        self._out_msg = SIYIMESSAGE(debug=self._debug)
        
        # Message received from the camera
        self._in_msg = SIYIMESSAGE(debug=self._debug)        

        self._server_ip = server_ip
        self._port = port

        self._BUFF_SIZE=1024

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rcv_wait_t = 2 # Receiving wait time
        self._socket.settimeout(self._rcv_wait_t)

        self._connected = False

        self._fw_msg = DeviceMsg()
        self._hw_msg = DeviceMsg()
        self._autoFocus_msg = AutoFocusMsg()
        self._gmm_msg = MoutionMode()
        self._manualZoom_msg=ManualZoomMsg()
        self._center_msg=CenterMsg()
        self._gimbalSpeed_msg=GimbalSpeedMsg()
        self._att_msg=AttitdueMsg()
        self._temp_msg = TemperatureMsg()
        self._box_temp_msg = BoxTemperatureMsg()
        self._rangefinder_msg = RangeFinderMsg()
        self._last_att_seq=-1

        # Stop threads
        self._stop = False # used to stop the above thread
        
        self._recv_thread = threading.Thread(target=self.recvLoop)

        # Connection thread
        self._last_fw_seq=0 # used to check on connection liveness
        self._conn_loop_rate = 1 # seconds
        self._conn_thread = threading.Thread(target=self.connectionLoop, args=(self._conn_loop_rate,))

        # Gimbal info thread @ 1Hz
        self._gimbal_info_loop_rate = 1
        self._g_info_thread = threading.Thread(target=self.gimbalInfoLoop,
                                                args=(self._gimbal_info_loop_rate,))

        # Gimbal attitude thread @ 10Hz
        self._gimbal_att_loop_rate = 0.1
        self._g_att_thread = threading.Thread(target=self.gimbalAttLoop,
                                                args=(self._gimbal_att_loop_rate,))

    def resetVars(self):
        """
        Resets variables to their initial values. For example, to prepare for a fresh connection
        """
        self._connected = False
        self._fw_msg = DeviceMsg()
        self._hw_msg = DeviceMsg()
        self._autoFocus_msg = AutoFocusMsg()
        self._gmm_msg = MoutionMode()
        self._manualZoom_msg=ManualZoomMsg()
        self._center_msg=CenterMsg()
        self._gimbalSpeed_msg=GimbalSpeedMsg()
        self._att_msg=AttitdueMsg()
        self._temp_msg = TemperatureMsg()
        self._box_temp_msg = BoxTemperatureMsg()
        self._rangefinder_msg = RangeFinderMsg()

        return True

    def connect(self, maxWaitTime=3.0):
        self._recv_thread.start()
        self._conn_thread.start()
        t0 = time()
        while(True):
            if self._connected:
                self._g_info_thread.start()
                self._g_att_thread.start()
                return True
            if (time() - t0)>maxWaitTime and not self._connected:
                self.disconnect()
                self._logger.error("Failed to connect to camera")
                return False

    def disconnect(self):
        self._logger.info("Stopping all threads")
        self._stop = True # stop the connection checking thread
        self.resetVars()

    def checkConnection(self):

        self.requestFirmwareVersion()
        sleep(0.1)
        if self._fw_msg.seq!=self._last_fw_seq and len(self._fw_msg.gimbal_firmware_ver)>0:
            self._connected = True
            self._last_fw_seq=self._fw_msg.seq
        else:
            self._connected = False

    def connectionLoop(self, t):
        while(True):
            if self._stop:
                self._connected=False
                self.resetVars()
                self._logger.warning("Connection checking loop is stopped. Check your connection!")
                break
            self.checkConnection()
            sleep(t)

    def isConnected(self):
        return self._connected

    def gimbalInfoLoop(self, t):
        while(True):
            if not self._connected:
                self._logger.warning("Gimbal info thread is stopped. Check connection")
                break
            self.requestFirmwareVersion()
            sleep(t)

    def gimbalAttLoop(self, t):
        while(True):
            if not self._connected:
                self._logger.warning("Gimbal attitude thread is stopped. Check connection")
                break
            self.requestGimbalAttitude()
            sleep(t)

    def sendMsg(self, msg):
        b = bytes.fromhex(msg)
        try:
            self._socket.sendto(b, (self._server_ip, self._port))
            return True
        except Exception as e:
            self._logger.error("Could not send bytes")
            return False

    def rcvMsg(self):
        data=None
        try:
            data,addr = self._socket.recvfrom(self._BUFF_SIZE)
        except Exception as e:
            self._logger.warning("%s. Did not receive message within %s second(s)", e, self._rcv_wait_t)
        return data

    def recvLoop(self):
        self._logger.debug("Started data receiving thread")
        while( not self._stop):
            try:
                self.bufferCallback()
            except:
                exit
        self._logger.debug("Exiting data receiving thread")

    def bufferCallback(self):
        """
        Receives messages and parses its content
        """
        try:
            buff,addr = self._socket.recvfrom(self._BUFF_SIZE)
        except:
            exit
        buff_str = buff.hex()
        self._logger.debug("Buffer: %s", buff_str)

        # 10 bytes: STX+CTRL+Data_len+SEQ+CMD_ID+CRC16
        #            2 + 1  +    2   + 2 +   1  + 2
        MINIMUM_DATA_LENGTH=10*2

        HEADER='5566'
        # Go through the buffer
        while(len(buff_str)>=MINIMUM_DATA_LENGTH):
            if buff_str[0:4]!=HEADER:
                # Remove the 1st element and continue 
                tmp=buff_str[1:]
                buff_str=tmp
                continue

            # Now we got minimum amount of data. Check if we have enough
            # Data length, bytes are reversed, according to SIYI SDK
            low_b = buff_str[6:8] # low byte
            high_b = buff_str[8:10] # high byte
            data_len = high_b+low_b
            data_len = int('0x'+data_len, base=16)
            char_len = data_len*2

            # Check if there is enough data (including payload)
            if(len(buff_str) < (MINIMUM_DATA_LENGTH+char_len)):
                # No useful data
                buff_str=''
                break
            
            packet = buff_str[0:MINIMUM_DATA_LENGTH+char_len]
            buff_str = buff_str[MINIMUM_DATA_LENGTH+char_len:]

            # Finally decode the packet!
            val = self._in_msg.decodeMsg(packet)
            if val is None:
                continue
            
            data, data_len, cmd_id, seq = val[0], val[1], val[2], val[3]
            #print (data, data_len, cmd_id, seq,"siyi")
            if cmd_id==COMMAND.ACQUIRE_DEVICE_INF:
                self.parseDevicemsg(data, seq)
            elif cmd_id==COMMAND.AUTO_FOCUS:
                self.parseAutoFocusMsg(data, seq)
            elif cmd_id==COMMAND.CENTER:
                self.parseGimbalCenterMsg(data, seq) 
            elif cmd_id==COMMAND.ACQUIRE_GIMBAL_MOUTION:
                self.parseMoutionModeMsg(data,seq)
            elif cmd_id==COMMAND.MANUAL_ZOOM:
                self.parseZoomMsg(data, seq)
            elif cmd_id==COMMAND.ACQUIRE_GIMBAL_ATT:
                self.parseAttitudeMsg(data, seq)
            elif cmd_id==COMMAND.Max_Min_Temp:
                self.parsTempratureMsg(data, seq)
            elif cmd_id==COMMAND.BOX_TEMP:
                self.parsBoxTempratureMsg(data, seq)
            elif cmd_id==COMMAND.RANGE_FİNDER:
                self.parseRangeFinderMsg(data, seq)
            elif cmd_id==COMMAND.GIMBAL_ROT:
                self.parseGimbalSpeedMsg(data, seq)

            else:
                self._logger.warning("CMD ID is not recognized")
        
        return
    

    ####################################################
    #                Parsing functions                 #
    ####################################################
    def parseDevicemsg(self, msg:str, seq:int):
        try:
            self._fw_msg.gimbal_firmware_ver= str(toInt(msg[12:14]))+"."+str(toInt(msg[10:12]))+"."+str(toInt(msg[8:10]))
            self._fw_msg.zoom_firmware_ver= str(toInt(msg[20:22]))+"."+str(toInt(msg[18:20]))+"."+str(toInt(msg[16:18]))
            if msg[14:16]=="6b":
                self._hw_msg.id="ZR10"
            elif msg[14:16]=="73":
                self._hw_msg.id="A8 Mini"
            elif msg[14:16]=="75":
                self._hw_msg.id="A2 Mini"            
            elif msg[14:16]=="78":
                self._hw_msg.id="ZR30" 
            elif msg[14:16]=="7a":
                self._hw_msg.id="ZT30"

            self._fw_msg.seq=seq
            self._hw_msg.seq=seq
            
            self._logger.debug("Firmware version: %s", self._fw_msg.gimbal_firmware_ver)

            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseAutoFocusMsg(self, msg:str, seq:int):
        
        try:
            self._autoFocus_msg.seq=seq
            self._autoFocus_msg.success = bool(int('0x'+msg, base=16))
            self._logger.debug("Auto focus success: %s", self._autoFocus_msg.success)

            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseMoutionModeMsg(self, msg:str, seq:int):
        try:
            self._gmm_msg.seq=seq
            mode =str(int('0x'+msg, base=16))
            if mode=="0":
                self._gmm_msg.gimbal_moution_mode = "lock mode"
            elif mode=="1":
                self._gmm_msg.gimbal_moution_mode = "follow mode"            
            elif mode=="2":
                self._gmm_msg.gimbal_moution_mode = "fpv mode"

            self._logger.debug("Moution Mode: %s", self._gmm_msg.gimbal_moution_mode)

            return self._gmm_msg.gimbal_moution_mode
        
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseGimbalCenterMsg(self, msg:str, seq:int):  
            try:
                self._center_msg.seq=seq
                self._center_msg.success = bool(int('0x'+msg, base=16))
                self._logger.debug("Gimbal center success: %s", self._center_msg.success)
                return True
            except Exception as e:
                self._logger.error("Error %s", e)
                return False

    def parseZoomMsg(self, msg:str, seq:int):
        try:
            self._manualZoom_msg.seq=seq
            self._manualZoom_msg.level = int('0x'+msg[2:4]+msg[0:2], base=16) /10.
            self._logger.debug("Zoom level %s", self._manualZoom_msg.level)
            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseAttitudeMsg(self, msg:str, seq:int):
        
        try:
            self._att_msg.seq=seq
            self._att_msg.yaw = toInt(msg[2:4]+msg[0:2]) /10.
            self._att_msg.pitch = toInt(msg[6:8]+msg[4:6]) /10.
            self._att_msg.roll = toInt(msg[10:12]+msg[8:10]) /10.
            self._att_msg.yaw_speed = toInt(msg[14:16]+msg[12:14]) /10.
            self._att_msg.pitch_speed = toInt(msg[18:20]+msg[16:18]) /10.
            self._att_msg.roll_speed = toInt(msg[22:24]+msg[20:22]) /10.

            self._logger.debug("(yaw, pitch, roll= (%s, %s, %s)", 
                                    self._att_msg.yaw, self._att_msg.pitch, self._att_msg.roll)
            self._logger.debug("(yaw_speed, pitch_speed, roll_speed= (%s, %s, %s)", 
                                    self._att_msg.yaw_speed, self._att_msg.pitch_speed, self._att_msg.roll_speed)
            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseGimbalSpeedMsg(self, msg:str, seq:int):
        
        try:
            self._gimbalSpeed_msg.seq=seq
            self._gimbalSpeed_msg.success = bool(int('0x'+msg, base=16))

            
            self._logger.debug("Gimbal speed success: %s", self._gimbalSpeed_msg.success)

            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parsTempratureMsg(self, msg:str, seq:int):
        try:
            self._temp_msg.seq=seq
            self._temp_msg.temp_max = toInt(msg[2:4]+msg[0:2]) /100.
            self._temp_msg.temp_min = toInt(msg[6:8]+msg[4:6])/100.
            self._temp_msg.temp_max_x =  toInt(msg[10:12]+msg[8:10])
            self._temp_msg.temp_max_y = toInt(msg[14:16]+msg[12:14]) 
            self._temp_msg.temp_min_x = toInt(msg[18:20]+msg[16:18])
            self._temp_msg.temp_min_y = toInt(msg[22:24]+msg[20:22])
            self._logger.debug("(max_temp, max_temp_x, max_temp_y= (%s, %s, %s)", 
                                    self._temp_msg.temp_max, self._temp_msg.temp_max_x, self._temp_msg.temp_max_y)
            self._logger.debug("(min_temp, min_temp_x, min_temp_y= (%s, %s, %s)", 
                                    self._temp_msg.temp_min, self._temp_msg.temp_min_x, self._temp_msg.temp_min_y)            
            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseRangeFinderMsg(self, msg:str, seq:int):
        try:
            self._rangefinder_msg.seq=seq
            self._rangefinder_msg.Range_value= toInt(msg[2:4]+msg[0:2]) /10.
            self._logger.debug("Range_value %s m",self._rangefinder_msg.Range_value)
            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parsBoxTempratureMsg(self, msg:str, seq:int):
        print(len(msg),msg)
        try:
            self._box_temp_msg.seq=seq
            self._box_temp_msg.startx = toInt(msg[2:4]+msg[0:2]) 
            self._box_temp_msg.starty = toInt(msg[6:8]+msg[4:6])
            self._box_temp_msg.endx =  toInt(msg[10:12]+msg[8:10])
            self._box_temp_msg.endy = toInt(msg[14:16]+msg[12:14]) 

            self._box_temp_msg.temp_max = toInt(msg[18:20]+msg[16:18])/100.
            self._box_temp_msg.temp_min = toInt(msg[22:24]+msg[20:22])/100.

            self._box_temp_msg.temp_max_x = toInt(msg[26:28]+msg[24:26])
            self._box_temp_msg.temp_max_y = toInt(msg[30:32]+msg[28:30])
            self._box_temp_msg.temp_min_x = toInt(msg[34:36]+msg[32:34])
            self._box_temp_msg.temp_min_y = toInt(msg[38:40]+msg[36:38])

            self._logger.debug("startx,starty,endx,endy= (%s, %s, %s, %s)",
                               self._box_temp_msg.startx,self._box_temp_msg.starty,self._box_temp_msg.endx,self._box_temp_msg.endy)
            self._logger.debug("(max_temp, max_temp_x, max_temp_y= (%s, %s, %s)", 
                                    self._box_temp_msg.temp_max, self._box_temp_msg.temp_max_x, self._box_temp_msg.temp_max_y)
            self._logger.debug("(min_temp, min_temp_x, min_temp_y= (%s, %s, %s)", 
                                    self._box_temp_msg.temp_min, self._box_temp_msg.temp_min_x, self._box_temp_msg.temp_min_y)           
            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    ##################################################
    #           Backand Request functions            #
    ##################################################    
    def requestFirmwareVersion(self):
        msg = self._out_msg.firmwareVerMsg()
        if not self.sendMsg(msg):
            return False
        return True
    
    def requestMoiton(self):
        msg = self._out_msg.MoutionMsg()
        if not self.sendMsg(msg):
            return False

    def requestZoomIn(self):
        msg = self._out_msg.zoomInMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestZoomOut(self):
        """
        Sends request for zoom out

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.zoomOutMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestZoomHold(self):
        """
        Sends request for stopping zoom

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.stopZoomMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def getAttitude(self):
        while True:
            cam.requestGimbalAttitude()
            if len(str(self._att_msg.yaw)) :
                break
            sleep(0.2)
        return(self._att_msg.yaw, self._att_msg.pitch, self._att_msg.roll)

    def getAttitudeSpeed(self):
        return(self._att_msg.yaw_speed, self._att_msg.pitch_speed, self._att_msg.roll_speed)

    def requestGimbalAttitude(self):
        msg = self._out_msg.gimbalAttMsg()
        if not self.sendMsg(msg):
            return False
        return True
   
    def requestGimbalSpeed(self, yaw_speed:int, pitch_speed:int):
        msg = self._out_msg.gimbalSpeedMsg(yaw_speed, pitch_speed)
        if not self.sendMsg(msg):
            return False
        return True  
    
    def requestGimbalAngle(self,yaw_angle,pitch_angle):
        msg = self._out_msg.gimbalTargetMsg(yaw_angle, pitch_angle)
        if not self.sendMsg(msg):
            return False
        return True 
   
    def requestMaxMinTemp(self):
        msg = self._out_msg.AllTempMsg()
        if not self.sendMsg(msg):
            return False
        return True
    
    def requestBoxTemp(self):
        msg = self._out_msg.BoxTempMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestRangeFinder(self):
        msg = self._out_msg.RangeFinderMsg()
        if not self.sendMsg(msg):
            return False
        return True
    
    
    ##################################################
    #               Request functions                #
    ################################################## 
    
    def requestAutoFocus(self):
        msg = self._out_msg.autoFocusMsg()
        if not self.sendMsg(msg):
            return False
        return True
    
    def requestCenterGimbal(self):
        msg = self._out_msg.centerMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestZoomSet(self, target_zoom, tolerance=0.5):
        current_zoom = cam.getZoomLevel()
        zoom_step_delay = 0.5  # Başlangıçta her zoom adımı arasında bekleme süresi (saniye cinsinden)

        while abs(current_zoom - target_zoom) > tolerance:
            zoom_difference = target_zoom - current_zoom
            
            if zoom_difference > 0:
                cam.requestZoomIn()
            else:
                cam.requestZoomOut()

            # Zoom işleminin tamamlanması için bekle
            sleep(zoom_step_delay)

            # Durumu tekrar kontrol et
            current_zoom = cam.getZoomLevel()

            # Hedefe yaklaştıkça bekleme süresini azalt
            if abs(current_zoom - target_zoom) < 5:
                zoom_step_delay = 0.1
        cam.requestZoomHold()
        cam.requestAutoFocus()

    def requestPhoto(self):
        msg = self._out_msg.takePhotoMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestRecording(self):
        msg = self._out_msg.recordMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestFPVMode(self):
        msg = self._out_msg.fpvModeMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestLockMode(self):
        msg = self._out_msg.lockModeMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestFollowMode(self):
        msg = self._out_msg.followModeMsg()
        if not self.sendMsg(msg):
            return False
        return True


    ##################################################
    #                   Get functions                #
    ##################################################
    def getGimbalFirmwareVersion(self):
        return(self._fw_msg.gimbal_firmware_ver)
    
    def getZoomFirmwareVersion(self):
        return(self._fw_msg.zoom_firmware_ver)

    def getHardwareID(self):
        return(self._hw_msg.id)

    def getMotionMode(self):
        while True:
            cam.requestMoiton()
            if len(self._gmm_msg.gimbal_moution_mode) :
                break
            sleep(0.2)
        return self._gmm_msg.gimbal_moution_mode

    def getZoomLevel(self):  
        while True:
            cam.requestZoomHold()
            if self._manualZoom_msg.level != -1 :
                break
            sleep(0.2)
        return(self._manualZoom_msg.level)

    def getMaxMinTemprature(self):
        while True:
            cam.requestMaxMinTemp()
            if self._temp_msg.temp_max:
                break
            sleep(0.2)
        return(self._temp_msg.temp_max, self._temp_msg.temp_max_x, self._temp_msg.temp_max_y,
               self._temp_msg.temp_min, self._temp_msg.temp_min_x, self._temp_msg.temp_min_y)

    def getRangeFinder(self):
        cam.requestRangeFinder()
        while True:
            cam.requestRangeFinder()
            if self._rangefinder_msg.Range_value != '':
                break
            sleep(0.2)
        return(self._rangefinder_msg.Range_value)
    
    def getBoxTemprature(self):
        while True:
            cam.requestBoxTemp()
            if self._box_temp_msg.temp_max !='':
                break
            sleep(0.8)
        return(self._box_temp_msg.startx,self._box_temp_msg.starty,self._box_temp_msg.endx,self._box_temp_msg.endy,
                  self._box_temp_msg.temp_max, self._box_temp_msg.temp_max_x, self._box_temp_msg.temp_max_y,
                  self._box_temp_msg.temp_min, self._box_temp_msg.temp_min_x, self._box_temp_msg.temp_min_y)

    #################################################
    #                 Set functions                 #
    #################################################
    def setGimbalRotation(self, yaw, pitch, err_thresh=1.0, kp=4):

        if (pitch >25 or pitch <-90):
            self._logger.error("desired pitch is outside controllable range -90~25")
            return

        if (yaw >45 or yaw <-45):
            self._logger.error("Desired yaw is outside controllable range -45~45")
            return

        th = err_thresh
        gain = kp
        while(True):
            self.requestGimbalAttitude()
            if self._att_msg.seq==self._last_att_seq:
                self._logger.info("Did not get new attitude msg")
                self.requestGimbalSpeed(0,0)
                continue

            self._last_att_seq = self._att_msg.seq

            yaw_err = -yaw + self._att_msg.yaw # NOTE for some reason it's reversed!!
            pitch_err = pitch - self._att_msg.pitch

            self._logger.debug("yaw_err= %s", yaw_err)
            self._logger.debug("pitch_err= %s", pitch_err)

            if (abs(yaw_err) <= th and abs(pitch_err)<=th):
                self.requestGimbalSpeed(0, 0)
                self._logger.info("Goal rotation is reached")
                break

            y_speed_sp = max(min(100, int(gain*yaw_err)), -100)
            p_speed_sp = max(min(100, int(gain*pitch_err)), -100)
            self._logger.debug("yaw speed setpoint= %s", y_speed_sp)
            self._logger.debug("pitch speed setpoint= %s", p_speed_sp)
            self.requestGimbalSpeed(y_speed_sp, p_speed_sp)

            sleep(0.1) # command frequency




def test():
    global cam
    cam=SIYISDK(debug=False)

    if not cam.connect():
        exit(1)
    while True:
        print(cam.getBoxTemprature())
        


    cam.disconnect()

if __name__=="__main__":
    test()