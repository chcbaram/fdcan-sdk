import sys
import time
import os
import serial
import serial.tools.list_ports as sp
import struct

from os import path

from sdk.cmd import *
from sdk.cmd_boot import *
from sdk.cmd_can import *
from sdk.err_code import *



class FDCanSDK:
  
  def __init__(self):
    self.is_run = True
    self.ports = []
    self.cmd = Cmd()
    self.cmd_boot = CmdBoot(self.cmd)
    self.cmd_can = CmdCAN(self.cmd)

  def __del__(self):
    self.cmd.close()

  def begin(self):
    self.scanPorts()

  def scanPorts(self):
    ret = self.cmd.scanPorts()
    self.ports = self.cmd.ports
    return ret

  def getPortCount(self):  
    return self.cmd.getPortCount()

  def closePort(self):
    self.cmd_can.close()
    self.cmd.close()
  
  def openPort(self, port = None):
    ret = False
    if port == None:
      if self.cmd.getPortCount() > 0:
        ret = self.cmd.open(self.ports[0], 600)
      else:
        self.scanPorts()        
        if self.cmd.getPortCount() > 0:
          ret = self.cmd.open(self.ports[0], 600)
    else:
      ret = self.cmd.open(port, 600)
    
    return ret
  
  def openPortByNum(self, port_num : int):
    ret = False
    if self.cmd.getPortCount() > 0 and port_num < self.cmd.getPortCount():
      ret = self.cmd.open(self.ports[port_num], 600)
    return ret
    
  def readVersion(self):
    return self.cmd_boot.readVersion()
  
  def setReceiveEvent(self, event_func):
    self.cmd.callbacks.append(event_func)

  def openCan(self, param :CmdCANOpen, timeout=500):
    resp = CmdResp()
    resp.err_code, ret_data = self.cmd_can.open(param, timeout)
    return resp
  
  def openCan(self, mode, frame, baud, baud_data, timeout=500):    
    resp = CmdResp()

    param = CmdCANOpen()
    param.mode  = mode
    param.frame = frame
    param.baud  = baud
    param.baud_data = baud_data
    resp.err_code, ret_data = self.cmd_can.open(param, timeout)
    return resp  
  
  def setFilterMask(self, id_type, id, mask):
    resp = CmdResp()
    param = CmdCANFilter()
    param.type = CAN_ID_MASK
    param.id_type = id_type
    param.id1 = id
    param.id2 = mask
    resp.err_code, ret = self.cmd_can.setFilter(param)  
    return resp
  
  def setFilterRange(self, id_type, id_begin, id_end):
    resp = CmdResp()
    param = CmdCANFilter()
    param.type = CAN_ID_MASK
    param.id_type = id_type
    param.id1 = id_begin
    param.id2 = id_end
    resp.err_code, ret = self.cmd_can.setFilter(param)  
    return resp  