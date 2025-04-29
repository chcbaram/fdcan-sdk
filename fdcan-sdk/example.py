import sys
import time
import os

from sdk.fdcan_sdk import *


def fdcanReceive(packet : CmdPacket):
  print(packet)


def main():
  
  fdcan = FDCanSDK()

  fdcan.begin()  
  fdcan.setReceiveEvent(fdcanReceive)

  if fdcan.openPort():
    resp = fdcan.readVersion()
    if resp.err_code == OK:
      print(resp.boot.name_str)
      print(resp.boot.version_str)
      print(resp.firm.name_str)
      print(resp.firm.version_str)

    resp = fdcan.openCan(CAN_MODE_NORMAL, CAN_FRAME_FD_BRS, CAN_BAUD_1M, CAN_BAUD_5M)
    print("openCan() -> " + str(resp.err_code))
    
    fdcan.setFilterMask(CAN_EXT, 0, 0)
    print("setFilterMask() -> " + str(resp.err_code))

    while(True):
      time.sleep(1)
  else:
    print("No FDCAN")
  fdcan.closePort()
  exit()


if __name__ == "__main__":  
  main()
