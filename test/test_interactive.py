import serial
import time

s = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

def send_cmd(cmd):
    s.write((cmd + '\r').encode('utf-8'))
    time.sleep(0.5)
    resp = s.read(s.in_waiting).decode('utf-8', errors='ignore')
    # Print repr to see exact characters without garbling the terminal
    print(f"[{cmd}] -> {repr(resp)}")

send_cmd('')
send_cmd('')
send_cmd('sensorStop')
send_cmd('flushCfg')
send_cmd('dfeDataOutputMode 1')
send_cmd('channelCfg 15 1 0 0 0')
send_cmd('adcCfg 2 0')
send_cmd('lowPower 0 0')
send_cmd('profileCfg 0 77 34 6 66 0 0 60 0 256 5000 0 0 30')
send_cmd('frameCfg 0 0 128 0 256 200 1 0')

s.close()
