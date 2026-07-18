"""Manual hardware probe; intentionally not an automated test."""


def main():
    import serial
    import time

    with serial.Serial('/dev/ttyACM0', 115200, timeout=1) as connection:
        def send_cmd(command):
            connection.write((command + '\r').encode('utf-8'))
            time.sleep(0.5)
            response = connection.read(connection.in_waiting).decode('utf-8', errors='ignore')
            print(f"[{command}] -> {response!r}")

        for command in (
            '', '', 'sensorStop', 'flushCfg', 'dfeDataOutputMode 1', 'channelCfg 15 1 0 0 0',
            'adcCfg 2 0', 'lowPower 0 0', 'profileCfg 0 77 34 6 66 0 0 60 0 256 5000 0 0 30',
            'frameCfg 0 0 128 0 256 200 1 0',
        ):
            send_cmd(command)


if __name__ == '__main__':
    main()
