import serial, time, struct, json, asyncio, websockets, threading, argparse, os

# 基础常量
MAGIC_WORD = b'\x02\x01\x04\x03\x06\x05\x08\x07'
latest_radar_frame = {"frame_num": 0, "points": []}

def send_config(port_name, cfg_file):
    print(f"🚀 [Legacy] 强制下发配置 -> {port_name}...")
    try:
        s = serial.Serial(port_name, 115200, timeout=1)
        with open(cfg_file, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('%'): continue
                s.write((line + '\r\n').encode())
                time.sleep(0.05)
                # 等待回写
                resp = ""
                while "mmwDemo:/>" not in resp:
                    resp += s.read(s.in_waiting).decode('utf-8', errors='ignore')
                    if time.time() - time.time() > 5: break # 简版超时
                print(f"  [OK] {line}")
        s.close()
        return True
    except Exception as e:
        print(f"❌ 配置下发异常: {e}"); return False

def data_thread(pname, baud):
    global latest_radar_frame
    print(f"🌊 [Legacy] 监听串口 {pname} ({baud})...")
    try:
        ser = serial.Serial(pname, baud, timeout=1)
        buffer = bytearray()
        while True:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                buffer.extend(chunk)
                
                magic_idx = buffer.find(MAGIC_WORD)
                if magic_idx == -1:
                    if len(buffer) > 4096: buffer = buffer[-2048:]
                    continue
                if magic_idx > 0: buffer = buffer[magic_idx:]
                if len(buffer) < 40: continue
                
                header = struct.unpack('<8sIIIIIIII', buffer[:40])
                total_len = header[2]
                frame_num = header[4]
                num_tlvs = header[7]
                
                if len(buffer) < total_len: continue
                
                # 极简解析
                frame_data = buffer[40:total_len]
                buffer = buffer[total_len:]
                pts = []
                offset = 0
                for _ in range(num_tlvs):
                    t_type, t_len = struct.unpack('<II', frame_data[offset:offset+8])
                    offset += 8
                    if t_type == 1:
                        p_cnt = t_len // 16
                        for p in range(p_cnt):
                            x,y,z,v = struct.unpack('<ffff', frame_data[offset:offset+16])
                            pts.append({"x": x, "y": y, "z": z, "v": v})
                            offset += 16
                    else: offset += t_len
                latest_radar_frame = {"frame_num": frame_num, "points": pts}
                print(f"\r[Live] 帧: {frame_num} | 点数: {len(pts)}", end="")
    except Exception as e: print(f"💥 数据线程退出: {e}")

async def server(ws, path):
    print("📱 客户端连接。")
    last_id = -1
    while True:
        try:
            if latest_radar_frame["frame_num"] != last_id:
                await ws.send(json.dumps(latest_radar_frame))
                last_id = latest_radar_frame["frame_num"]
            await asyncio.sleep(0.04)
        except: break

async def main_ws(port):
    print(f"🌐 [Legacy] WebSocket 服务器就绪: ws://0.0.0.0:{port}")
    async with websockets.serve(server, '0.0.0.0', port):
        await asyncio.Future()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--cfg_port', default='/dev/ttyACM0')
    parser.add_argument('--data_port', default='/dev/ttyACM1')
    parser.add_argument('--baud', type=int, default=3125000)
    parser.add_argument('--cfg_file', default='')
    ARGS = parser.parse_args()
    
    if ARGS.cfg_file: send_config(ARGS.cfg_port, ARGS.cfg_file)
    threading.Thread(target=data_thread, args=(ARGS.data_port, ARGS.baud), daemon=True).start()
    
    try:
        asyncio.run(main_ws(8765))
    except KeyboardInterrupt:
        print("\n中枢已关闭。")
