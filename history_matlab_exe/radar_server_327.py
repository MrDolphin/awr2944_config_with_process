import serial, time, struct, json, asyncio, websockets, threading, argparse, sys, os

# --- 核心解析常数 (3.27 基准) ---
MAGIC_WORD = b'\x02\x01\x04\x03\x06\x05\x08\x07'
latest_radar_frame = {"frame_num": 0, "points": []}

def send_config(port_name, cfg_file):
    # 路径健壮性增强：自动尝试 Config 目录
    if not os.path.exists(cfg_file):
        alt_path = os.path.join("/home/pi/Config", os.path.basename(cfg_file))
        if os.path.exists(alt_path):
            cfg_file = alt_path
        else:
            print(f"❌ 配置文件不存在: {cfg_file} (且在 Config 目录下也未找到)")
            return False

    print(f"⚙️ [3.27版] 正在下发配置: {cfg_file} -> {port_name}...")
    try:
        s = serial.Serial(port_name, 115200, timeout=1)
        with open(cfg_file, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('%'): continue
                s.write((line + '\r\n').encode())
                time.sleep(0.05)
                # 针对导致芯片卡死的长耗时指令，增加等待时间
                timeout = 5.0 if any(c in line for c in ["calib", "sensorStart", "measure", "cfar"]) else 1.0
                if "sensorStart" in line: time.sleep(1.0) # 发起扫描前让芯片缓一下
                
                resp = ""
                start_t = time.time()
                while "mmwDemo:/>" not in resp and (time.time() - start_t) < timeout:
                    if s.in_waiting > 0:
                        resp += s.read(s.in_waiting).decode('utf-8', errors='ignore')
                    else:
                        time.sleep(0.01)
                print(f"  > {line}")
        s.close()
        return True
    except Exception as e:
        print(f"❌ 配置失败: {e}"); return False

def radar_thread(pname, baud):
    global latest_radar_frame
    print(f"🌊 [3.27版] 解析引擎启动: {pname}...")
    buffer = bytearray()
    try:
        ser = serial.Serial(pname, baud, timeout=1)
        ser.reset_input_buffer()
        while True:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                buffer.extend(chunk)
                
                # 原始极速寻址逻辑
                idx = buffer.find(MAGIC_WORD)
                if idx == -1:
                    if len(buffer) > 65536: buffer = buffer[-2048:]
                    continue
                if idx > 0: buffer = buffer[idx:]
                if len(buffer) < 40: continue
                
                try:
                    header = struct.unpack('<8sIIIIIIII', buffer[:40])
                    p_len, f_num, n_tlvs = header[2], header[4], header[7]
                except: continue
                
                if len(buffer) < p_len: continue
                
                f_data = buffer[40:p_len]
                buffer = buffer[p_len:]
                pts = []
                offset = 0
                for _ in range(n_tlvs):
                    t_type, t_len = struct.unpack('<II', f_data[offset:offset+8])
                    offset += 8
                    if t_type == 1:
                        p_cnt = t_len // 16
                        for p in range(p_cnt):
                            x, y, z, v = struct.unpack('<ffff', f_data[offset:offset+16])
                            pts.append({"x": x, "y": y, "z": z, "v": v})
                            offset += 16
                    else: offset += t_len
                
                latest_radar_frame = {"frame_num": f_num, "points": pts}
                sys.stdout.write(f"\r[327 Live] 帧: {f_num} | 点数: {len(pts)}   ")
                sys.stdout.flush()
    except Exception as e:
        print(f"💥 数据链路中断: {e}")

async def handle_ws(websocket):
    print("📱 客户端已连接!")
    last_sent_frame = -1
    
    async def receive_commands():
        async for msg in websocket:
            try:
                cmd = json.loads(msg)
                ctype = cmd.get("type")
                config_dir = "/home/pi/Config"
                if not os.path.exists(config_dir): os.makedirs(config_dir)

                if ctype == "list_configs":
                    files = [f for f in os.listdir(config_dir) if f.endswith('.cfg')]
                    await websocket.send(json.dumps({"type": "config_list", "files": files}))
                
                elif ctype == "read_config":
                    fname = cmd.get("filename")
                    fpath = os.path.join(config_dir, fname)
                    if os.path.exists(fpath):
                        with open(fpath, 'r', encoding='utf-8') as f:
                            content = f.read()
                        await websocket.send(json.dumps({"type": "config_content", "filename": fname, "content": content}))
                
                elif ctype == "apply_config":
                    fname = cmd.get("filename")
                    fpath = os.path.join(config_dir, fname)
                    print(f"\n⚙️ 远程应用配置: {fname}")
                    success = send_config(ARGS.cfg_port, fpath)
                    await websocket.send(json.dumps({"type": "apply_status", "success": success}))
                
                elif ctype == "save_config":
                    fname, content = cmd.get("filename"), cmd.get("content")
                    fpath = os.path.join(config_dir, fname)
                    with open(fpath, 'w', encoding='utf-8') as f: f.write(content)
                    print(f"📁 已保存配置: {fname}")
                    await websocket.send(json.dumps({"type": "save_status", "success": True, "message": f"已存入: {fname}"}))
            except Exception as e:
                print(f"\n⚠️ 指令解析异常: {e}")

    async def stream_data():
        nonlocal last_sent_frame
        while True:
            try:
                if latest_radar_frame["frame_num"] != last_sent_frame:
                    await websocket.send(json.dumps(latest_radar_frame))
                    last_sent_frame = latest_radar_frame["frame_num"]
                await asyncio.sleep(0.04) 
            except: break

    await asyncio.gather(receive_commands(), stream_data())

async def main(args):
    global ARGS
    ARGS = args
    if args.cfg_file:
        send_config(args.cfg_port, args.cfg_file)
    
    threading.Thread(target=radar_thread, args=(args.data_port, args.baud), daemon=True).start()
    
    print(f"📡 WebSocket 服务已就绪: ws://0.0.0.0:{args.ws_port}")
    async with websockets.serve(handle_ws, "0.0.0.0", args.ws_port):
        await asyncio.Future()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--cfg_port', default='/dev/ttyACM0')
    parser.add_argument('--data_port', default='/dev/ttyACM1')
    parser.add_argument('--baud', type=int, default=3125000)
    parser.add_argument('--cfg_file', default='')
    parser.add_argument('--ws_port', type=int, default=8765)
    args = parser.parse_args()
    
    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        print("\n中枢程序已退出。")
