import serial
import time
import struct
import argparse
import sys
import json
import threading
import asyncio
import websockets

# TI 毫米波雷达的“魔法词”
MAGIC_WORD = b'\x02\x01\x04\x03\x06\x05\x08\x07'

# 共享变量，存储最新一帧的结构化数据，供 WebSockets 广播
latest_radar_frame = {
    "frame_num": 0,
    "points": [],  # Type 1 散点云 [{x, y, v}, ...]
    "targets": []  # Type 7 目标轨迹 [{id, x, y, vx, vy}, ...]
}

def send_config_to_radar(cfg_port_name, config_file_path):
    print(f"[配置] 正在打开串口 {cfg_port_name} (115200)...")
    try:
        cfg_port = serial.Serial(cfg_port_name, 115200, timeout=0.1)
    except Exception as e:
        print(f"❌ 无法打开配置串口: {e}")
        return False

    try:
        with open(config_file_path, 'r') as f:
            lines = f.readlines()
            
        print(f"[配置] 开始下发 {len(lines)} 行指令...")
        for line in lines:
            line = line.strip()
            if line == "" or line.startswith("%"): continue
            
            cfg_port.write((line + '\r').encode('utf-8'))
            time.sleep(0.01)
            
            while cfg_port.in_waiting > 0:
                try: cfg_port.readline().decode('utf-8')
                except: pass
                
        print("✅ 雷达配置下发完毕! 传感器启动。\n")
        time.sleep(0.5) 
        cfg_port.close()
        return True
    except Exception as e:
        print(f"❌ 下发配置报错: {e}")
        cfg_port.close()
        return False

def radar_serial_thread(data_port_name, baud_rate, log_file=""):
    """
    负责在后台死循环读取雷达串口，并解析出坐标点，塞进全局变量中
    """
    global latest_radar_frame
    
    log_fd = None
    if log_file:
        try:
            if log_file.endswith('.bin') or log_file.endswith('.dat'):
                log_fd = open(log_file, 'ab')
                print(f"📁 开启无损 RAW BINARY 记录！雷达物理层字节流将实时追加至: {log_file}")
            else:
                log_fd = open(log_file, 'a', encoding='utf-8')
                print(f"📁 开启高精 JSONL 记录！解析后点云将实时追加至: {log_file}")
        except Exception as e:
            print(f"❌ 无法创建日志文件 {log_file}: {e}")

    print(f"[接收] 尝试连接数据串口 {data_port_name} ({baud_rate})...")
    try:
        data_port = serial.Serial(data_port_name, baud_rate, timeout=1)
        # 【极其重要】：打开串口后立刻清空操作系统底层的旧缓存！
        # 否则会读取到重启前雷达遗留的一半残缺帧，导致解包逻辑产生永久性死锁！
        data_port.reset_input_buffer()
        print("✅ 成功监听雷达数据流，解析引擎开启...\n")
    except Exception as e:
        print(f"❌ 无法打开数据串口: {e}")
        return

    buffer = bytearray()
    
    while True:
        try:
            buffer.extend(data_port.read(data_port.in_waiting or 1))
            magic_idx = buffer.find(MAGIC_WORD)
            if magic_idx == -1:
                if len(buffer) > 2048: buffer = buffer[-2048:]
                continue
            if magic_idx > 0: buffer = buffer[magic_idx:]
            
            if len(buffer) < 40: continue # 等待足够长的头部数据
                
            try:
                header_data = struct.unpack('<8sIIIIIIII', buffer[:40])
                magic, version, total_packet_len, platform, frame_num, time_cpu, num_obj, num_tlvs, sub_frame_num = header_data
            except struct.error:
                continue
            
            # 【核心防死锁】：如果解析出的包长异常(大于1MB或小于40)，说明发生严重错位
            # 必须立刻切除掉现在的魔法字，强迫进入下一次寻找，否则陷入无限等待死锁！
            if total_packet_len > 1000000 or total_packet_len < 40:
                buffer = buffer[8:]
                continue
                
            if len(buffer) < total_packet_len:
                continue
            
            # 若日志需求为纯原始字节流 (.bin / .dat)，在这里无损落盘
            if log_fd and (log_file.endswith('.bin') or log_file.endswith('.dat')):
                log_fd.write(buffer[:total_packet_len])
                log_fd.flush()
            
            frame_data = buffer[40:total_packet_len]
            buffer = buffer[total_packet_len:]
            
            # 准备装载当前帧的新数据
            current_frame = {
                "frame_num": frame_num,
                "points": [],
                "targets": []
            }
            
            offset = 0
            for i in range(num_tlvs):
                if offset + 8 > len(frame_data): break
                tlv_type, tlv_length = struct.unpack('<II', frame_data[offset:offset+8])
                offset += 8
                tlv_payload = frame_data[offset:offset+tlv_length]
                offset += tlv_length
                
                # Type 1: 点云 Point Cloud (每个点 16 字节)
                if tlv_type == 1 and tlv_length >= 16:
                    point_count = tlv_length // 16
                    for p in range(min(num_obj, point_count)):
                        x, y, z, v = struct.unpack('<ffff', tlv_payload[p*16:(p+1)*16])
                        # 传输无损满精度！去除之前的 round(x, 2) 导致算法无法精准分析微小位移和多普勒
                        current_frame["points"].append({"x": x, "y": y, "z": z, "v": v})
                        
                # 屏蔽这部分，因为发现 AWR2944 当前固件输出的 Type 7 根本不是 40 bytes 的追踪目标 (可能只是区域驻留检测 Zone Presence)
                # 导致它把 40 字节解析成了各种 id 为 63303901、坐标全为 0 的幽灵数据！
                elif tlv_type == 7:
                    pass


            # 更新给全局变量，供 WebSocket 推流
            latest_radar_frame = current_frame
            
            # 若开启了录制，安全落盘 (使用 JSONL 格式，即每行一个 JSON)
            if log_fd and not (log_file.endswith('.bin') or log_file.endswith('.dat')) and len(current_frame["points"]) > 0:
                current_frame["sys_time"] = time.time()
                log_fd.write(json.dumps(current_frame) + "\n")
                log_fd.flush()

            sys.stdout.write(f"\r⚡ [直播中] 帧号: {frame_num} | 散点数: {len(current_frame['points'])} | 锁定目标数: {len(current_frame['targets'])}")
            sys.stdout.flush()
            
        except Exception as e:
            pass # 屏蔽由于粘包导致的偶然崩溃，保持守护线程坚挺


# WebSocket 广播服务器逻辑
async def broadcast_radar_data(websocket):
    print(f"\n📱 手机/前端 客户端已建立连接! 正在疯狂推流 (20Hz)...")
    last_sent_frame = -1
    try:
        while True:
            # 高频查询，如果有新帧则推动
            if latest_radar_frame["frame_num"] != last_sent_frame:
                await websocket.send(json.dumps(latest_radar_frame))
                last_sent_frame = latest_radar_frame["frame_num"]
            # 控制查询频率约 30Hz，略快于雷达的 20Hz 保证不漏帧
            await asyncio.sleep(0.03)
    except websockets.exceptions.ConnectionClosed:
        print("\n📱 客户端断开了连接。等待下一个客户端...")


async def main_ws_server(port):
    print(f"\n🌐 启动 WebSocket 服务器: ws://0.0.0.0:{port} ...")
    # asyncio.get_event_loop().run_until_complete 的现代写法
    async with websockets.serve(broadcast_radar_data, "0.0.0.0", port):
        await asyncio.Future()  # 永远运行不要停止


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="雷达【硬核版服务器】 - 解析与WebSocket多端推流")
    parser.add_argument('--cfg_port', type=str, default='/dev/ttyACM0')
    parser.add_argument('--data_port', type=str, default='/dev/ttyACM1')
    parser.add_argument('--baud', type=int, default=3125000)
    parser.add_argument('--cfg_file', type=str, default='')
    parser.add_argument('--ws_port', type=int, default=8765, help='WebSocket 广播端口')
    parser.add_argument('--log_file', type=str, default='', help='保存雷达点云日志的文件路径')
    args = parser.parse_args()
    
    # 初始化雷达
    if args.cfg_file:
        success = send_config_to_radar(args.cfg_port, args.cfg_file)
        if not success: sys.exit(1)
            
    # 开一个纯洁无瑕的后台线程去死磕串口 (不阻塞主线程)
    t = threading.Thread(target=radar_serial_thread, args=(args.data_port, args.baud, args.log_file), daemon=True)
    t.start()
    
    # 主线程开启协程，做 WebSocket 网络广播
    # 如果提示没有 websockets，记得在树莓派 pip3 install websockets
    try:
        asyncio.run(main_ws_server(args.ws_port))
    except KeyboardInterrupt:
        print("\n手动关闭了服务器中枢。")
