import serial
import time
import struct
import argparse
import sys
import json
import threading
import asyncio
import websockets
import logging
import os

# 配置日志：同时输出到文件和控制台
config_dir = os.path.dirname(os.path.abspath(__file__))
log_path = os.path.join(config_dir, "radar_debug.log")

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[
        logging.FileHandler(log_path, encoding='utf-8'),
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger("RadarServer")

# TI 毫米波雷达的“魔法词”
MAGIC_WORD = b'\x02\x01\x04\x03\x06\x05\x08\x07'

# 共享变量，存储最新一帧的结构化数据，供 WebSockets 广播
latest_radar_frame = {
    "frame_num": 0,
    "points": [],  # Type 1 散点云 [{x, y, v}, ...]
    "targets": []  # Type 7 目标轨迹 [{id, x, y, vx, vy}, ...]
}

def auto_detect_ports():
    import glob
    ports = glob.glob('/dev/ttyACM*')
    cfg_port = None
    data_port = None
    
    logger.info(f"🔍 启动暴力版全自动搜寻 (候选: {ports})...")
    for p in ports:
        test_s = None
        try:
            # 探测 115200 是否有 CLI 响应
            test_s = serial.Serial(p, 115200, timeout=1.0)
            # 【暴力唤醒】：快速击发 10 个回车，冲散残留垃圾数据
            for _ in range(10):
                test_s.write(b'\r\n')
                time.sleep(0.02)
                
            time.sleep(0.5)
            if test_s.in_waiting > 0:
                resp = test_s.read(test_s.in_waiting).decode('utf-8', errors='ignore')
                if 'mmwDemo:/>' in resp:
                    cfg_port = p
                    logger.info(f"  [SUCCESS] 强制找回配置口: {p}")
                    # 如果已经找到了，就没必要把缓冲区留给后面，清空它
                    test_s.flushInput()
        except Exception as e:
            logger.debug(f"  [Skip] {p}: {e}")
        finally:
            if test_s:
                try: test_s.close()
                except: pass
    
    if cfg_port:
        remaining = [p for p in ports if p != cfg_port]
        if remaining:
            data_port = remaining[0]
            logger.info(f"  [PROBE] 推测数据口: {data_port}")
            
    return cfg_port, data_port
global_cold_boot = True

def send_config_to_radar(cfg_port_name, config_file_path):
    global global_cold_boot
    if not cfg_port_name:
        logger.error("❌ 找不到雷达配置端口")
        return False
        
    logger.info(f"🚀 [雷达底层重置版] 准备强袭下发配置 -> {cfg_port_name}...")
    try:
        # AWR2944 CLI 建议使用 0.5s 以上的超时
        cfg_port = serial.Serial(cfg_port_name, 115200, timeout=0.5) 
        
        # 【终极救命神技】：执行纯净版“硬件+软件”双修复位 (Hard & Soft Reset)！
        # TI AWR2944 的底层 DFE/通道参数在不掉电的情况下是被物理锁死的（强行覆写报错）。
        # 这里直接让雷达强制归零崩溃状态，把它扇醒！
        logger.info("  [Reset Sequence] 正在强行剥夺底层物理锁，将芯片轰入冷启动态...")
        # 【终极救命神技】：执行纯净版“软复位 (Soft Reboot)”！
        logger.info("  [Reset Sequence] 正在下发软重启指令，将芯片轰入纯净冷启动态...")
        try:
            # 1. 软打断
            cfg_port.write(b'sensorStop\r')
            time.sleep(0.3)
            cfg_port.reset_input_buffer()
            # 2. 软件重启
            cfg_port.write(b'resetDevice\r')
            logger.info("  [Wait Boot] 等待雷达内核重新加载并交出控制权 (最长 8s) ...")
            
            boot_timeout = time.time() + 8.0
            is_awake = False
            full_wake_resp = ""
            while time.time() < boot_timeout:
                cfg_port.write(b'\r')  # 不断敲门
                time.sleep(0.2)
                if cfg_port.in_waiting > 0:
                    resp = cfg_port.read(cfg_port.in_waiting).decode('utf-8', errors='ignore')
                    full_wake_resp += resp
                    if "mmwDemo:" in full_wake_resp or "Texas Instruments" in full_wake_resp:
                        is_awake = True
                        break
                        
            if is_awake:
                logger.info("  [BOOT OK] 雷达固件已苏醒！")
            else:
                logger.warning("  [BOOT WARN] 8秒内未收到明确苏醒旗语，强制放行试试...")
                
            time.sleep(0.5) 
            cfg_port.reset_input_buffer()
        except: pass

    except Exception as e:
        logger.error(f"❌ 无法打开配置口 {cfg_port_name}: {e}")
        return False

    try:
        # 路径健壮性增强：如果直接找不到，去 Config 目录下找
        if not os.path.exists(config_file_path):
            alt_path = os.path.join("/home/pi/Config", os.path.basename(config_file_path))
            if os.path.exists(alt_path):
                config_file_path = alt_path
            else:
                logger.error(f"❌ 配置文件不存在: {config_file_path} (且在 Config 目录下也未找到)")
                return False
                
        with open(config_file_path, 'r', encoding='utf-8') as f:
            lines = f.readlines()

        filtered_lines = []
        for line in lines:
            line_str = line.strip()
            # 由于我们现在有了物理级重启，不需要再搞什么阉割参数的脏活了！全盘照收！
            if line_str == "" or line_str.startswith("%"): continue
            filtered_lines.append(line_str)
            
        global_cold_boot = False # 第一次下发完毕后永远标为False

        # --- 【探测脉冲】：发送 \r\n 唤醒雷达提示符 ---
        logger.info("  [Port Probe] 正在唤醒雷达提示符...")
        cfg_port.write(b'\r\n')
        time.sleep(0.5)
        
        # 清空启动时的残留回响
        if cfg_port.in_waiting > 0:
            resp = cfg_port.read(cfg_port.in_waiting).decode('utf-8', errors='ignore')
            if 'mmwDemo:/>' in resp:
                logger.info("  [OK] 雷达 CLI 握手成功！")
            
        logger.info(f"开始下发 {len(filtered_lines)} 行脚本指令 (来自: {config_file_path})...")
        silence_streak = 0
        
        for line in filtered_lines:
            if line == "" or line.startswith("%"): continue
            
            # 【不可逾越的物理发射冷却】
            # 即便代码运行再快，TI 的底层 RTOS 也必须有几十毫秒的消化时间，否则必死机！
            time.sleep(0.05)
            
            # 【指令提速：设定基础等待】
            timeout_val = 0.3
            if any(cmd in line for cmd in ["calib", "sensorStart", "multiObjBeamForming", "cfarFovCfg", "compRangeBias", "measureRangeBias", "profileCfg"]):
                timeout_val = 1.0
            
            # 【核心护城河】：发射前强制清空上一条可能残留的幽灵回显，杜绝串口串台！
            try: cfg_port.reset_input_buffer()
            except: pass
            
            # 【致命警告】：TI 雷达控制台(CLI)规范终端字符【必须且只能】是回车(\r)！
            # 千万不要改成 \n 或 \r\n！这会导致部分配置命令被跳过执行（返回空），从而引发后续紧跟着的物理参数指令报错！
            cfg_port.write((line + '\r').encode('utf-8'))
            
            # 【快速读取雷达回显】
            prompt_found = False
            start_wait = time.time()
            full_response = ""
            
            while not prompt_found and (time.time() - start_wait < timeout_val):
                if cfg_port.in_waiting > 0:
                    chunk = cfg_port.read(cfg_port.in_waiting).decode('utf-8', errors='ignore')
                    full_response += chunk
                    # 强匹配 mmwDemo:/>
                    if "mmwDemo:" in full_response:
                        prompt_found = True
                        break
                time.sleep(0.01)
            
            # 清理雷达经常回传的回车换行，方便在一行显示
            clean_resp = full_response.replace('\r', '').replace('\n', ' ').strip()
            
            if prompt_found:
                silence_streak = 0
                if "Error" in full_response:
                    logger.error(f"  [Radar] {line} ❌ 失败: {clean_resp}")
                else:
                    logger.info(f"  [Radar] {line} -> ✅ OK")
            else:
                if clean_resp:
                    silence_streak = 0
                    # 虽然没看到明确的结束符，但雷达确实“说话”了，不当做真正的 Timeout
                    logger.info(f"  [Radar] {line} -> ⏳ {clean_resp}")
                else:
                    silence_streak += 1
                    # 极其简短的静默通过，不再大惊小怪报 Warning
                    logger.debug(f"  [Radar] {line} -> (Silent)")
                    if silence_streak == 5:
                        logger.error("🚨 致命异常：雷达芯片连续多次无任何响应！可能是上一条错误的参数导致雷达底层固件崩溃锁死。")
                        logger.error("👉 唯一解法：请【物理拔除并重新插入】雷达的电源/USB连线来重启硬件！")

        logger.info("✅ 雷达握手配置下发流程完毕!")
        time.sleep(0.5) 
        cfg_port.close()
        return True
    except Exception as e:
        logger.error(f"❌ 下发配置过程报错: {e}")
        if 'cfg_port' in locals(): cfg_port.close()
        return False

def radar_serial_thread(data_port_name, baud_rate, log_file=""):
    global latest_radar_frame
    buffer = bytearray()
    stats = {"count": 0, "last": time.time()}

    try:
        ser = serial.Serial(data_port_name, baud_rate, timeout=1)
        ser.reset_input_buffer()
        logger.info(f"✅ [Engine 3.27+] 解析引擎就绪: {data_port_name}")
    except Exception as e:
        logger.error(f"❌ 无法开启数据口: {e}"); return

    while True:
        try:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                buffer.extend(chunk)
                stats["count"] += len(chunk)
                
                if time.time() - stats["last"] > 5:
                    logger.info(f"💓 [LIVE] T:{stats['count']} bytes | Buffer:{len(buffer)}")
                    stats["last"] = time.time()

            # 【无情刮痧机器】：只要手里还有牌，就疯狂把它解包出去直到耗干！
            progress_made = False
            while len(buffer) >= 40:
                m_idx = buffer.find(MAGIC_WORD)
                if m_idx == -1:
                    if len(buffer) > 65536: buffer = buffer[-4096:]
                    break # 连魔数都没有，跳出等喂饭
                if m_idx > 0: buffer = buffer[m_idx:]
                if len(buffer) < 40: break
                
                try:
                    h_data = struct.unpack('<8sIIIIIIII', buffer[:40])
                    p_len, f_num, n_tlvs = h_data[2], h_data[4], h_data[7]
                except: 
                    buffer = buffer[1:] # 错开一位继续找
                    continue

                # 【防死循环终极护甲】：绝不能允许 p_len 小于最小帧头（40），否则若 p_len=0 会导致 buffer 永远不缩减！
                if p_len < 40:
                    buffer = buffer[8:] # 这是一个伪装的破损头部，跳过魔数继续往后搜！
                    continue

                if len(buffer) < p_len: break # 这一帧还没吐完，保留现场，去等喂饭
                
                # 开始大解包！
                f_data = buffer[40:p_len]
                buffer = buffer[p_len:] # 💥核 心：切下一块肉，必须马上吞掉！
                progress_made = True
                
                pts = []
                offset = 0
                for _ in range(n_tlvs):
                    try:
                        t_type, t_len = struct.unpack('<II', f_data[offset:offset+8])
                        offset += 8
                        if t_type == 1:
                            p_cnt = t_len // 16
                            for p in range(p_cnt):
                                x, y, z, v = struct.unpack('<ffff', f_data[offset:offset+16])
                                pts.append({"x": x, "y": y, "z": z, "v": v})
                                offset += 16
                        else: offset += t_len
                    except: 
                        break # TLV 碎了，但这帧算看过了，不影响继续
                
                latest_radar_frame = {"frame_num": f_num, "points": pts}
                
            # 【CPU 退烧药】：如果这一圈下来根本没东西吃，也没有任何成功开包的帧，必须休眠释放 CPU！
            if not progress_made and ser.in_waiting == 0:
                time.sleep(0.005)
                
        except Exception as e:
            err_msg = str(e)
            if "Input/output error" in err_msg or "Errno 5" in err_msg:
                # 物理拔插或设备离线导致的端口句柄失效，不再无限刷屏报错，静默等待重连或重启
                time.sleep(2)
            else:
                logger.error(f"⚠️ 核心引擎跑飞: {e}，正在尝试自愈...")
                time.sleep(1)
            
            # 自愈：清空缓冲区并尝试重新获取端口掌控
            buffer = bytearray()
            try: ser.reset_input_buffer() 
            except: pass

# WebSocket 广播与指令处理逻辑
async def handle_client(websocket):
    logger.info("📱 客户端已建立 Webhook 连接! 正在启动指令同步...")
    last_sent_frame = -1
    
    # 创建一个监听任务用于接收前端发来的指令
    async def receive_commands():
        global ARGS  # 提取全局参数以便动态修改
        try:
            async for message in websocket:
                try:
                    cmd = json.loads(message)
                    ctype = cmd.get("type")
                    
                    if ctype == "list_configs":
                        import os
                        # 设置统一的配置文件存放目录
                        config_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'Config')
                        if not os.path.exists(config_dir): os.makedirs(config_dir)
                        files = [f for f in os.listdir(config_dir) if f.endswith('.cfg')]
                        await websocket.send(json.dumps({"type": "config_list", "files": files}))
                    
                    elif ctype == "read_config":
                        fname = cmd.get("filename")
                        config_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'Config')
                        fpath = os.path.join(config_dir, fname)
                        if fname and os.path.exists(fpath):
                            with open(fpath, 'r', encoding='utf-8') as f:
                                content = f.read()
                            await websocket.send(json.dumps({"type": "config_content", "filename": fname, "content": content}))
                    
                    elif ctype == "save_config":
                        fname = cmd.get("filename")
                        content = cmd.get("content")
                        if fname and content is not None:
                            config_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'Config')
                            if not os.path.exists(config_dir): os.makedirs(config_dir)
                            fpath = os.path.join(config_dir, fname)
                            with open(fpath, 'w', encoding='utf-8') as f:
                                f.write(content)
                            await websocket.send(json.dumps({"type": "save_status", "success": True, "message": f"已存入 Config 目录: {fname}"}))
                    
                    elif ctype == "apply_config":
                        fname = cmd.get("filename")
                        config_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'Config')
                        fpath = os.path.join(config_dir, fname)
                        if fname and os.path.exists(fpath):
                            logger.info(f"⚙️ 正在应用 Config 下的动态配置: {fpath}")
                            success = send_config_to_radar(ARGS.cfg_port, fpath)
                            await websocket.send(json.dumps({"type": "apply_status", "success": success}))
                        else:
                            logger.warn(f"⚠️ 找不到配置文件: {fpath}")
                            await websocket.send(json.dumps({"type": "apply_status", "success": False, "message": "找不到配置文件"}))
                            
                except Exception as e:
                    logger.error(f"❌ 指令处理解析错误: {e}")
        except websockets.exceptions.ConnectionClosed:
            pass

    # 创建一个推流任务
    async def stream_data():
        nonlocal last_sent_frame
        try:
            while True:
                if latest_radar_frame["frame_num"] != last_sent_frame:
                    # 如果不是命令包，则发送雷达点云包
                    try:
                        await websocket.send(json.dumps(latest_radar_frame))
                        last_sent_frame = latest_radar_frame["frame_num"]
                    except: break
                await asyncio.sleep(0.04) # 约 25Hz 推送
        except websockets.exceptions.ConnectionClosed:
            pass

    # 并行执行两个任务
    await asyncio.gather(receive_commands(), stream_data())

async def main_ws_server(port):
    logger.info(f"🌐 远程交互 WebSocket 服务器已就绪: ws://0.0.0.0:{port}")
    async with websockets.serve(handle_client, "0.0.0.0", port):
        await asyncio.Future() 

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="雷达【远程交互版服务器】")
    parser.add_argument('--cfg_port', type=str, default='')
    parser.add_argument('--data_port', type=str, default='')
    parser.add_argument('--baud', type=int, default=3125000)
    parser.add_argument('--cfg_file', type=str, default='')
    parser.add_argument('--ws_port', type=int, default=8765)
    parser.add_argument('--log_file', type=str, default='')
    ARGS = parser.parse_args()
    
    # 【战前清场】：自动猎杀全系统内所有残留的前代 radar_server.py 进程
    import os, signal
    current_pid = os.getpid()
    try:
        pids = [int(p) for p in os.listdir('/proc') if p.isdigit()]
        for pid in pids:
            if pid == current_pid: continue
            try:
                with open(f'/proc/{pid}/cmdline', 'r', encoding='utf-8', errors='ignore') as f:
                    cmdline = f.read().replace('\x00', ' ')
                    if 'radar_server.py' in cmdline and 'python' in cmdline:
                        logger.warning(f"🧹 发现残存前代环境影子 (PID: {pid})，正在强制扫除...")
                        os.kill(pid, signal.SIGKILL)
            except: pass
    except Exception as e:
        logger.debug(f"进程清理跳过: {e}")

    # 【独断专行】：系统级单例锁，杜绝一切多开进程抢夺串口的灵异现象！
    import fcntl
    lock_file = open('/tmp/radar_server.lock', 'w')
    try:
        fcntl.lockf(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except IOError:
        logger.error("🛑 检测到旧的 radar_server 仍在后台运行（可能是 systemd 守护进程或另一终端）！")
        logger.error("👉 为保护雷达串口状态机，本次启动已被自动拦截。请先彻底 kill 前代进程！")
        sys.exit(1)
        
    # 自动探测逻辑集成
    logger.info("⏳ 正在自动识别雷达串口映射...")
    d_cfg, d_data = auto_detect_ports()
    
    import glob
    sys_ports = sorted(glob.glob('/dev/ttyACM*'))
    
    # 智能选择：如果写死的参数不存在，依次降级：
    # 1. 尝试使用全自动探测出的可用端口 (d_cfg / d_data)
    # 2. 如果雷达挂起装死导致探测不到，强制从当前系统中实际插着的 ACM 列表里按顺序认领！
    if ARGS.cfg_port and os.path.exists(ARGS.cfg_port): real_cfg_port = ARGS.cfg_port
    elif d_cfg: real_cfg_port = d_cfg
    else: real_cfg_port = sys_ports[0] if sys_ports else '/dev/ttyACM0'
        
    if ARGS.data_port and os.path.exists(ARGS.data_port): real_data_port = ARGS.data_port
    elif d_data: real_data_port = d_data
    else:
        rem_ports = [p for p in sys_ports if p != real_cfg_port]
        real_data_port = rem_ports[0] if rem_ports else '/dev/ttyACM1'
    
    logger.info(f"📍 最终映射确定: Config={real_cfg_port}, Data={real_data_port}")

    # 等待端口上线且未被占用的自愈容错逻辑
    logger.info(f"⏳ 等待雷达串口连接与就绪 ({real_cfg_port}, {real_data_port})...")
    wait_tic = 0
    while True:
        if os.path.exists(real_cfg_port) and os.path.exists(real_data_port):
            s1 = None
            s2 = None
            try:
                # 试开一下，确保没有被残留的孤儿进程独占（Errno 16）
                s1 = serial.Serial(real_cfg_port, 115200, timeout=0)
                s2 = serial.Serial(real_data_port, ARGS.baud, timeout=0)
                logger.info("✅ 雷达串口已上线并处于空闲状态！")
                break
            except serial.SerialException as e:
                err_msg = str(e)
                if wait_tic == 0:
                    if "Device or resource busy" in err_msg:
                        logger.warning(f"⚠️ 端口被占用 ({err_msg.split(':')[-1].strip()})，持续释放资源并等待控制权...")
                    elif "Permission denied" in err_msg:
                        logger.warning(f"⚠️ 端口权限不足，等待 udev 规则赋予权限...")
            finally:
                if s1: 
                    try: s1.close()
                    except: pass
                if s2: 
                    try: s2.close()
                    except: pass
        wait_tic += 1
        time.sleep(1.0)
    # [强烈修改！] 彻底剥离启动时自动下发配置的逻辑。
    # 为了保护雷达上电后唯一的 channelCfg 设置机会，
    # 强制让所有模式切换权交由网页前端手动掌控。
    if ARGS.cfg_file:
        logger.warning(f"🚧 启动参数屏蔽: 虽收到 {ARGS.cfg_file}，但为保护雷达硬件状态，已禁止后台自动下发。请在网页端手动下发。")

            
    t = threading.Thread(target=radar_serial_thread, args=(real_data_port, ARGS.baud, ARGS.log_file), daemon=True)
    t.start()
    
    try:
        asyncio.run(main_ws_server(ARGS.ws_port))
    except KeyboardInterrupt:
        print("\n中枢已关闭。")
