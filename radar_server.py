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
import math
from pathlib import Path

from radar_runtime import ConfigPathError, PointCloudRecorder, config_snapshot, resolve_config_path

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
    "points": [],  # Type 1 散点云 [{x, y, z, v, snr, noise}, ...]
    "tlv_types": [],
    "side_info_count": 0,
    "has_side_info": False,
    "targets": []
}

pointcloud_recording = {
    "enabled": False,
    "file": None,
    "writer": None,
    "path": "",
    "rows": 0,
    "last_frame": -1
}

runtime_state = {
    "cfg_port": "",
    "data_port": "",
    "active_config": {"name": None, "sha256": None, "content": None},
}
pointcloud_recorder = PointCloudRecorder(Path(config_dir) / "captures" / "pointcloud_logs")

gimbal_scan = {
    "enabled": False,
    "thread": None,
    "stop_event": None,
    "last_error": "",
    "status": "idle",
    "cycle": 0,
    "endpoint": "",
    "yaw_actual_deg": 0.0,
    "pitch_actual_deg": 0.0,
    "direction": "cw",
    "rpm": 1.0,
    "frame_angle_deg": 3.936,
    "start_time_s": 0.0,
    "combined_points": [],
    "last_update_s": 0.0,
    "config": {
        "mode": "motor",
        "motor_pwm_gpio": 12,
        "motor_dir_gpio": 17,
        "direction": "cw",
        "rpm": 1.0,
        "pwm_duty": 0.25,
        "frame_period_s": 0.656,
        "servo_port": "/dev/ttyUSB0",
        "baud": 115200,
        "pitch_id": 1,
        "yaw_id": 2,
        "pitch_fixed_deg": 0.0,
        "yaw0_deg": 0.0,
        "yaw180_deg": 180.0,
        "move_ms": 500,
        "pause_ms": 250,
        "center_pwm": 1500,
        "min_pwm": 500,
        "max_pwm": 2500,
        "us_per_degree": 2000.0 / 270.0
    }
}

SERVO_POS_RE = None

def clamp_value(value, low, high):
    return max(low, min(high, value))

def servo_deg_to_pwm(deg, center_pwm, us_per_degree, min_pwm, max_pwm):
    return int(round(clamp_value(center_pwm + deg * us_per_degree, min_pwm, max_pwm)))

def servo_pwm_to_deg(pwm, center_pwm, us_per_degree):
    return (pwm - center_pwm) / us_per_degree

def build_servo_move_cmd(servo_id, pwm, move_ms):
    return f"#{int(servo_id):03d}P{int(pwm):04d}T{int(move_ms):04d}!".encode("ascii")

def read_servo_positions(ser, wait_s=0.12):
    global SERVO_POS_RE
    if SERVO_POS_RE is None:
        import re
        SERVO_POS_RE = re.compile(rb"#(\d{3})P(\d{3,4})!")
    try:
        ser.reset_input_buffer()
        ser.write(b"#255PRAD!")
        time.sleep(wait_s)
        data = ser.read_all()
        return {int(sid): int(pos) for sid, pos in SERVO_POS_RE.findall(data)}
    except Exception:
        return {}

def rotate_point_xy(pt, yaw_deg):
    yaw = yaw_deg * 3.141592653589793 / 180.0
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    x = float(pt.get("x", 0.0))
    y = float(pt.get("y", 0.0))
    out = dict(pt)
    out["x"] = cos_y * x - sin_y * y
    out["y"] = sin_y * x + cos_y * y
    out["scan_yaw_deg"] = yaw_deg
    return out

def snapshot_rotated_points(yaw_deg):
    frame = dict(latest_radar_frame)
    return [rotate_point_xy(pt, yaw_deg) for pt in frame.get("points", [])]

def motor_scan_angle_deg():
    if not gimbal_scan.get("start_time_s"):
        return 0.0
    elapsed = max(0.0, time.time() - float(gimbal_scan["start_time_s"]))
    sign = 1.0 if gimbal_scan.get("direction") == "cw" else -1.0
    return (sign * float(gimbal_scan.get("rpm", 0.0)) * 6.0 * elapsed) % 360.0

def motor_scan_loop(config, stop_event):
    gimbal_scan["status"] = "opening"
    gimbal_scan["last_error"] = ""
    pwm = None
    direction = None
    try:
        from gpiozero import DigitalOutputDevice, PWMOutputDevice

        pwm_gpio = int(config["motor_pwm_gpio"])
        dir_gpio = int(config["motor_dir_gpio"])
        rpm = max(0.0, float(config["rpm"]))
        duty = clamp_value(float(config["pwm_duty"]), 0.0, 1.0)
        clockwise = str(config.get("direction", "cw")).lower() != "ccw"

        pwm = PWMOutputDevice(pwm_gpio, frequency=1000)
        direction = DigitalOutputDevice(dir_gpio)

        if clockwise:
            direction.off()
            pwm.value = duty
        else:
            direction.on()
            pwm.value = 1.0 - duty

        gimbal_scan["status"] = "running"
        gimbal_scan["last_update_s"] = time.time()
        logger.info(
            f"Motor scan started: PWM GPIO{pwm_gpio}, DIR GPIO{dir_gpio}, "
            f"direction={'cw' if clockwise else 'ccw'}, rpm={rpm:.2f}, duty={duty:.2f}"
        )

        while not stop_event.is_set():
            gimbal_scan["yaw_actual_deg"] = motor_scan_angle_deg()
            gimbal_scan["last_update_s"] = time.time()
            time.sleep(0.05)

        gimbal_scan["status"] = "stopping"
    except Exception as e:
        gimbal_scan["last_error"] = str(e)
        gimbal_scan["status"] = "error"
        logger.error(f"Motor scan loop error: {e}")
    finally:
        try:
            if pwm:
                pwm.value = 0
                pwm.close()
            if direction:
                direction.off()
                direction.close()
        except Exception:
            pass
        gimbal_scan["enabled"] = False
        if gimbal_scan["status"] != "error":
            gimbal_scan["status"] = "idle"

def gimbal_scan_loop(config, stop_event):
    gimbal_scan["status"] = "opening"
    gimbal_scan["last_error"] = ""
    logger.info(f"Gimbal scan opening servo port {config['servo_port']} baud={config['baud']}")
    try:
        ser = serial.Serial(config["servo_port"], int(config["baud"]), timeout=0.2)
    except Exception as e:
        gimbal_scan["enabled"] = False
        gimbal_scan["status"] = "error"
        gimbal_scan["last_error"] = f"open servo port failed: {e}"
        logger.error(f"Gimbal scan open servo port failed: {e}")
        return

    try:
        pitch_pwm = servo_deg_to_pwm(
            float(config["pitch_fixed_deg"]),
            int(config["center_pwm"]),
            float(config["us_per_degree"]),
            int(config["min_pwm"]),
            int(config["max_pwm"])
        )
        logger.info(
            f"Gimbal scan set pitch ID{int(config['pitch_id']):03d}: "
            f"{float(config['pitch_fixed_deg']):.1f}deg -> P{pitch_pwm}"
        )
        ser.write(build_servo_move_cmd(config["pitch_id"], pitch_pwm, int(config["move_ms"])))
        time.sleep(max(0.05, int(config["move_ms"]) / 1000.0))

        yaw0_pwm = servo_deg_to_pwm(
            float(config["yaw0_deg"]), int(config["center_pwm"]), float(config["us_per_degree"]),
            int(config["min_pwm"]), int(config["max_pwm"])
        )
        yaw180_pwm = servo_deg_to_pwm(
            float(config["yaw180_deg"]), int(config["center_pwm"]), float(config["us_per_degree"]),
            int(config["min_pwm"]), int(config["max_pwm"])
        )

        cycle = 0
        endpoint_points = {"yaw0": [], "yaw180": []}
        while not stop_event.is_set():
            for name, yaw_cmd_deg, yaw_pwm in [
                ("yaw0", float(config["yaw0_deg"]), yaw0_pwm),
                ("yaw180", float(config["yaw180_deg"]), yaw180_pwm),
            ]:
                if stop_event.is_set():
                    break
                gimbal_scan["status"] = "moving"
                gimbal_scan["endpoint"] = name
                logger.info(
                    f"Gimbal scan move yaw ID{int(config['yaw_id']):03d} {name}: "
                    f"{yaw_cmd_deg:.1f}deg -> P{yaw_pwm}, move_ms={int(config['move_ms'])}"
                )
                ser.write(build_servo_move_cmd(config["yaw_id"], yaw_pwm, int(config["move_ms"])))
                time.sleep(max(0.02, int(config["move_ms"]) / 1000.0))

                gimbal_scan["status"] = "settling"
                time.sleep(max(0.0, int(config["pause_ms"]) / 1000.0))

                positions = read_servo_positions(ser)
                yaw_pwm_actual = positions.get(int(config["yaw_id"]), yaw_pwm)
                pitch_pwm_actual = positions.get(int(config["pitch_id"]), pitch_pwm)
                yaw_actual = servo_pwm_to_deg(yaw_pwm_actual, int(config["center_pwm"]), float(config["us_per_degree"]))
                pitch_actual = servo_pwm_to_deg(pitch_pwm_actual, int(config["center_pwm"]), float(config["us_per_degree"]))

                gimbal_scan["status"] = "capturing"
                gimbal_scan["yaw_actual_deg"] = yaw_actual
                gimbal_scan["pitch_actual_deg"] = pitch_actual
                logger.info(
                    f"Gimbal scan actual yaw={yaw_actual:.1f}deg P{yaw_pwm_actual}, "
                    f"pitch={pitch_actual:.1f}deg P{pitch_pwm_actual}, points={len(latest_radar_frame.get('points', []))}"
                )
                endpoint_points[name] = snapshot_rotated_points(yaw_actual)
                combined = endpoint_points["yaw0"] + endpoint_points["yaw180"]
                if combined:
                    gimbal_scan["combined_points"] = combined
                    gimbal_scan["cycle"] = cycle
                    gimbal_scan["last_update_s"] = time.time()
                    gimbal_scan["status"] = "running"

            cycle += 1

        gimbal_scan["status"] = "stopping"
        center_pwm = int(config["center_pwm"])
        try:
            ser.write(build_servo_move_cmd(config["yaw_id"], center_pwm, int(config["move_ms"])))
            ser.write(build_servo_move_cmd(config["pitch_id"], pitch_pwm, int(config["move_ms"])))
            time.sleep(max(0.05, int(config["move_ms"]) / 1000.0))
        except Exception:
            pass
    except Exception as e:
        gimbal_scan["last_error"] = str(e)
        gimbal_scan["status"] = "error"
        logger.error(f"Gimbal scan loop error: {e}")
    finally:
        try:
            ser.close()
        except Exception:
            pass
        gimbal_scan["enabled"] = False
        if gimbal_scan["status"] != "error":
            gimbal_scan["status"] = "idle"

def start_gimbal_scan(params):
    if gimbal_scan["enabled"]:
        return False, "gimbal scan already running"

    config = dict(gimbal_scan["config"])
    for key in config:
        if key in params:
            config[key] = params[key]

    numeric_int_keys = ["baud", "pitch_id", "yaw_id", "move_ms", "pause_ms", "center_pwm", "min_pwm", "max_pwm", "motor_pwm_gpio", "motor_dir_gpio"]
    numeric_float_keys = ["pitch_fixed_deg", "yaw0_deg", "yaw180_deg", "us_per_degree", "rpm", "pwm_duty", "frame_period_s"]
    for key in numeric_int_keys:
        config[key] = int(float(config[key]))
    for key in numeric_float_keys:
        config[key] = float(config[key])

    stop_event = threading.Event()
    frame_angle_deg = abs(float(config["rpm"]) * 6.0 * float(config["frame_period_s"]))
    if str(config.get("mode", "motor")).lower() == "servo":
        thread_target = gimbal_scan_loop
    else:
        config["mode"] = "motor"
        thread_target = motor_scan_loop
    thread = threading.Thread(target=thread_target, args=(config, stop_event), daemon=True)
    gimbal_scan.update({
        "enabled": True,
        "thread": thread,
        "stop_event": stop_event,
        "config": config,
        "last_error": "",
        "status": "starting",
        "direction": str(config.get("direction", "cw")).lower(),
        "rpm": float(config["rpm"]),
        "frame_angle_deg": frame_angle_deg,
        "start_time_s": time.time(),
        "yaw_actual_deg": 0.0,
        "combined_points": [],
        "last_update_s": 0.0
    })
    thread.start()
    return True, "motor scan started" if config["mode"] == "motor" else "gimbal scan started"

def stop_gimbal_scan():
    ev = gimbal_scan.get("stop_event")
    if ev:
        ev.set()
    return True, "gimbal scan stopping"

def get_gimbal_scan_status():
    return {
        "type": "gimbal_scan_status",
        "enabled": gimbal_scan["enabled"],
        "status": gimbal_scan["status"],
        "cycle": gimbal_scan["cycle"],
        "endpoint": gimbal_scan["endpoint"],
        "yaw_actual_deg": gimbal_scan["yaw_actual_deg"],
        "pitch_actual_deg": gimbal_scan["pitch_actual_deg"],
        "direction": gimbal_scan["direction"],
        "rpm": gimbal_scan["rpm"],
        "frame_angle_deg": gimbal_scan["frame_angle_deg"],
        "points": len(gimbal_scan["combined_points"]),
        "last_update_s": gimbal_scan["last_update_s"],
        "last_error": gimbal_scan["last_error"],
        "config": gimbal_scan["config"]
    }

def start_pointcloud_recording():
    out_path = pointcloud_recorder.start(runtime_state["active_config"])
    _sync_recording_status()
    logger.info(f"📝 点云记录已开启: {out_path}")
    return out_path

def stop_pointcloud_recording():
    status = pointcloud_recorder.stop()
    _sync_recording_status()
    logger.info(f"🛑 点云记录已关闭: {status['path']} rows={status['rows']}")
    return status["path"], status["rows"]

def record_pointcloud_frame(frame, raw_packet=None):
    pointcloud_recorder.record_frame(frame, raw_packet)
    _sync_recording_status()

def get_recording_status():
    _sync_recording_status()
    return pointcloud_recorder.status()


def _sync_recording_status():
    status = pointcloud_recorder.status()
    pointcloud_recording.update({
        "enabled": status["enabled"],
        "path": status["path"],
        "rows": status["rows"],
    })

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
    stats = {"count": 0, "last": time.time(), "last_tlv": time.time()}
    MAX_PACKET_LEN = 1024 * 1024

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
                    p_len, f_num, cpu_cycles, detected_objects, n_tlvs = (
                        h_data[2], h_data[4], h_data[5], h_data[6], h_data[7]
                    )
                except: 
                    buffer = buffer[1:] # 错开一位继续找
                    continue

                # 【防死循环终极护甲】：绝不能允许 p_len 小于最小帧头（40），否则若 p_len=0 会导致 buffer 永远不缩减！
                if p_len < 40:
                    buffer = buffer[8:] # 这是一个伪装的破损头部，跳过魔数继续往后搜！
                    continue
                if p_len > MAX_PACKET_LEN:
                    logger.warning(f"⚠️ 异常包长 {p_len}，疑似破损帧或非预期TLV，丢弃当前魔法字后继续同步")
                    buffer = buffer[8:]
                    continue

                if len(buffer) < p_len: break # 这一帧还没吐完，保留现场，去等喂饭
                
                # 开始大解包！
                raw_packet = bytes(buffer[:p_len])
                f_data = buffer[40:p_len]
                buffer = buffer[p_len:] # 💥核 心：切下一块肉，必须马上吞掉！
                progress_made = True
                
                pts = []
                point_side_info = []
                range_profile = []
                tlv_types = []
                offset = 0
                for _ in range(n_tlvs):
                    try:
                        t_type, t_len = struct.unpack('<II', f_data[offset:offset+8])
                        tlv_types.append(t_type)
                        offset += 8
                        if t_type == 1:
                            p_cnt = t_len // 16
                            for p in range(p_cnt):
                                x, y, z, v = struct.unpack('<ffff', f_data[offset:offset+16])
                                pts.append({"x": x, "y": y, "z": z, "v": v})
                                offset += 16
                        elif t_type == 2:
                            sample_cnt = t_len // 2
                            # Range Profile TLV: one uint16 magnitude per range bin.
                            fmt = '<' + ('H' * sample_cnt)
                            profile_vals = struct.unpack(fmt, f_data[offset:offset + t_len])
                            range_profile = list(profile_vals)
                            offset += t_len
                        elif t_type == 7:
                            # Detected Points Side Info TLV: one int16 SNR + int16 noise per point.
                            # TI demos commonly encode both values in 0.1 dB units.
                            side_cnt = t_len // 4
                            for p in range(side_cnt):
                                snr_raw, noise_raw = struct.unpack('<hh', f_data[offset:offset+4])
                                point_side_info.append({
                                    "snr": snr_raw / 10.0,
                                    "noise": noise_raw / 10.0
                                })
                                offset += 4
                        else: offset += t_len
                    except: 
                        break # TLV 碎了，但这帧算看过了，不影响继续

                for i, side in enumerate(point_side_info):
                    if i < len(pts):
                        pts[i].update(side)
                
                latest_radar_frame = {
                    "frame_num": f_num,
                    "host_time_s": time.time(),
                    "host_monotonic_s": time.monotonic(),
                    "device_time_cpu_cycles": cpu_cycles,
                    "detected_object_count": detected_objects,
                    "points": pts,
                    "range_profile": range_profile,
                    "tlv_types": tlv_types,
                    "side_info_count": len(point_side_info),
                    "has_side_info": len(point_side_info) > 0
                }
                record_pointcloud_frame(latest_radar_frame, raw_packet)
                if time.time() - stats["last_tlv"] > 5:
                    logger.info(
                        f"📦 [TLV] frame={f_num} types={tlv_types} "
                        f"points={len(pts)} sideInfo={len(point_side_info)}"
                    )
                    stats["last_tlv"] = time.time()
                
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
                    profiles_dir = Path(config_dir) / "Config"
                    
                    if ctype == "list_configs":
                        profiles_dir.mkdir(parents=True, exist_ok=True)
                        files = sorted(path.name for path in profiles_dir.glob("*.cfg") if path.is_file())
                        await websocket.send(json.dumps({"type": "config_list", "files": files}))
                    
                    elif ctype == "read_config":
                        fname = cmd.get("filename")
                        try:
                            fpath = resolve_config_path(profiles_dir, fname, must_exist=True)
                            content = fpath.read_text(encoding="utf-8")
                            await websocket.send(json.dumps({"type": "config_content", "filename": fpath.name, "content": content}))
                        except (ConfigPathError, OSError, UnicodeError, TypeError) as exc:
                            await websocket.send(json.dumps({"type": "config_error", "message": str(exc)}))
                    
                    elif ctype == "save_config":
                        fname = cmd.get("filename")
                        content = cmd.get("content")
                        try:
                            if content is None:
                                raise ConfigPathError("配置内容不能为空")
                            fpath = resolve_config_path(profiles_dir, fname)
                            fpath.write_text(content, encoding="utf-8")
                            await websocket.send(json.dumps({"type": "save_status", "success": True, "message": f"已存入 Config 目录: {fpath.name}"}))
                        except (ConfigPathError, OSError, UnicodeError, TypeError) as exc:
                            await websocket.send(json.dumps({"type": "save_status", "success": False, "message": str(exc)}))
                    
                    elif ctype == "apply_config":
                        fname = cmd.get("filename")
                        content = cmd.get("content")
                        try:
                            fpath = resolve_config_path(profiles_dir, fname)
                            if content is not None:
                                fpath.write_text(content, encoding="utf-8")
                            fpath = resolve_config_path(profiles_dir, fname, must_exist=True)
                            logger.info(f"⚙️ 正在应用 Config 下的动态配置: {fpath}")
                            success = send_config_to_radar(runtime_state["cfg_port"], str(fpath))
                            if success:
                                runtime_state["active_config"] = config_snapshot(fpath)
                            await websocket.send(json.dumps({"type": "apply_status", "success": success}))
                        except (ConfigPathError, OSError, UnicodeError, TypeError) as exc:
                            logger.warning(f"⚠️ 配置请求被拒绝: {exc}")
                            await websocket.send(json.dumps({"type": "apply_status", "success": False, "message": str(exc)}))

                    elif ctype == "pointcloud_record":
                        action = cmd.get("action")
                        if action == "start":
                            path = start_pointcloud_recording()
                            status = get_recording_status()
                            await websocket.send(json.dumps({
                                "type": "pointcloud_record_status",
                                "recording": True,
                                "path": path,
                                "rows": status["rows"],
                                "frames": status["frames"]
                            }))
                        elif action == "stop":
                            path, rows = stop_pointcloud_recording()
                            await websocket.send(json.dumps({
                                "type": "pointcloud_record_status",
                                "recording": False,
                                "path": path,
                                "rows": rows
                            }))
                        elif action == "status":
                            status = get_recording_status()
                            status["type"] = "pointcloud_record_status"
                            status["recording"] = status["enabled"]
                            await websocket.send(json.dumps(status))
                    elif ctype == "gimbal_scan":
                        action = cmd.get("action")
                        params = cmd.get("params") or {}
                        if action == "start":
                            ok, message = start_gimbal_scan(params)
                            payload = get_gimbal_scan_status()
                            payload.update({"success": ok, "message": message})
                            await websocket.send(json.dumps(payload))
                        elif action == "stop":
                            ok, message = stop_gimbal_scan()
                            payload = get_gimbal_scan_status()
                            payload.update({"success": ok, "message": message})
                            await websocket.send(json.dumps(payload))
                        elif action == "status":
                            await websocket.send(json.dumps(get_gimbal_scan_status()))

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
                        frame = dict(latest_radar_frame)
                        frame["pointcloud_recording"] = get_recording_status()
                        if gimbal_scan["enabled"] and gimbal_scan["config"].get("mode") == "motor":
                            yaw_deg = motor_scan_angle_deg()
                            gimbal_scan["yaw_actual_deg"] = yaw_deg
                            gimbal_scan["combined_points"] = [rotate_point_xy(pt, yaw_deg) for pt in frame.get("points", [])]
                            gimbal_scan["last_update_s"] = time.time()
                        frame["gimbal_scan"] = {
                            "enabled": gimbal_scan["enabled"],
                            "status": gimbal_scan["status"],
                            "cycle": gimbal_scan["cycle"],
                            "endpoint": gimbal_scan["endpoint"],
                            "yaw_actual_deg": gimbal_scan["yaw_actual_deg"],
                            "pitch_actual_deg": gimbal_scan["pitch_actual_deg"],
                            "direction": gimbal_scan["direction"],
                            "rpm": gimbal_scan["rpm"],
                            "frame_angle_deg": gimbal_scan["frame_angle_deg"],
                            "points": gimbal_scan["combined_points"],
                            "last_update_s": gimbal_scan["last_update_s"],
                            "last_error": gimbal_scan["last_error"]
                        }
                        await websocket.send(json.dumps(frame))
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
    parser.add_argument('--gimbal_port', type=str, default='/dev/ttyUSB0')
    ARGS = parser.parse_args()
    gimbal_scan["config"]["servo_port"] = ARGS.gimbal_port
    
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
    # WebSocket 配置下发必须使用自动探测后的真实端口，而不是空的启动参数。
    ARGS.cfg_port = real_cfg_port
    ARGS.data_port = real_data_port
    runtime_state["cfg_port"] = real_cfg_port
    runtime_state["data_port"] = real_data_port

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
