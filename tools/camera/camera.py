import os
import subprocess
import threading
import time
import configparser
from datetime import datetime


# ======================================
# 基础路径
# ======================================

BASE_DIR = os.path.dirname(
    os.path.abspath(__file__)
)


CONFIG_FILE = os.path.join(
    BASE_DIR,
    "camera_config.cfg"
)


# ======================================
# 读取配置
# ======================================

config = configparser.ConfigParser()
config.read(CONFIG_FILE)


# Camera

DEVICE = config["camera"]["device"]
INPUT_FORMAT = config["camera"]["input_format"]


# Photo

PHOTO_WIDTH = config["photo"]["width"]
PHOTO_HEIGHT = config["photo"]["height"]


# Video

VIDEO_WIDTH = config["video"]["width"]
VIDEO_HEIGHT = config["video"]["height"]
FPS = config["video"]["fps"]


# Encoder

CODEC = config["encoder"]["codec"]
PRESET = config["encoder"]["preset"]
TUNE = config["encoder"]["tune"]
PROFILE = config["encoder"]["profile"]
BITRATE = config["encoder"]["bitrate"]


# x264

GOP = config["x264"]["gop"]
KEYINT_MIN = config["x264"]["keyint_min"]
X264_PARAMS = config["x264"]["params"]


# Runtime

STATUS_INTERVAL = int(
    config["runtime"]["status_interval"]
)

FFMPEG_LOG = config["runtime"]["ffmpeg_log"]



# ======================================
# 输出目录
# ======================================

OUTPUT_DIR = os.path.join(
    BASE_DIR,
    config["path"]["output"]
)


PHOTO_DIR = os.path.join(
    OUTPUT_DIR,
    config["path"]["photo_dir"]
)


VIDEO_DIR = os.path.join(
    OUTPUT_DIR,
    config["path"]["video_dir"]
)


os.makedirs(
    PHOTO_DIR,
    exist_ok=True
)


os.makedirs(
    VIDEO_DIR,
    exist_ok=True
)



# ======================================
# 全局状态
# ======================================

video_process = None

video_start_time = None

video_filename = None

status_thread = None

status_running = False



# ======================================
# 时间格式
# ======================================

def format_time(seconds):

    h = int(seconds // 3600)

    m = int(
        (seconds % 3600) // 60
    )

    s = int(
        seconds % 60
    )

    return (
        f"{h:02d}:{m:02d}:{s:02d}"
    )



# ======================================
# ffmpeg日志
# ======================================

def ffmpeg_output():

    if FFMPEG_LOG.lower() == "true":

        return None

    else:

        return subprocess.DEVNULL



# ======================================
# 查看拍摄规格
# ======================================

def print_config():


    print()

    print("===================")
    print("当前拍摄规格")
    print("===================")


    print()

    print("照片:")

    print(
        f" 分辨率 : {PHOTO_WIDTH}x{PHOTO_HEIGHT}"
    )

    print(
        " 格式   : JPEG"
    )

    print(
        f" 来源   : {INPUT_FORMAT}"
    )



    print()

    print("录像:")

    print(
        f" 分辨率 : {VIDEO_WIDTH}x{VIDEO_HEIGHT}"
    )

    print(
        f" 帧率   : {FPS} FPS"
    )

    print(
        f" 输入   : {INPUT_FORMAT}"
    )


    print()

    print("编码:")

    print(
        f" 编码器 : {CODEC}"
    )

    print(
        f" preset : {PRESET}"
    )

    print(
        f" tune   : {TUNE}"
    )

    print(
        f" profile: {PROFILE}"
    )


    print()

    print("参数:")

    print(
        f" bitrate: {BITRATE}"
    )

    print(
        f" GOP    : {GOP}"
    )

    print(
        f" keyint : {KEYINT_MIN}"
    )

    print(
        f" x264   : {X264_PARAMS}"
    )

    print()



# ======================================
# 录像状态提示线程
# ======================================

def record_monitor():

    global status_running


    while status_running:


        time.sleep(
            STATUS_INTERVAL
        )


        if not status_running:

            break


        if video_process:


            if video_process.poll() is None:


                elapsed = (
                    time.time()
                    -
                    video_start_time
                )


                print()

                print(
                    f"正在录制视频中: "
                    f"{int(elapsed)} s"
                )


            else:

                print()

                print(
                    "ffmpeg异常退出"
                )

                break



# ======================================
# 开始录像
# ======================================

def start_video():

    global video_process
    global video_start_time
    global video_filename
    global status_thread
    global status_running



    if video_process and video_process.poll() is None:

        print(
            "当前已经在录像"
        )

        return



    filename = datetime.now().strftime(
        "%Y%m%d_%H%M%S.mp4"
    )


    output_file = os.path.join(
        VIDEO_DIR,
        filename
    )



    cmd = [

        "ffmpeg",

        "-f",
        "v4l2",

        "-input_format",
        INPUT_FORMAT,

        "-video_size",
        f"{VIDEO_WIDTH}x{VIDEO_HEIGHT}",

        "-framerate",
        FPS,

        "-i",
        DEVICE,


        "-vf",
        "format=yuv420p",


        "-c:v",
        CODEC,


        "-preset",
        PRESET,


        "-tune",
        TUNE,


        "-profile:v",
        PROFILE,


        "-b:v",
        BITRATE,


        "-g",
        GOP,


        "-keyint_min",
        KEYINT_MIN,


        "-x264-params",
        X264_PARAMS,


        output_file

    ]



    print()

    print(
        "开始录像:"
    )

    print(
        output_file
    )



    video_process = subprocess.Popen(

        cmd,

        stdin=subprocess.PIPE,

        stdout=ffmpeg_output(),

        stderr=ffmpeg_output()

    )



    video_filename = output_file

    video_start_time = time.time()


    status_running = True


    status_thread = threading.Thread(

        target=record_monitor,

        daemon=True

    )


    status_thread.start()




# ======================================
# 停止录像
# ======================================

def stop_video():

    global video_process
    global status_running



    if video_process is None:

        print(
            "当前没有录像"
        )

        return



    print()

    print(
        "正在停止录像..."
    )


    status_running = False



    try:

        video_process.stdin.write(
            b"q\n"
        )

        video_process.stdin.flush()


        video_process.wait(
            timeout=5
        )


    except Exception:

        video_process.kill()



    elapsed = (
        time.time()
        -
        video_start_time
    )


    video_process = None


    print()

    print(
        "录像结束"
    )


    print(
        "录像时间:",
        format_time(elapsed)
    )

    print()




# ======================================
# 拍照
# ======================================

def take_photo():


    filename = datetime.now().strftime(
        "%Y%m%d_%H%M%S.jpg"
    )


    photo_file = os.path.join(
        PHOTO_DIR,
        filename
    )



    cmd = [

        "ffmpeg",

        "-f",
        "v4l2",

        "-input_format",
        INPUT_FORMAT,


        "-video_size",
        f"{PHOTO_WIDTH}x{PHOTO_HEIGHT}",


        "-i",
        DEVICE,


        "-frames:v",
        "1",


        photo_file

    ]



    subprocess.run(

        cmd,

        stdout=ffmpeg_output(),

        stderr=ffmpeg_output()

    )



    print()

    print(
        "照片保存:"
    )

    print(
        photo_file
    )

    print()




# ======================================
# 主程序
# ======================================

def main():


    print(
"""
===================
 Raspberry Pi Camera
===================

1 : 拍照
2 : 开始录像
3 : 停止录像
4 : 查看拍摄规格

q : 退出

"""
    )



    while True:


        try:


            key = input(
                "选择:"
            ).strip()



            if key == "1":

                take_photo()



            elif key == "2":

                start_video()



            elif key == "3":

                stop_video()



            elif key == "4":

                print_config()



            elif key.lower() == "q":


                stop_video()

                break



            else:

                print(
                    "无效输入"
                )



        except KeyboardInterrupt:


            print()

            print(
                "退出程序"
            )


            stop_video()

            break





if __name__ == "__main__":

    main()