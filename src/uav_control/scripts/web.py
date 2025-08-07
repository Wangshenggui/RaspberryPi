# -*- coding:utf-8 -*-
import socket
import subprocess
import os
import signal
import threading
import time
from flask import Flask, request, jsonify, render_template
import rospy
from std_msgs.msg import Float32MultiArray, String  # 导入 String 消息类型
import json
import mmap
import logging


class FlaskApp:
    def __init__(self, host='0.0.0.0', port=8000):
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)

        self.filepath1 = "/dev/shm/shared_memory1"
        self.filepath2 = "/dev/shm/shared_memory2"
        self.max_size = 1024

        self.app = Flask(__name__)
        self.host = host
        self.port = port
        self._register_routes()


    def _register_routes(self):
        """注册 Flask 路由"""
        
        # 根路由，渲染 index.html
        @self.app.route("/", methods=["GET"])
        def index():
            return render_template("index.html")

        # 处理接收数据的 POST 请求
        @self.app.route("/client_to_server", methods=["POST"])
        def client_to_server():
            try:
                # 从请求中获取 JSON 数据
                # data = request.data.decode('utf-8')

                data = request.data.decode('utf-8')
                json_data = json.loads(data)

                # 将接收到的数据写入共享内存
                with open(self.filepath1, "r+b") as f:
                    # 创建内存映射对象，允许写入
                    mm = mmap.mmap(f.fileno(), self.max_size, access=mmap.ACCESS_WRITE)

                    # 将接收到的数据转换为字符串并确保它在内存中以 null 字符终止
                    string_data = json.dumps(json_data)  # 将字典转为 JSON 字符串
                    mm[:len(string_data)] = string_data.encode('utf-8')  # 写入数据到共享内存
                    mm[len(string_data):len(string_data) + 1] = b'\x00'  # 添加 null 字符确保字符串结束

                # 返回响应
                return jsonify({"message": "Data received and updated successfully", "received": data}), 200

            except Exception as e:
                print("error: ", str(e))
                return jsonify({"message": "Data receiving failure", "error": str(e)}), 400

        
        # 新增的路由：获取后端数据
        @self.app.route("/server_to_client", methods=["GET"])
        def server_to_client():
            with open(self.filepath2, "rb") as f:
                # 创建只读的内存映射对象
                mm = mmap.mmap(f.fileno(), self.max_size, access=mmap.ACCESS_READ)

                # 读取到 null 字符为止
                raw_data = bytearray()
                while True:
                    byte = mm.read(1)
                    if byte == b'\x00' or byte == b'':
                        break
                    raw_data.extend(byte)

                # 尝试以 UTF-8 解码打印
                try:
                    # print("读取到的数据（字符串形式）:")
                    # print(raw_data.decode('utf-8'))
                    return raw_data.decode('utf-8')
                except UnicodeDecodeError:
                    print("无法以 UTF-8 解码，原始数据如下:")
                    print(raw_data)

    def run(self):
        """运行 Flask 应用"""
        try:
            self.app.run(host=self.host, port=self.port)
        except Exception as e:
            print("Flask 启动失败: {}",str(e))

    def run_in_thread(self):
        """在线程中启动 Flask 应用"""
        thread = threading.Thread(target=self.run)
        thread.daemon = True  # 设置为守护线程，程序退出时自动结束
        thread.start()

def check_port_in_use(port):
    """检查端口是否被占用"""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        result = s.connect_ex(('127.0.0.1', port))
        return result == 0
    finally:
        s.close()

def kill_process_using_port(port):
    """根据端口号终止占用该端口的进程 (Linux/macOS/Windows)"""
    # Windows
    if os.name == 'nt':
        cmd = 'netstat -ano | findstr :{}'.format(port)
        result = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout, stderr = result.communicate()
        if stdout:
            pid = stdout.strip().split()[-1]
            subprocess.call('taskkill /PID {} /F'.format(pid), shell=True)
            print("Windows: 进程 {} 已结束".format(pid))
    else:
        # Linux/macOS
        cmd = 'lsof -t -i :{}'.format(port)
        result = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout, stderr = result.communicate()
        if stdout:
            pid = stdout.strip()
            os.kill(int(pid), signal.SIGKILL)
            print("Linux/macOS: 进程 {} 已结束".format(pid))

def kill_web_py_process():
    """杀死 /home/ggb/catkin_ws/src/uav_control/scripts/web.py 的进程"""
    try:
        # 使用 pgrep 查找进程ID
        result = subprocess.Popen(['pgrep', '-f', '/home/ggb/catkin_ws/src/uav_control/scripts/web.py'], 
                                  stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout, stderr = result.communicate()
        
        # 如果找到了进程ID，stdout 返回的是多个PID，按行分割
        if stdout:
            pids = stdout.decode().splitlines()  # 获取多个PID
            for pid in pids:
                pid = int(pid)  # 将每个进程ID转换为整数
                os.kill(pid, signal.SIGKILL)
                print("已结束进程 {}，即 /home/ggb/catkin_ws/src/uav_control/scripts/web.py".format(pid))
        else:
            print("没有找到正在运行的 web.py 进程")
    except Exception as e:
        print("无法结束进程: ", str(e))

if __name__ == "__main__":
    # kill_web_py_process()

    port = 8000
    
    # 检查端口是否被占用
    if check_port_in_use(port):
        print("端口 {} 被占用，准备结束该进程...".format(port))
        kill_process_using_port(port)
        print("端口 {} 已释放，准备启动 Flask 应用".format(port))
    else:
        print("端口 {} 未被占用，直接启动 Flask 应用".format(port))
    
    # 创建 FlaskApp 实例并在单独的线程中运行
    flask_app = FlaskApp(host='0.0.0.0', port=port)
    flask_app.run_in_thread()

    # 在主线程中等待 Ctrl+C 信号来终止程序
    def signal_handler(sig, frame):
        print('程序被中断，正在关闭...')
        os._exit(0)  # 强制退出程序

    signal.signal(signal.SIGINT, signal_handler)  # 捕获 Ctrl+C 信号

    # 可以在这里运行其他代码，Flask 将继续在后台运行
    try:
        while True:
            print("Flask 应用正在后台运行...")
            time.sleep(5)  # 主线程空闲等待
    except KeyboardInterrupt:
        print("程序已被中断")
        # 关闭内存映射
        mm.close()
        os._exit(0)  # 结束程序
