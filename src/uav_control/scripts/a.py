# -*- coding:utf-8 -*-
import mmap
import time

filepath = "/dev/shm/my_shared_str"
max_size = 256

# 打开共享内存文件进行写入
with open(filepath, "r+b") as f:
    # 创建内存映射对象（访问模式为读写）
    mm = mmap.mmap(f.fileno(), max_size, access=mmap.ACCESS_WRITE)

    # 初始化计数器
    counter = 0

    while True:
        # 创建字符串，包含递增的数字
        string = f"Hello from Python {counter}"
        
        # 将字符串转换为字节并写入共享内存，包含 null 字符（\x00）来终止字符串
        mm[:len(string)] = string.encode('utf-8')
        
        # 为确保数据终止，在字符串后加上 null 字符
        mm[len(string)] = 0
        
        # 输出已写入的数据（此处仅做验证）
        print(f"Python wrote: {string}")
        
        # 递增计数器
        counter += 1
        
        # 每次写入后等待 1 秒
        time.sleep(1)
        
    # 关闭内存映射
    mm.close()
