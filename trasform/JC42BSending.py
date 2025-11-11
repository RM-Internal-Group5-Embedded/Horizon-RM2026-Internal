import serial
import time
import threading

class JC24BTransceiver:
    def __init__(self, port='COM8', baudrate=9600, timeout=1):
        """
        初始化JC24B通信模块
        
        Args:
            port: 串口端口 (Windows: COM3, Linux: /dev/ttyUSB0)
            baudrate: 波特率默认9600
            timeout: 串口超时时间
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.connected = False
        self.running = False
        self.reconnect_count = 0
        self.last_ack_time = 0
        self.ack_timeout = 0.5  # 500ms确认超时
        
        # 线程锁
        self.lock = threading.Lock()
        
    def connect(self):
        """连接串口"""
        try:
            with self.lock:
                if self.ser and self.ser.is_open:
                    self.ser.close()
                    
                self.ser = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=self.timeout,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE
                )
                
                self.connected = True
                self.reconnect_count = 0
                print(f"串口连接成功: {self.port}")
                return True
                
        except Exception as e:
            print(f"串口连接失败: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """断开连接并关闭串口"""
        self.running = False
        self.connected = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        print("串口已断开")
    
    def stop(self):
        """暂停发送"""
        self.running = False
        print("发送已暂停")
    
    def calculate_checksum(self, data):
        """计算校验和"""
        return sum(data) & 0xFF
    
    def build_position_packet(self, positions):
        """
        构建位置数据包
        
        Args:
            positions: 4个位置的状态列表 [pos1, pos2, pos3, pos4]
            
        Returns:
            打包后的字节数据
        """
        if len(positions) != 4:
            raise ValueError("位置数据必须是4个元素")
        
        # 构建数据包: [AA BB pos1 pos2 pos3 pos4 checksum]
        packet_data = [
            0xAA, 0xBB,                    # 帧头
            positions[0], positions[1],    # 位置1,2
            positions[2], positions[3],    # 位置3,4
            0x00                           # 校验和占位
        ]
        
        # 计算校验和
        checksum = self.calculate_checksum(packet_data[:-1])
        packet_data[-1] = checksum
        
        return bytes(packet_data)
    
    def parse_ack_packet(self, data):
        """
        解析确认包
        
        Args:
            data: 接收到的字节数据
            
        Returns:
            True: 确认包有效, False: 无效
        """
        if len(data) < 4:
            return False
            
        # 检查帧头
        if data[0] != 0xCC or data[1] != 0xDD:
            return False
            
        # 验证校验和
        checksum = self.calculate_checksum(data[:-1])
        if checksum != data[-1]:
            return False
        
        return True
    
    def send_positions(self, positions):
        """
        发送位置数据
        
        Args:
            positions: 4个位置的状态列表
            
        Returns:
            bool: 发送是否成功
        """
        if not self.connected or not self.ser:
            print("串口未连接，无法发送数据")
            return False
        
        try:
            with self.lock:
                # 构建数据包
                packet = self.build_position_packet(positions)
                
                # 发送数据
                self.ser.write(packet)
                self.ser.flush()
                print(f"发送位置: {positions}")
                return True
                
        except Exception as e:
            print(f"发送数据失败: {e}")
            self.connected = False
            return False
    
    def check_for_ack(self):
        """
        检查确认包
        
        Returns:
            bool: 是否收到有效确认包
        """
        if not self.connected or not self.ser:
            return False
        
        try:
            with self.lock:
                # 检查是否有数据可读
                if self.ser.in_waiting >= 4:  # 确认包是4字节
                    data = self.ser.read(4)
                    
                    if self.parse_ack_packet(data):
                        self.last_ack_time = time.time()
                        return True
                    else:
                        print("收到无效确认包")
                        
        except Exception as e:
            print(f"读取数据失败: {e}")
            
        return False
    
    def is_connection_healthy(self):
        """检查连接是否健康（基于确认包）"""
        if not self.connected:
            return False
            
        # 如果最近收到过确认包，认为连接健康
        current_time = time.time()
        return (current_time - self.last_ack_time) <= self.ack_timeout
    
    def start_communication(self, positions_callback=None):
        """
        开始通信循环
        
        Args:
            positions_callback: 可选的函数，返回要发送的位置数据
                               如果不提供，使用默认测试数据
        """
        if not self.connected:
            if not self.connect():
                print("无法启动通信：串口连接失败")
                return False
        
        print("启动通信循环...")
        self.running = True
        
        # 如果没有提供回调函数，使用默认测试数据
        if positions_callback is None:
            def default_callback():
                return [1, 0, 1, 0]  # 测试数据
            positions_callback = default_callback
        
        # 启动通信线程
        comm_thread = threading.Thread(target=self._communication_loop, args=(positions_callback,))
        comm_thread.daemon = True
        comm_thread.start()
        
        return True
    
    def _communication_loop(self, positions_callback):
        """通信循环（在独立线程中运行）"""
        ack_miss_count = 0  # 连续未收到ACK的次数
        
        while self.running:
            try:
                # 检查连接状态
                if not self.connected or not self.ser or not self.ser.is_open:
                    print("连接已断开，停止发送")
                    break
                
                # 获取要发送的位置数据
                positions = positions_callback()
                
                # 发送位置数据
                if self.send_positions(positions):
                    # 检查确认包（非阻塞）
                    if self.check_for_ack():
                        ack_miss_count = 0  # 重置计数
                    else:
                        ack_miss_count += 1
                        if ack_miss_count >= 5:  # 连续5次没收到ACK
                            print("警告：连续未收到确认包")
                
                # 严格保持100ms间隔
                time.sleep(0.1)
                
            except KeyboardInterrupt:
                print("用户中断")
                break
            except Exception as e:
                print(f"通信循环错误: {e}")
                time.sleep(0.1)  # 出错时短暂等待
        
        print("通信循环已结束")