#主运行
#检测block是否进入预先标注的4个区域，并把进入事件通过 JC24B 发送出去
import os
import json
import cv2  
import numpy as np
import time
from JC42BSending import JC24BTransceiver
import serial.tools.list_ports

CONFIG_DIR = os.path.join(os.path.dirname(__file__), 'config')
REGIONS_PATH = os.path.join(CONFIG_DIR, 'block_regions_view.json')


def find_jc24b_port():
    """自动查找可能的JC24B设备"""
    possible_ports = []
    
    # 获取所有串口设备
    ports = list(serial.tools.list_ports.comports())
    
    for port in ports:
        print(f"发现设备: {port.device} - {port.description} - {port.hwid}")
        
        # 根据常见特征判断是否是JC24B
        if any(keyword in port.description.upper() for keyword in ['JC24B', 'CH340', 'CP210', 'USB-SERIAL']):
            return port.device
        elif 'VID:PID=1A86:7523' in port.hwid:  # CH340芯片的常见VID:PID
            return port.device
        elif 'VID:PID=10C4:EA60' in port.hwid:  # CP210x芯片的常见VID:PID
            return port.device
    
    # 如果没有自动识别到，让用户选择
    if ports:
        print("\n请选择JC24B设备:")
        for i, port in enumerate(ports):
            print(f"{i}: {port.device} - {port.description}")
        
        try:
            choice = int(input("输入设备编号: "))
            if 0 <= choice < len(ports):
                return ports[choice].device
        except:
            pass
    
    return None

def load_regions():
    if not os.path.exists(REGIONS_PATH):
        print("缺少block_regions_view.json文件,需要先运行define_regions.py")
        return []
    with open(REGIONS_PATH,'r') as f:
        data=json.load(f)
        return data['regions']
        #json.dump({'regions':regions},f) 用字典存的

def point_in_region(point,region):
    # 1:在多边形内部，0:在多边形边上，-1:在多边形外部
    a=cv2.pointPolygonTest(np.array(region,dtype=np.int32),(int(point[0]),int(point[1])),False)
    return a==1

def detect_blocks(frame):#只用detect矩形边框 等下看在框定的区域中有没有完整的矩形边框
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #通过选择的算法 自动计算阈值 图像二值化 黑底白块 比平均值亮5以上的变白色
    binary = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV,31,5)
    #去噪
    kernel = np.ones((5, 5), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)#腐蚀膨胀 去白点
    binary=cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)#膨胀腐蚀 去黑洞

    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)#只检测最外层轮廓 智能压缩直线上的冗余点】
    detections=[]
    for c in contours:
        #面积过滤
        area=cv2.contourArea(c)
        if area<800:#可调整 面积必须比800大
            continue
        
        #形状过滤
        #最小的可以包裹着住边框的正方形. 左上角x,y
        x,y,w,h=cv2.boundingRect(c)
        if w/max(h,1)<0.5 or w/max(h,1)>2:
            continue

        #在图像坐标系中，向下是 +y
        detections.append({'block':(x,y,x+w,y+h),'centre':(x+w/2.0,y+h/2.0)})

    return binary,detections

def draw_regions(img,regions):
    for index in range(len(regions)):
        poly=regions[index]
        cv2.polylines(img,[np.int32(poly)],True,(0,255,0),2)
        if len(poly)>0:
            text='R'+chr(index+1+ord('A'))
            #绘制字符串 画布，字符串，坐标，字体序号，缩放系数，颜色，粗细，线条类型(以下实线)
            cv2.putText(img,text, tuple(poly[0]), cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,0),2,1)

def regions_to_positions(index):
    positions=[0,0,0,0]
    positions[index]=1
    return positions

def main():
    regions=load_regions()
    if not regions:
        return 
    
    capture=cv2.VideoCapture(0)
    if capture.isOpened():
        print("USB相机连接成功")
    else:
        print("USB相机连接失败")
        return 
    

    # 自动查找JC24B设备
    jc24b_port = find_jc24b_port()
    if not jc24b_port:
        print("未找到JC24B设备，请检查连接")
        # 或者让用户手动输入
        jc24b_port = input("请手动输入设备路径 (如 /dev/tty.usbserial-XXXX): ").strip()
    
    transceiver = None
    if jc24b_port:
        print(f"尝试连接JC24B设备: {jc24b_port}")
        transceiver = JC24BTransceiver(port=jc24b_port)
        if transceiver.connect():
            print("JC24B设备连接成功")
        else:
            print("JC24B设备连接失败，将运行无通信版本")
            transceiver = None
    else:
        print("无法确定JC24B设备，将运行无通信版本")
        transceiver = None



    prev_inside = [False]*len(regions)  #记录上一帧每个区域是否有block

    # 添加状态跟踪
    current_positions = [0, 0, 0, 0]  # 当前所有区域状态
    last_positions = [0, 0, 0, 0]     # 上一帧的状态
    consecutive_frames = 0             # 连续相同状态的帧数
    DEBOUNCE_THRESHOLD = 3             # 去抖阈值，与STM32端保持一致
    
    # 发送控制
    send_interval = 0.1  # 100ms发送间隔
    last_send_time = 0

    while True:
        ret,frame=capture.read()
        if not ret:
            print("无法读取视频帧")
            break
        
        #mask:处理后的二值图像 detections:检测到的block列表，包含位置和中心点
        mask,detections = detect_blocks(frame)

        #画出regions区域
        canvas=frame.copy()
        draw_regions(canvas,regions)
        for d in detections:
            x1,y1,x2,y2=d['block']
            cx,cy=d['centre']
            #橙色矩形框:block 橙色圆点:block中心
            cv2.rectangle(canvas,(x1,y1),(x2,y2),(0, 200, 255), 2)
            cv2.circle(canvas,(int(cx),int(cy)), 4, (0, 200, 255), -1)

        #检测block中心是否在regions内
        #本次情况
        now_inside=[]
        for poly in regions:
            is_block=False
            for d in detections:
                if point_in_region(d['centre'],poly):
                    is_block=True             
            now_inside.append(is_block)

        # 计算新状态
        new_positions = [1 if inside else 0 for inside in now_inside]
        
        # 视觉端去抖逻辑
        if new_positions == last_positions:
            consecutive_frames += 1
        else:
            consecutive_frames = 0
            last_positions = new_positions.copy()
         # 只有连续多帧状态稳定才更新当前状态
        if consecutive_frames >= DEBOUNCE_THRESHOLD and new_positions != current_positions:
            current_positions = new_positions.copy()
            print(f"状态稳定确认: {current_positions}")
        
        # 定期发送当前位置（比如每0.5秒）或者状态变化时立即发送
        current_time = time.time()
        if current_time - last_send_time >= send_interval:
            if transceiver and transceiver.connected:
                ok = transceiver.send_positions(current_positions)
                if ok:
                    # print(f"发送位置: {current_positions}")  # 调试时可注释掉，减少输出
                    pass
                else:
                    print("发送失败")
            last_send_time = current_time
        
        # for i in range(len(regions)):
        #     was_inside=prev_inside[i]
        #     is_inside=now_inside[i]
            
        #     if not was_inside and is_inside:
        #         print("Block 到达第", i+1 ,"个区域") #无线传输
        #         positions=regions_to_positions(i)
        #         ok = False
        #         if transceiver.connect():
        #             ok=transceiver.send_positions(positions)
        #             if ok:
        #                 print("发送成功: ", positions)
        #         if not ok:
        #             print("首次发送失败，尝试重连并重发...")
        #             # 尝试重连一次
        #             time.sleep(0.05)
        #             ok2=transceiver.send_positions(positions)
        #             if ok2:
        #                 print("重连后发送成功")
        #             else:
        #                 print("重连后仍然发送失败（请检查串口或接收端）")
      
        #更新
        prev_inside = now_inside.copy()
        
        #显示画面
        cv2.imshow('view', canvas)
        #cv2.imshow('mask', mask)
 
        # 改进的按键检测 - 使用更长的等待时间并添加异常处理
        try:
            key = cv2.waitKey(30) & 0xFF  # 增加到30ms，减少CPU使用率
            if key == ord('q'):
                print("收到退出指令，正在关闭...")
                break
            elif key == 27:  # ESC键
                print("收到ESC退出指令,正在关闭...")
                break
        except KeyboardInterrupt:
            print("收到中断信号，正在关闭...")
            break
        

    capture.release()
    cv2.destroyAllWindows()
    transceiver.disconnect()
    print("程序退出，串口已关闭")

if __name__=='__main__':
    main()