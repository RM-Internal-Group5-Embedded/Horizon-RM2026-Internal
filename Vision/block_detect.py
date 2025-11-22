#主运行
#检测block是否进入预先标注的4个区域，并把进入事件通过 JC24B 发送出去
import os
import json
import cv2  
import numpy as np
from PIL import Image
import time
from JC42BSending import JC24BTransceiver
import serial.tools.list_ports

red=[0,0,255]#red in BGR colorspace
blue=[255,0,0]
green=[0,255,0]

CONFIG_DIR = os.path.join(os.path.dirname(__file__), 'config')
REGIONS_BLOCK_PATH = os.path.join(CONFIG_DIR, 'block_regions_view.json')
REGIONS_COLOR_PATH = os.path.join(CONFIG_DIR, 'color_plate_region.json')


def find_jc24b_port():
    """自动查找可能的JC24B设备"""
    
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

def load_regions(path):
    if not os.path.exists(path):
        print("缺少json文件,需要先运行define_regions.py")
        return []
    if path==REGIONS_COLOR_PATH:
        a='region'
    else:
        a='regions'
    with open(path,'r') as f:
        data=json.load(f)
        return data[a]
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
            text='R'+str(index+1)
            #绘制字符串 画布，字符串，坐标，字体序号，缩放系数，颜色，粗细，线条类型(以下实线)
            cv2.putText(img,text, tuple(poly[0]), cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,0),2,1)

def regions_to_positions(index):
    positions=[0,0,0,0]
    positions[index]=1
    return positions

#color 

# 创建区域掩膜
def create_region_mask(frame, region_points):
    if region_points is None or len(region_points) < 3:
        return None
    
    mask = np.zeros(frame.shape[:2], dtype=np.uint8) #全0（黑色）画布
    points = np.array(region_points, dtype=np.int32)
    cv2.fillPoly(mask, [points], 255) #画布，区域，填充颜色
    return mask

#找到color所在的地方 获取mask
def detect_colors(frame,region_mask=None):
    hsv_frame=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)#转换为HSV

    # 红色在HSV空间上分两段
    lower_red1 = np.array([0, 70, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 70, 50])
    upper_red2 = np.array([180, 255, 255])
    
    red_mask1 = cv2.inRange(hsv_frame, lower_red1, upper_red1)
    red_mask2 = cv2.inRange(hsv_frame, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)

    # 蓝色区间Hue建议100~130左右，饱和度/明度门槛别太低
    lower_blue = np.array([100, 120, 70])
    upper_blue = np.array([130, 255, 255])
    blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)

    # 绿色区间Hue
    lower_green = np.array([35, 40, 40])
    upper_green = np.array([90, 255, 255])
    green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)

    #去噪
    kernel = np.ones((5, 5), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)

     # 如果指定了区域掩膜，则只在区域内检测
    if region_mask is not None:
        #裁剪颜色掩膜
        #cv2.bitwise_and(A, B) 按位OR：A和B都为255才保留
        red_mask = cv2.bitwise_and(red_mask, region_mask)
        blue_mask = cv2.bitwise_and(blue_mask, region_mask)
        green_mask = cv2.bitwise_and(green_mask, region_mask)
    
    return red_mask, blue_mask, green_mask  


#找到color对应的所有bounding box
def find_bboxs(mask):
    #找到所有独立的颜色区域
    contours, _ =cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    bboxs=[]
    
    for contour in contours:
        # 过滤掉太小的区域
        area=cv2.contourArea(contour)
        if area>1000:  #可以修改这个阈值来过滤噪声
            x,y,w,h=cv2.boundingRect(contour)
            bboxs.append((x,y,x+w,y+h))
    
    return bboxs

def main():
    try:
        regions=load_regions(REGIONS_BLOCK_PATH)
        if not regions:
            return 
        
        capture=cv2.VideoCapture(0)
        if capture.isOpened():
            print("USB相机连接成功")
        else:
            print("USB相机连接失败")
            return 
        
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH,1280)#设置相机采集分辨率 还要看是否支持

        # 读取区域配置
        region_points = load_regions(REGIONS_COLOR_PATH)
        use_region = region_points is not None
        region_mask = None


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
                transceiver.last_ack_time = time.time()
            else:
                print("JC24B设备连接失败，将运行无通信版本")
                transceiver = None
        else:
            print("无法确定JC24B设备，将运行无通信版本")
            transceiver = None



        # 添加状态跟踪
        current_positions = [0, 0, 0, 0]  # 当前所有区域状态
        last_positions = [0, 0, 0, 0]     # 上一帧的状态
        consecutive_frames = 0             # 连续相同状态的帧数
        DEBOUNCE_THRESHOLD = 3             # 去抖阈值，与STM32端保持一致
        
        # 发送控制
        send_interval = 0.1  # 100ms发送间隔
        last_send_time = 0
        # 通信状态
        connection_healthy = False
        ack_timeout = 0.5  # 500ms，与STM32看门狗一致

        while True:
            ret,frame=capture.read()
            if not ret:
                print("无法读取视频帧")
                break
            
            if use_region and region_mask is None:
                region_mask = create_region_mask(frame, region_points)
        
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
                        transceiver.connect() 
                last_send_time = current_time
            
            if transceiver:
                # 非阻塞检查ACK
                transceiver.check_for_ack()
                    
            # 检查通信状态
            if hasattr(transceiver, 'last_ack_time'):
                if current_time - transceiver.last_ack_time <= ack_timeout:
                    if not connection_healthy:
                        print("通信连接正常")
                        connection_healthy = True
                else:
                    if connection_healthy:
                        print("警告: 通信连接异常 - 未收到ACK")
                        connection_healthy = False
            
            #检测红蓝色块，现在可以检测多个
            red_mask, blue_mask, green_mask = detect_colors(frame,region_mask)
            red_blocks = find_bboxs(red_mask)
            blue_blocks = find_bboxs(blue_mask)
            green_blocks=find_bboxs(green_mask)
            
            # 绘制检测区域（如果使用区域检测）
            if use_region and region_points is not None:
                # 绘制区域边界
                points = np.array(region_points, dtype=np.int32)
                cv2.polylines(canvas, [points], True, (255, 255, 255), 2)
                # 添加区域标签
                cv2.putText(canvas, 'Detection Region', 
                        (region_points[0][0], region_points[0][1]-10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            #在图像上绘制结果
            #红色block边框显示
            for red_block in red_blocks:
                x1,y1,x2,y2= red_block
                cv2.rectangle(canvas,(x1,y1),(x2,y2),(0,0,255),5)
                # 添加标签
                cv2.putText(canvas, 'Red', (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

            #蓝色block边框显示
            for blue_block in blue_blocks:
                x1,y1,x2,y2= blue_block
                cv2.rectangle(canvas,(x1,y1),(x2,y2),(255,0,0),5)
                # 添加标签
                cv2.putText(canvas, 'Blue', (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,0), 2)
        
            #绿色block边框显示
            for green_block in green_blocks:
                x1,y1,x2,y2= green_block
                cv2.rectangle(canvas,(x1,y1),(x2,y2),(0,255,0),5)
                # 添加标签
                cv2.putText(canvas, 'Green', (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        
            #显示画面
            cv2.imshow('view', canvas)
    
            # 按键检测
            key = cv2.waitKey(30) & 0xFF
            if key == ord('q') or key == 27:
                print("收到退出指令")
                break

    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"程序发生错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        capture.release()
        cv2.destroyAllWindows()
        if transceiver:
            transceiver.disconnect()
        print("程序已退出")

if __name__=='__main__':
    main()