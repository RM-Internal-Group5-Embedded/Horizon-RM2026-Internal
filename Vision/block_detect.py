#主运行
#检测block是否进入预先标注的4个区域，并把进入事件通过 JC24B 发送出去
import os
import json
import cv2  
import numpy as np
import time
from JC42BSending import JC24BTransceiver

CONFIG_DIR = os.path.join(os.path.dirname(__file__), 'config')
REGIONS_PATH = os.path.join(CONFIG_DIR, 'regions_view.json')

def load_regions():
    if not os.path.exists(REGIONS_PATH):
        print("缺少regions_view.json文件,需要先运行define_regions.py")
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
    
    # 创建 JC24BTransceiver 并尝试连接
    transceiver=JC24BTransceiver(port='COM8')#哪个串口要检测一下
    transceiver.connect()


    prev_inside = [False]*len(regions)  #记录上一帧每个区域是否有block

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

        for i in range(len(regions)):
            was_inside=prev_inside[i]
            is_inside=now_inside[i]
            
            if not was_inside and is_inside:
                print("Block 到达第", i+1 ,"个区域") #无线传输
                positions=regions_to_positions(i)
                ok = False
                if transceiver.connect():
                    ok=transceiver.send_positions(positions)
                    if ok:
                        print("发送成功: ", positions)
                if not ok:
                    print("首次发送失败，尝试重连并重发...")
                    # 尝试重连一次
                    if transceiver.connect():
                        time.sleep(0.05)
                        ok2=transceiver.send_positions(positions)
                        if ok2:
                            print("重连后发送成功")
                        else:
                            print("重连后仍然发送失败（请检查串口或接收端）")
                    else:
                        print("重连失败（请检查串口设备或端口）")
      
        #更新
        prev_inside = now_inside.copy()
        
        #显示画面
        cv2.imshow('view', canvas)
        cv2.imshow('mask', mask)
 
        key = cv2.waitKey(1) & 0xFF #只取键盘码
        if key==ord('q'):
            break

    capture.release()
    cv2.destroyAllWindows()
    transceiver.disconnect()
    print("程序退出，串口已关闭")

if __name__=='__main__':
    main()