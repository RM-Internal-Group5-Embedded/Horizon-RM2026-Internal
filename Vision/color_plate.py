import cv2
import numpy as np
from PIL import Image

red=[0,0,255]#red in BGR colorspace
blue=[255,0,0]
green=[0,255,0]

#找到color所在的地方 获取mask
def detect_colors(frame):
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
    #使用opencv调用电脑中的摄像头 需要传入摄像头的序号 到设备管理器中看
    capture=cv2.VideoCapture(0)

    if capture.isOpened():
        print("USB相机连接成功")
    else:
        print("USB相机连接失败")

    capture.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH,1280)#设置相机采集分辨率 还要看是否支持

    #要循环读取每一帧的图像
    while True:
        #读取每一帧图像
        ret,frame=capture.read()
        if not ret:
            print("无法读取视频帧")
            break
        
        #检测红蓝色块，现在可以检测多个
        red_mask, blue_mask, green_mask = detect_colors(frame)
        red_blocks = find_bboxs(red_mask)
        blue_blocks = find_bboxs(blue_mask)
        green_blocks=find_bboxs(green_mask)
           
        #在图像上绘制结果
        #红色block边框显示
        for red_block in red_blocks:
            x1,y1,x2,y2= red_block
            cv2.rectangle(frame,(x1,y1),(x2,y2),(0,0,255),5)
            # 添加标签
            cv2.putText(frame, 'Red', (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

        #蓝色block边框显示
        for blue_block in blue_blocks:
            x1,y1,x2,y2= blue_block
            cv2.rectangle(frame,(x1,y1),(x2,y2),(255,0,0),5)
            # 添加标签
            cv2.putText(frame, 'Blue', (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,0), 2)
       
        #绿色block边框显示
        for green_block in green_blocks:
            x1,y1,x2,y2= green_block
            cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),5)
            # 添加标签
            cv2.putText(frame, 'Green', (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

        cv2.imshow("camera",frame)
        #等待键盘输入1毫秒
        key=cv2.waitKey(1)
        #如果输入了任意键 key的值就不等于-1
        if key!=-1:
            break

    #最后释放capture指针
    capture.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()