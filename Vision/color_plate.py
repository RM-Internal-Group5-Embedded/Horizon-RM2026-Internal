import cv2
import numpy as np
from PIL import Image

red=[0,0,255]#red in BGR colorspace
blue=[255,0,0]

def get_limits(color):
    c=np.uint8([[color]])#insert the bgr value which you want to convert to hsv
    hsvC=cv2.cvtColor(c,cv2.COLOR_BGR2HSV)
    
    lowerLimit=hsvC[0][0][0]-20,50,50 #可以修改上下限范围
    upperLimit=hsvC[0][0][0]+20,255,255

    lowerLimit=np.array(lowerLimit,dtype=np.uint8)
    upperLimit=np.array(upperLimit,dtype=np.uint8)

    return lowerLimit,upperLimit

#找到color所在的地方 获取mask
def detect_colors(frame):
    hsv_frame=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)#转换为HSV

    lowerLimit,upperLimit=get_limits(red)
    red_mask=cv2.inRange(hsv_frame,lowerLimit,upperLimit)
    #exact locations of all the pixels containing the information we want

    lowerLimit,upperLimit=get_limits(blue)
    blue_mask=cv2.inRange(hsv_frame,lowerLimit,upperLimit)

    return red_mask, blue_mask
    

#找到color对应的所有bounding box
def find_bboxs(mask):
    #找到所有独立的颜色区域
    contours, _ =cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    bboxs=[]
    
    for contour in contours:
        # 过滤掉太小的区域
        area=cv2.contourArea(contour)
        if area>500:  #可以修改这个阈值来过滤噪声
            x,y,w,h=cv2.boundingRect(contour)
            bboxs.append((x,y,x+w,y+h))
    
    return bboxs

def main():
    #使用opencv调用电脑中的摄像头 需要传入摄像头的序号 到设备管理器中看
    capture=cv2.VideoCapture()

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
        red_mask, blue_mask = detect_colors(frame)
        red_blocks = find_bboxs(red_mask)
        blue_blocks = find_bboxs(blue_mask)
        
           
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