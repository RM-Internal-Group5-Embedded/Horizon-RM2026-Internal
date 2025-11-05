import cv2
import numpy as np
import math

grey=[,,]#gray in BGR colorspace

#根据原有图像识别block特征点  现在一次只能识别一个
def blockKeypoints(img,frame):
    block=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    scene=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    #ORB检测器 可以修改feature数量
    orb=cv2.ORB_create(nfeatures=2000)
    #检测关键点 计算描述符
    kp1,des1=orb.detectAndCompute(block,None)
    kp2,des2=orb.detectAndCompute(scene,None)
    #创建BFMatcher
    bf=cv2.BFMatcher(cv2.NORM_HAMMING,True)
    #KNN匹配
    matches=bf.knnMatch(des1,des2,k=2)
    #使用lowe's ratio test筛选匹配点 如果有一个明显优于另一个 那很有可能是对的
    good_matches=[]
    for m,n in matches:
        if m.distance<0.75 * n.distance:
            good_matches.append(m)

    if len(good_matches)>=10:#检查是否有足够多的良好匹配点
        #获取匹配点坐标
        src_pts=np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1,1,2)
        dst_pts=np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1,1,2)
        #计算单应性矩阵
        H,mask=cv2.findHomography(src_pts,dst_pts,cv2.RANSAC,5.0)
    
    if H is not None:
        #获取目标图像的四个角点
        h,w=img.shape[:2]
        pts=np.float32([[0,0],[0,h-1],[w-1,h-1],[w-1,0]]).reshape(-1,1,2)

        #将角点映射到场景图像中
        dst=cv2.perspectiveTransform(pts,H)
        # 计算方块中心位置
        center_pt = np.mean(dst, axis=0)[0]
        center_x, center_y = int(center_pt[0]), int(center_pt[1])
            
        #在场景图像中绘制目标边框 不需要可以删掉
        cv2.polylines(scene,[np.int32(dst)],True,(0,255,0),3,cv2.LINE_AA)

        return [center_x,center_y]

def findBlockContour(frame):




def main():
    #使用opencv调用电脑中的摄像头 需要传入摄像头的序号 到设备管理器中看
    capture=cv2.VideoCapture(0)

    if capture.isOpened():
        print("USB相机连接成功")
    else:
        print("USB相机连接失败")

    capture.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH,1280)#设置相机采集分辨率 还要看是否支持

    #特征提取原图
    img=cv2.imread("")

    #要循环读取每一帧的图像
    while True:
        #读取每一帧图像
        ret,frame=capture.read()
        if not ret:
            print("无法读取视频帧")
            break

        x,y=-1,-1 #可以吗
        #find the Block  现在一次只能找到一个
        x,y=blockKeypoints(img,frame)

        if x!=-1 and y !=-1:
            #find where is those four block position
            _=findBlockContour(frame)

        key=cv2.waitKey(1)
        #如果输入了任意键 key的值就不等于-1
        if key!=-1:
            break

    #最后释放capture指针
    capture.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()