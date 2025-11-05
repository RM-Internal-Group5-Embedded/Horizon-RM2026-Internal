#用鼠标在摄像头画面上画出4个区域，然后保存这些区域的坐标。

#操作:
#空格冻结当前帧
#用鼠标依次点击顶点绘制一个区域（≥3点）
#按 n 确认当前区域，依次画满4个
#按 s 保存（保存到 config/regions_view.json）
#按 q 退出

#输出：一个包含4个区域坐标的配置文件 以 JSON 格式写入
import os   #管理文件和文件夹
import json #保存和读取数据
import cv2  
import numpy as np


#__file__ 当前Python文件自己的位置
#os.path.dirname(__file__) = 找到这个文件所在的文件夹
#os.path.join(文件夹, 'config') = 在这个文件夹里创建一个叫config的子文件夹
CONFIG_PATH = os.path.join(os.path.dirname(__file__), 'config')#'config'文件夹路径
REGIONS_PATH = os.path.join(CONFIG_PATH, 'regions_view.json')#'regions_view.json'文件路径

regions = []    # 存放所有画好的区域
current = []    # 存放正在画的区域
frozen = None   # 存放暂停时的画面

#event:鼠标动作（点击、移动等） x, y :鼠标点击的坐标位置
#param:是否冻结画面
def on_mouse(event,x,y,flags,param):
    global current
    if param is None:
        return
    if event==cv2.EVENT_LBUTTONDOWN:
        current.append([int(x),int(y)])

#画图函数   
def draw_poly(img,pts,color):
    if len(pts)>=3:
        cv2.polylines(img,[np.int32(pts)],False,color,2)#bool是否闭合多边形
    for p in pts:
        cv2.circle(img,tuple(p),4,color,-1)

    
def main():
    os.makedirs(CONFIG_PATH,exist_ok=True)
    capture=cv2.VideoCapture(0)

    if capture.isOpened():
        print("USB相机连接成功")
    else:
        print("USB相机连接失败")
        return 

    capture.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH,1280)#设置相机采集分辨率 还要看是否支持

    print("按空格冻结当前帧,用鼠标依次点击顶点绘制一个区域 (大于等于3个点)")
    print("按 n 确认当前区域并开始下一个,共4个;按 u 撤销一点")
    print("按 s 保存全部4个区域;按 r 重新冻结;按 q 退出")

    cv2.namedWindow('regions_view')
    #设置鼠标监听，当在窗口里点击时调用mouse函数
    #cv2.setMouseCallback(窗口名, 回调函数, 参数)
    #mouse 只能通过 param 参数传一个数据
    cv2.setMouseCallback('regions_view',on_mouse, param=None)

    while True:
        if frozen is None:
            ret,frame=capture.read()
            if not ret:
                print("无法读取视频帧")
                break
            view=frame.copy()
        else:
            view=frozen.copy()  #使用冻结的画面，后面是不是要调回来
        canvas=view.copy()

        #已完成的区域 绿色
        for index in range(len(regions)):
            #画轮廓
            poly=regions[index]
            draw_poly(canvas,poly,(0,255,0))

            #只有三个点以上才标注
            if len(poly)>=3:
                text='R'+chr(index+1+ord('A'))
                #绘制字符串 画布，字符串，坐标，字体序号，缩放系数，颜色，粗细，线条类型(以下实线)
                cv2.putText(canvas,text, tuple(poly[0]),0,0.7,(0,255,0),2,1)

        #正在画的区域 蓝色
        draw_poly(canvas,current,(255,0,0))

        #按键处理
        cv2.imshow('regions_view',canvas)
        key = cv2.waitKey(1) & 0xFF #只取键盘码

        #按空格冻结当前帧,用鼠标依次点击顶点绘制一个区域 (大于等于3个点)
        #按 n 确认当前区域并开始下一个,共4个;按 u 撤销一点
        #按 s 保存全部4个区域;按 r 重新冻结;按 q 退出
        #按下q按键
        if key==ord('q'):
            break
        if key==ord(' '):
            ret,frame=capture.read()
            if ret:
                frozen=frame.copy()
                current=[]
                cv2.setMouseCallback('regions_view',on_mouse, frozen)
        if key==ord('u'):
            if current:
                current.pop()
        if key==ord('n'):
            if len(current)>=3:
                regions.append(current.copy())#必须用copy 存储地址
                current=[]
            else:
                print("当前区域至少需要三个点")
        if key==ord('r'):
            frozen=None
            current=[]
            cv2.setMouseCallback('regions_view', on_mouse, None)
            #setMouseCallback 是“重新绑定鼠标行为”: 改变param 或 重启绘制时才需要！
        if key==ord('s'):
            if len(regions)==4:
                #json格式写入文件
                with open(REGIONS_PATH,'w') as f:
                    json.dump({'regions':regions},f)
                print("已保存")
            else:
                print("需要绘制4个区域后再保存")
    capture.release()
    cv2.destroyAllWindows()
if __name__=='__main__':
    main()

        
          