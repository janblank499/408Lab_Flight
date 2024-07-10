#main.py -- put your code here!
import cpufreq
import pyb
import sensor,image, time,math
from pyb import LED,Timer,UART

sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)  # Set frame size to QVGA (320x240)
sensor.skip_frames(time = 2000)     #延时跳过一些帧，等待感光元件变稳定
#sensor.set_auto_gain(False)
#sensor.set_auto_whitebal(False)
clock = time.clock()                # Create a clock object to track the FPS.
sensor.set_auto_exposure(True, exposure_us=5000) # 设置自动曝光

uart=UART(3,256000)#1382400
THRESHOLD = (0,100) # Grayscale threshold for dark things... (5, 70, -23, 15, -57, 0)(18, 100, 31, -24, -21, 70)
IMAGE_WIDTH=sensor.snapshot().width()
IMAGE_HEIGHT=sensor.snapshot().height()
IMAGE_DIS_MAX=(int)(math.sqrt(IMAGE_WIDTH*IMAGE_WIDTH+IMAGE_HEIGHT*IMAGE_HEIGHT)/2)


#OPENMV外接测距模块
uart1=UART(1,921600)


class target_check(object):
    x=0          #int16_t
    y=0          #int16_t
    pixel=0      #uint16_t
    flag=0       #uint8_t
    state=0      #uint8_t
    angle=0      #int16_t
    distance=0   #uint16_t
    apriltag_id=0#uint16_t
    img_width=0  #uint16_t
    img_height=0 #uint16_t
    reserved1=0  #uint8_t
    reserved2=0  #uint8_t
    reserved3=0  #uint8_t
    reserved4=0  #uint8_t
    fps=0        #uint8_t
    range_sensor1=0
    range_sensor2=0
    range_sensor3=0
    range_sensor4=0
    camera_id=0
    reserved1_int32=0
    reserved2_int32=0
    reserved3_int32=0
    reserved4_int32=0


class rgb(object):
    def __init__(self):
        self.red=LED(1)
        self.green=LED(2)
        self.blue=LED(3)



class uart_buf_prase(object):
    uart_buf = []
    _data_len = 0
    _data_cnt = 0
    state = 0

class mode_ctrl(object):
    work_mode = 0x01 #工作模式.默认是点检测，可以通过串口设置成其他模式
    check_show = 1   #开显示，在线调试时可以打开，离线使用请关闭，可提高计算速度

ctr=mode_ctrl()


rgb=rgb()
R=uart_buf_prase()
target=target_check()
target.camera_id=0x01
target.reserved1_int32=65536
target.reserved2_int32=105536
target.reserved3_int32=65537
target.reserved4_int32=105537

HEADER=[0xFF,0xFC]
MODE=[0xF1,0xF2,0xF3]
#__________________________________________________________________
def package_blobs_data(mode):
    #数据打包封装
    data=bytearray([HEADER[0],HEADER[1],0xA0+mode,0x00,
                   target.x>>8,target.x,        #将整形数据拆分成两个8位
                   target.y>>8,target.y,        #将整形数据拆分成两个8位
                   target.pixel>>8,target.pixel,#将整形数据拆分成两个8位
                   target.flag,                 #数据有效标志位
                   target.state,                #数据有效标志位
                   target.angle>>8,target.angle,#将整形数据拆分成两个8位
                   target.distance>>8,target.distance,#将整形数据拆分成两个8位
                   target.apriltag_id>>8,target.apriltag_id,#将整形数据拆分成两个8位
                   target.img_width>>8,target.img_width,    #将整形数据拆分成两个8位
                   target.img_height>>8,target.img_height,  #将整形数据拆分成两个8位
                   target.fps,      #数据有效标志位
                   target.reserved1,#数据有效标志位
                   target.reserved2,#数据有效标志位
                   target.reserved3,#数据有效标志位
                   target.reserved4,#数据有效标志位
                   target.range_sensor1>>8,target.range_sensor1,
                   target.range_sensor2>>8,target.range_sensor2,
                   target.range_sensor3>>8,target.range_sensor3,
                   target.range_sensor4>>8,target.range_sensor4,
                   target.camera_id,
                   target.reserved1_int32>>24&0xff,target.reserved1_int32>>16&0xff,
                   target.reserved1_int32>>8&0xff,target.reserved1_int32&0xff,
                   target.reserved2_int32>>24&0xff,target.reserved2_int32>>16&0xff,
                   target.reserved2_int32>>8&0xff,target.reserved2_int32&0xff,
                   target.reserved3_int32>>24&0xff,target.reserved3_int32>>16&0xff,
                   target.reserved3_int32>>8&0xff,target.reserved3_int32&0xff,
                   target.reserved4_int32>>24&0xff,target.reserved4_int32>>16&0xff,
                   target.reserved4_int32>>8&0xff,target.reserved4_int32&0xff,
                   0x00])
    #数据包的长度
    data_len=len(data)
    #print("serialport data length is:",data_len)
    data[3]=data_len-5#有效数据的长度
    #和校验
    sum=0
    for i in range(0,data_len-1):
        sum=sum+data[i]
    data[data_len-1]=sum
    #返回打包好的数据
    return data
#__________________________________________________________________



#串口数据解析
def Receive_Anl(data_buf,num):
    #和校验
    sum = 0
    i = 0
    while i<(num-1):
        sum = sum + data_buf[i]
        i = i + 1
    sum = sum%256 #求余
    if sum != data_buf[num-1]:
        return
    #和校验通过
    if data_buf[2]==0xA0:
        #设置模块工作模式
        ctr.work_mode = data_buf[4]
        print(ctr.work_mode)
        print("Set work mode success!")

#__________________________________________________________________
def uart_data_prase(buf):
    if R.state==0 and buf==0xFF:#帧头1
        R.state=1
        R.uart_buf.append(buf)
    elif R.state==1 and buf==0xFE:#帧头2
        R.state=2
        R.uart_buf.append(buf)
    elif R.state==2 and buf<0xFF:#功能字
        R.state=3
        R.uart_buf.append(buf)
    elif R.state==3 and buf<50:#数据长度小于50
        R.state=4
        R._data_len=buf  #有效数据长度
        R._data_cnt=buf+5#总数据长度
        R.uart_buf.append(buf)
    elif R.state==4 and R._data_len>0:#存储对应长度数据
        R._data_len=R._data_len-1
        R.uart_buf.append(buf)
        if R._data_len==0:
            R.state=5
    elif R.state==5:
        R.uart_buf.append(buf)
        R.state=0
        Receive_Anl(R.uart_buf,R.uart_buf[3]+5)
#        print(R.uart_buf)
        R.uart_buf=[]#清空缓冲区，准备下次接收数据
    else:
        R.state=0
        R.uart_buf=[]#清空缓冲区，准备下次接收数据

#__________________________________________________________________



def uart_data_read():
    buf_len=uart.any()
    for i in range(0,buf_len):
        uart_data_prase(uart.readchar())

#————————————————————————————————————————————————————————————————————————

class tof_buf_prase(object):
    uart_buf = []
    _data_len = 0
    _data_cnt = 0
    state = 0


class tof_check(object):
    tof_reserved1=0         #uint8_t
    tof_id=0                #uint8_t
    tof_system_time=0       #uint32_t
    tof_dis=0               #int32_t
    tof_dis_status=0        #uint8_t
    tof_signal_strength=0   #uint16_t
    tof_reserved2=0         #uint8_t


TOF=tof_buf_prase()
tof_target=tof_check()


#串口数据解析
def Receive_Anl_TOF(data_buf,num):
    #和校验
    sum = 0
    i = 0
    while i<(num-1):
        sum = sum + data_buf[i]
        i = i + 1
    sum = sum%256 #求余
    if sum != data_buf[num-1]:
        return
    #和校验通过
    #print("Set work mode success!")
    tof_target.id=data_buf[3];
    tof_target.system_time=data_buf[4]|(data_buf[5]<<8)|(data_buf[6]<<16)|(data_buf[7]<<24)
    tof_target.dis=(int)(data_buf[8]<< 8|data_buf[9]<<16|data_buf[10]<<24)/256;
    tof_target.dis_status=data_buf[11];
    tof_target.signal_strength=data_buf[12]|(data_buf[13]<<8);
    #print(tof_target.system_time,tof_target.dis,tof_target.dis_status,tof_target.signal_strength)
    #将串口1获取的距离传感器数据赋值给串口3发送区
    target.range_sensor4=(int)(tof_target.dis)
    #print(target.range_sensor4)






def uart_tof_prase(buf):
    if TOF.state==0 and buf==0x57:#帧头1
        TOF.state=1
        TOF.uart_buf.append(buf)
    elif TOF.state==1 and buf==0x00:#帧头2
        TOF.state=2
        TOF.uart_buf.append(buf)
    elif TOF.state==2 and buf==0xff:#帧头3
        TOF.state=3
        TOF.uart_buf.append(buf)
    elif TOF.state==3 and buf==0x00:#帧头4
        TOF.state=4
        TOF.uart_buf.append(buf)
        TOF._data_len=11
    elif TOF.state==4 and TOF._data_len>0:#存储对应长度数据
        TOF._data_len=TOF._data_len-1
        TOF.uart_buf.append(buf)
        if TOF._data_len==0:
            TOF.state=5
    elif TOF.state==5:
        TOF.uart_buf.append(buf)
        TOF.state=0
        Receive_Anl_TOF(TOF.uart_buf,16)
#        print(TOF.uart_buf)
        TOF.uart_buf=[]#清空缓冲区，准备下次接收数据
    else:
        TOF.state=0
        TOF.uart_buf=[]#清空缓冲区，准备下次接收数据

def tof_data_read():
    buf_len=uart1.any()
    for i in range(0,buf_len):
        uart_tof_prase(uart1.readchar())


def time_callback(info):
    rgb.red.toggle()

timer=Timer(2,freq=4)
timer.callback(time_callback)


# 绘制水平线
def draw_hori_line(img, x0, x1, y, color):
    for x in range(x0, x1):
        img.set_pixel(x, y, color)
# 绘制竖直线
def draw_vec_line(img, x, y0, y1, color):
    for y in range(y0, y1):
        img.set_pixel(x, y, color)
# 绘制矩形
def draw_rect(img, x, y, w, h, color):
    draw_hori_line(img, x, x+w, y, color)
    draw_hori_line(img, x, x+w, y+h, color)
    draw_vec_line(img, x, y, y+h, color)
    draw_vec_line(img, x+w, y, y+h, color)



blob_threshold_rgb=[0, 30, -10,5, -10, 5]#(L Min, L Max, A Min, A Max, B Min, B Max)
# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# The below thresholds track in general red/green/blue things. You may wish to tune them...
thresholds_rgb = [(30, 100, 15, 127, 15, 127), # generic_red_thresholds
                  (30, 100, -64, -8, -32, 32), # generic_green_thresholds
                  (0, 30, 0, 64, -128, 0)]     # generic_blue_thresholds




#寻色块
def opv_find_color_blob():
    target.flag=0
    if (ctr.work_mode&0x01)!=0:
        img=sensor.snapshot()
        target.img_width=IMAGE_WIDTH
        target.img_height=IMAGE_HEIGHT
        pixels_max=0
        for b in img.find_blobs([thresholds_rgb[0]],pixels_threshold=30,merge=True,margin=50):
            img.draw_rectangle(b[0:4])#圈出搜索到的目标
            if pixels_max<b.pixels():
                pixels_max=b.pixels()
                target.x = b.cx()
                target.y = b.cy()
                target.pixel=pixels_max
                target.reserved1=b.w()>>8
                target.reserved2=b.w()
                target.flag = 1
        if target.flag==1:
            img.draw_cross(target.x,target.y, color=127, size = 15)
            img.draw_circle(target.x,target.y, 15, color = 127)
#        print(target.x,target.y,target.pixel,target.reserved1,target.reserved2)


b=0
#寻Apriltag
def opv_find_april_tag():
    img=sensor.snapshot()
    target.img_width=IMAGE_WIDTH
    target.img_height=IMAGE_HEIGHT
    apriltag_area=0
    apriltag_dis=IMAGE_DIS_MAX
    target.flag = 0
    for tag in img.find_apriltags(): # defaults to TAG36H11 without "families".
        img.draw_rectangle(tag.rect(), color = (255, 0, 0))
        b=(tag.rect()[2]/(abs(math.sin(tag.rotation()))+abs(math.cos(tag.rotation()))))
        #print(tag.rect()[2],b*b)
        #保存最大像素面积得apritag信息
        apriltag_dis_tmp=math.sqrt((tag.cx()-80)*(tag.cx()-80)+(tag.cy()-60)*(tag.cy()-60))
        apriltag_area_tmp=tag.w()*tag.h()
        if apriltag_dis>apriltag_dis_tmp:
            apriltag_area=tag.w()*tag.h()
            target.x = tag.cx()
            target.y = tag.cy()
            target.apriltag_id=tag.id()
            target.pixel=int(b*b)#apriltag_area
            apriltag_dis=apriltag_dis_tmp
            target.flag = 1
    if target.flag==1:
        img.draw_cross(target.x,target.y, color=127, size = 15)
#        img.draw_circle(target.x,target.y, 15, color = 127)
#    print(target.x,target.y,target.pixel,target.apriltag_id,apriltag_dis)



class singleline_check():
    rho_err = 0
    theta_err = 0
    state = 0


singleline = singleline_check()
THRESHOLD = (0,100) # Grayscale threshold for dark things
thresholds =(0, 30, -10,10, -10, 10)
#找线
def found_line():
    target.img_width=IMAGE_WIDTH
    target.img_height=IMAGE_HEIGHT
    target.flag = 0
    #sensor.set_pixformat(sensor.GRAYSCALE)
    #img=sensor.snapshot().binary([THRESHOLD])
    img=sensor.snapshot()
    target.img_width =IMAGE_WIDTH
    target.img_height=IMAGE_HEIGHT
    pixels_max=0
    singleline.state = img.get_regression([thresholds],x_stride=2,y_stride=2,pixels_threshold=10,robust = True)
    if(singleline.state):
        singleline.rho_err = abs(singleline.state.rho())
        singleline.theta_err = singleline.state.theta()
        target.x=singleline.rho_err;
        target.angle=singleline.theta_err;
        target.flag = 1
    if target.flag==1:
        img.draw_line(singleline.state.line(), color = 127)


ctr.work_mode=0x01
last_ticks=0
ticks=0
ticks_delta=0;
while True:
    clock.tick()

    if ctr.work_mode==0x00:#空闲模式
        img=sensor.snapshot()
        rgb.blue.toggle()

    elif ctr.work_mode==0x01:#色块模式
        opv_find_color_blob()
        rgb.blue.off()

    elif ctr.work_mode==0x02:#AprilTag模式
        opv_find_april_tag()
        rgb.blue.off()

    elif ctr.work_mode==0x03:#巡线模式
        found_line()
        rgb.blue.off()

    elif ctr.work_mode==0x04:#AprilTag模式
        opv_find_april_tag()
        rgb.blue.off()

    elif ctr.work_mode==0x05:#预留模式1
        img=sensor.snapshot()
        rgb.blue.toggle()

    elif ctr.work_mode==0x06:#预留模式2
        img=sensor.snapshot()
        rgb.blue.toggle()

    elif ctr.work_mode==0x07:#预留模式3
        img=sensor.snapshot()
        rgb.blue.toggle()
    else:
        rgb.blue.toggle()

    tof_data_read()
    uart.write(package_blobs_data(ctr.work_mode))
    uart_data_read()
    #uart1.write(package_blobs_data(ctr.work_mode))
#__________________________________________________________________
    #计算fps
    last_ticks=ticks
    ticks=time.ticks_ms()#ticks=time.ticks_ms()
                      #新版本OPENMV固件使用time.ticks_ms()
                      #旧版本OPENMV固件使用time.ticks()
    ticks_delta=ticks-last_ticks
    if ticks_delta<1:
        ticks_delta=1
    target.fps=(int)(1000/ticks_delta)
    #target.fps = (int)(clock.fps())
#__________________________________________________________________
    print(target.fps,ticks-last_ticks,target.range_sensor4)

















#    print(image.rgb_to_lab(blob_threshold_rgb))
                       # Update the FPS clock.
#    img = sensor.snapshot()         # Take a picture and return the image.
#    print(clock.fps())              # Note: OpenMV Cam runs about half as fast when connected
#    print(img.get_pixel(30,30))
#    draw_rect(img,30,30,10,10,(255,0,100))
#    print(img.width(),img.height())
#    uart_data_read()
#    uart.write(package_blobs_data())
