
import sensor, image, time, pyb, machine
from pyb import UART
from pyb import LED
from pyb import Pin
uart = UART(1, 9600, timeout_char=1000)
red_led   = LED(1)
green_led = LED(2)
blue_led  = LED(3)


#p_in = Pin('P8', Pin.IN, Pin.PULL_UP)
p_in2 = Pin('P2', Pin.IN, Pin.PULL_DOWN)
p_in3 = Pin('P3', Pin.IN, Pin.PULL_DOWN)


thresholds = [(37, 63, 33, 86, 0, 75), # generic_red_thresholds
             (37, 79, -53, -19, 4, 29), # generic_green_thresholds(25, 43, -41, -12, -25, 20)(21, 64, -47, -19, 7, 28)
              (62, 76, -29, -13, -47, -8),#mid
              (33, 54, 5, 86, -1, 71),#rightred
              (15, 51, -53, -17, -6, 33),#rightgreen
              (17, 49, -16, 30, -62, -18)] #rightbllue

#环
#(52, 92, 127, 7, -7, 40)red
#(89, 60, -79, -9, -1, 40)GREEN
#(100, 1, 11, 127, 33, -110)blue
sensor.reset()       #初始化感光器件
sensor.set_pixformat(sensor.RGB565)#设置为彩色 每个像素16位 ，GRAYSCALE为灰度，每个像素为8位
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 500)
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking

green_led.on()
red_led.on()
blue_led.on()
global a   #存物块
global b   #存圆环
a = ['0', '0', '0']
b = ['0', '0', '0']
time.sleep(500)
while(True):
#  red_threshold = (45, 66, 10, 90, -21, 76)     这个阈值非常好 ，就用这个，这个基本不变了
#  green_threshold =  (20, 92, -62, -19, -16, 31)
#  blue_threshold =  (22, 83, -27, 59, -76, -14)


#这个是圆环的阈值，还得调
#thresholds = [(16, 94, 24, 124, -25, 65), # generic_red_thresholds
 #             (20, 92, -62, -19, -16, 31c), # generic_green_thresholds
 #             (0, 85, -22, 77, -74, -47) ]# generic_blue_threshold
    p_in2 = Pin('P2', Pin.IN, Pin.PULL_DOWN)
    p_in3 = Pin('P3', Pin.IN, Pin.PULL_DOWN)
    value2 = p_in2.value()
    value3 = p_in3.value()

    #默认全是0
    ##啥几把也不是
    #11 发三色
    #10 发灰度二维码

    #01 发单色
    #00二维码
    #10三色
    if value2 == 1 and value3 ==1: #啥几把也不是
        sensor.set_pixformat(sensor.RGB565)#设置为彩色 每个像素16位 ，GRAYSCALE为灰度，每个像素为8位
        sensor.set_framesize(sensor.QQVGA)
        img = sensor.snapshot().lens_corr(1.8)
        area1 = (55,65,32,47)
        area2 = (0,0,160,120)
        statistics1 = img.get_statistics(roi=area1)#像素颜色统计
        statistics2 = img.get_statistics(roi=area2)#像素颜色统计
        blue_led.on()
        red_led.on()
        green_led.on()
    elif value2 == 1 and value3 ==0:#11 发三色   发物块
        sensor.set_pixformat(sensor.RGB565)#设置为彩色 每个像素16位 ，GRAYSCALE为灰度，每个像素为8位
        sensor.set_framesize(sensor.QQVGA)
        img = sensor.snapshot().lens_corr(1.8)
        area1 = (55,50,32,60)
        area2 = (0,0,160,120)
        statistics1 = img.get_statistics(roi=area1)#像素颜色统计
        statistics2 = img.get_statistics(roi=area2)#像素颜色统计
        if (45<statistics1.l_mode()<66 and 10<statistics1.a_mode()<90 and -21<statistics1.b_mode()<76)or(11<statistics1.l_mode()<68 and 28<statistics1.a_mode()<76 and 13<statistics1.b_mode()<70):#if the circle is red
            if a[0]=='0':
                a[0]='1'
            elif a[0]!='0' and a[0]!='1' and a[1]=='0':
                a[1]='1'
            elif a[0]!='0' and a[1]!='0' and a[0]!='1' and a[1]!='1' and a[2]=='0':
                a[2]='1'
        elif (20<statistics1.l_mode()<92 and -62<statistics1.a_mode()<-19 and -16<statistics1.b_mode()<31)or(0<statistics1.l_mode()<99 and -65<statistics1.a_mode()<-40 and 10<statistics1.b_mode()<51):#if the circle is green
            if a[0]=='0':
                a[0]='2'
            elif a[0]!='0' and a[0]!='2' and a[1]=='0':
                a[1]='2'
            elif a[0]!='0' and a[1]!='0' and a[0]!='2' and a[1]!='2' and a[2]=='0':
                a[2]='2'
        elif (22<statistics1.l_mode()<83 and -27<statistics1.a_mode()<59 and -76<statistics1.b_mode()<-14)or(0<statistics1.l_mode()<100 and -128<statistics1.a_mode()<127 and -64<statistics1.b_mode()<-16):#if the circle is blue
            if a[0]=='0':
                a[0]='3'
            elif a[0]!='0' and a[0]!='3' and a[1]=='0':
                a[1]='3'
            elif a[0]!='0' and a[1]!='0' and a[0]!='3' and a[1]!='3' and a[2]=='0':
                a[2]='3'
        else:
            img.draw_rectangle(area1, color = (255, 255, 255))
        if a[0]!='0' and a[1]!='0' and a[2]!='0':
            uart.write('s')
            uart.write('t')
            uart.write(a[0])
            uart.write(a[1])
            uart.write(a[2])
            print(a[0])
            print(a[1])
            print(a[2])
        blue_led.on()
        red_led.off()
        green_led.off()
    elif value2 == 0 and value3 ==1:  #扫描圆环的，阈值可能还需要改
        sensor.set_pixformat(sensor.RGB565)#设置为彩色 每个像素16位 ，GRAYSCALE为灰度，每个像素为8位
        sensor.set_framesize(sensor.QQVGA)
        img = sensor.snapshot().lens_corr(1.8)
        area1 = (55,65,32,47)
        area2 = (30,30,90,70)
        statistics1 = img.get_statistics(roi=area1)#像素颜色统计
        statistics2 = img.get_statistics(roi=area2)#像素颜色统计
        if (45<statistics2.l_mean()<66 and 10<statistics2.a_mean()<90 and -21<statistics2.b_mean()<76)or(11<statistics2.l_mean()<68 and 28<statistics2.a_mean()<76 and 13<statistics2.b_mean()<70):#if the circle is red
            uart.write("co1")
            print(1)
        elif (20<statistics2.l_mean()<92 and -62<statistics2.a_mean()<-19 and -16<statistics2.b_mean()<31)or(0<statistics2.l_mean()<99 and -65<statistics2.a_mean()<-40 and 10<statistics2.b_mean()<51)or(35<statistics2.l_mean()<60 and -69<statistics2.a_mean()<-5 and -66<statistics2.b_mean()<54):#if the circle is green
            uart.write("co2")
            print(2)
        elif (7<statistics2.l_mean()<46 and -74<statistics2.a_mean()<55 and -44<statistics2.b_mean()<-2)or(0<statistics2.l_mean()<100 and -128<statistics2.a_mean()<127 and -64<statistics2.b_mean()<-16):#if the circle is blue
            uart.write("co3")
            print(3)
            #(20<statistics2.l_mode()<92 and -62<statistics2.a_mode()<-19 and -16<statistics2.b_mode()<31)or(42<statistics2.l_mode()<74 and -19<statistics2.a_mode()<-5 and 14<statistics2.b_mode()<38)
        else:
            img.draw_rectangle(area2, color = (255, 255, 255))
        blue_led.off()
        red_led.on()
        green_led.on()
    elif value2 == 0 and value3 ==0:
        sensor.set_pixformat(sensor.GRAYSCALE)   #最稳定的二维码程序
        sensor.set_framesize(sensor.QVGA)
        sensor.set_brightness(-3)
        #sensor.set_contrast(0)
        sensor.skip_frames(10)
        img = sensor.snapshot()
        img.lens_corr(1.8)
        for code in img.find_qrcodes():
            print(code)
            m=code[4:5]
            uart.write(str(m))  #发送二维码信息
            blue_led.on()
            red_led.off()
            green_led.on()
        blue_led.on()
        red_led.on()
        green_led.off()

    green_led.off()
    red_led.off()
    blue_led.off()
