#import pymysql  # pymysql 임포트
#from matplotlib import pyplot as plt
#from matplotlib import animation
#import numpy as np
#
## 전역변수 선언부
#conn = None
#cur = None
#
#sql = ""
#
## 메인 코드
#conn = pymysql.connect(host='127.0.0.1', user='admin',
#                       password='gusals97', db='sensor',port = 3306, charset='utf8')  # 접속정보
#cur = conn.cursor()  # 커서생성
#
## 실행할 sql문
#sql = "select * from test"
#cur.execute(sql)  # 커서로 sql문 실행
#print(cur)
#conn.commit()  # 저장
#
#conn.close()  # 종료
#
#

#from matplotlib import pyplot as plt
#from matplotlib import animation
#import numpy as np
#import random
#import time
#
#
#fig = plt.figure()  # figure(도표) 생성
#
#line_2 = plt.plot(np.arange(-100, 100, 1), lw=1, ms=1)
#
#
#
#
#def init_2():
#    return line_2
#
#
#
#
#def animate_2(i):
#    y_2 = random.randint(0, 512)
#    old_y_2 = line_2.axes.get_ydata()
#    new_y_2 = np.r_[old_y_2[1:], y_2]
#    line_2.axes.set_ydata(new_y_2)
#    print(new_y_2)
#    return line_2
#
#
#anim_2 = animation.FuncAnimation(
#    fig, animate_2, init_func=init_2, frames=200, interval=10, blit=False)
#plt.show()
#
#

from numpy import *
import numpy as np
import random
import matplotlib.pyplot as plt
import pymysql  # pymysql 임포트
from matplotlib import animation
import time


fig = plt.figure()
ax_2 = plt.axes(xlim=(0, 100), ylim=(-100, 100))
plt.legend()
line1, = ax_2.plot([],[], lw=2)

max_points = 100
line1, = ax_2.plot(np.arange(max_points), np.ones(max_points, dtype=np.float)*np.nan, lw=2)

conn = None
cur = None

sql = ""

# 메인 코드
conn = pymysql.connect(host='127.0.0.1', user='admin',
                       password='gusals97', db='sensor', port=3306, charset='utf8')  # 접속정보
cur = conn.cursor()  # 커서생성


def init_2():
    return line1
def animate_2(i):
    # 실행할 sql문
    
    # 시간을 초단위로 계산하고 센서값을 리스트에 각각 저장한다.
    sql = "select * from test"
    cur.execute(sql)
    data = cur.fetchall()
    y_2 = (data[-1][1])
    old_y_2 = line1.get_ydata()
    new_y_2 = np.r_[old_y_2[1:], y_2]
    line1.set_ydata(new_y_2)
    return line1
anim_2 = animation.FuncAnimation(fig, animate_2, init_func=init_2, frames=200, interval=10, blit=False)
plt.show()
#    plt.draw(), plt.pause(0.00001)

