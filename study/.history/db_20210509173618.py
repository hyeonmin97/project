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

from matplotlib import pyplot as plt
from matplotlib import animation
import numpy as np
import random
import time


fig = plt.figure()  # figure(도표) 생성

line_2 = plt.plot(np.arange(-100, 100, 1), lw=1, ms=1)




def init_2():
    return line_2




def animate_2(i):
    y_2 = random.randint(0, 512)
    old_y_2 = line_2.axes.get_ydata()
    new_y_2 = np.r_[old_y_2[1:], y_2]
    line_2.axes.set_ydata(new_y_2)
    print(new_y_2)
    return line_2


anim_2 = animation.FuncAnimation(
    fig, animate_2, init_func=init_2, frames=200, interval=10, blit=False)
plt.show()
