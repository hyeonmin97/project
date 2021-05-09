from numpy import *
import numpy as np
import random
import matplotlib.pyplot as plt
import pymysql  # pymysql 임포트
from matplotlib import animation
import time

conn = None
cur = None

fig = plt.figure()
ax_2 = plt.axes(xlim=(0, 100), ylim=(-100, 100))

max_points = 100
line1, = ax_2.plot(np.arange(max_points), np.ones(max_points, dtype=np.float)*np.nan, lw=2)



def init_2():
    return line1

def animate_2(i):
    sql = "SELECT * FROM test ORDER BY num DESC LIMIT 1;"
    conn = pymysql.connect(host='127.0.0.1', user='admin',
                        password='gusals97', db='sensor', port=3306, charset='utf8')  # 접속정보
    # 커서생성
    cur = conn.cursor()
    cur.execute(sql)
    data = cur.fetchone()
    try:
        if type(data) == "<class 'NoneType'>":
            return 1 
        print(data[1])
        y_2 = (data[1])
        old_y_2 = line1.get_ydata()
        new_y_2 = np.r_[old_y_2[1:], y_2]
        line1.set_ydata(new_y_2)
        return line1
    except TypeError as e:
        print(e)
anim_2 = animation.FuncAnimation(fig, animate_2, init_func=init_2, frames=200, interval=0.1, blit=False)
plt.show()


