#import pymysql  # pymysql 임포트
#
## 전역변수 선언부
#conn = None
#cur = None
#
#sql = ""
#
## 메인 코드
#conn = pymysql.connect(host='192.168.0.24', user='admin',
#                       password='gusals97', db='sensor', charset='utf8')  # 접속정보
#cur = conn.cursor()  # 커서생성
#
## 실행할 sql문
#sql = "select * from test"
#cur.execute(sql)  # 커서로 sql문 실행
#
#conn.commit()  # 저장
#
#conn.close()  # 종료
#

# 패키지 선언
import numpy as np
import matplotlib.pyplot as plt

# 파이썬 실시간 그래프 그리기
x = 0
for i in range(1000):
    x = x + 0.1
    y = np.sin(x)

    plt.scatter(x, y)
    plt.pause(0.001)

plt.show()
