import pymysql  # pymysql 임포트

# 전역변수 선언부
conn = None
cur = None

sql = ""

# 메인 코드
conn = pymysql.connect(host='127.0.0.1', user='admin',
                       password='gusals97', db='sensor', charset='utf8')  # 접속정보
cur = conn.cursor()  # 커서생성

# 실행할 sql문
sql = "select * from test"
cur.execute(sql)  # 커서로 sql문 실행

conn.commit()  # 저장

conn.close()  # 종료

