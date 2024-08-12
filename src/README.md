### 수정된 코드 - small world 기준

1. 패키지명을 turtlebot3_multi_robot_small로 수정하였으므로 패키지 디렉토리 경로 수정
```
turtlebot3_multi_robot = get_package_share_directory("turtlebot3_multi_robot_small")
```

2. world 파일을 직접 만든 small_square.world로 수정
```
 world = os.path.join(
        turtlebot3_multi_robot, "worlds", "small_square.world"
    )
```

3. 바뀐 world에 따른 로봇 위치 수정
```
COLS = [0, -1.6]
ROWS = [-0.5, -2.7]
```
![image](https://github.com/user-attachments/assets/ee7839e4-1af9-4574-abfb-d5784e7bcbc3)

