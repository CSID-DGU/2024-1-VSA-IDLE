### 수정된 코드 - small world 기준

1. 패키지명을 turtlebot3_multi_robot_small로 수정하였으므로 패키지 디렉토리 경로 수정
```python
turtlebot3_multi_robot = get_package_share_directory("turtlebot3_multi_robot_small")
```

2. world 파일을 직접 만든 small_square.world로 수정
```python
 world = os.path.join(
        turtlebot3_multi_robot, "worlds", "small_square.world"
    )
```

3. 바뀐 world에 따른 로봇 위치 수정
```python
COLS = [0, -1.6]
ROWS = [-0.5, -2.7]
```
![image](https://github.com/user-attachments/assets/ee7839e4-1af9-4574-abfb-d5784e7bcbc3)

4. 로봇 name 부여방식 수정
```python
namespace_x = i
namespace_y = j
if namespace_x < 0:
 namespace_x = "16"
else:
 namespace_x = "0"
if namespace_y > -2:
 namespace_y = "05"
else:
 namespace_y = "2"
name = "turtlebot" + namespace_x + "_" + namespace_y
namespace = "/tb" + namespace_x + "_" + namespace_y
```
