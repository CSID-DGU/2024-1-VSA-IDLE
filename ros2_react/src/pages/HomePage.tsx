import React, { useState, useEffect, useRef } from 'react';
import { 
  AppBar, Toolbar, Typography, Container, Grid, Card, CardHeader, CardContent,
  Button, Select, MenuItem, SelectChangeEvent, ThemeProvider, createTheme, 
  CssBaseline, Box
} from '@mui/material';
import { styled } from '@mui/system';
import ROSLIB, {ActionClient} from 'roslib';
import { Map, Maximize2, Minimize2, Navigation } from 'lucide-react';

type MapType = 'warehouse1' | 'warehouse2';

interface Robot {
  namespace: string;
  destination: string;
  battery: number;
}
interface MapMetaData {
  map_load_time: { secs: number; nsecs: number };
  resolution: number;
  width: number;
  height: number;
  origin: {
    position: { x: number; y: number; z: number };
    orientation: { x: number; y: number; z: number; w: number };
  };
}

interface OccupancyGrid {
  header: {
    seq: number;
    stamp: { secs: number; nsecs: number };
    frame_id: string;
  };
  info: MapMetaData;
  data: number[];
}

// Lucide 아이콘을 MUI 아이콘 스타일로 감싸는 컴포넌트
const IconWrapper = styled('span')(({ theme }) => ({
  display: 'inline-flex',
  marginRight: theme.spacing(1),
  verticalAlign: 'middle',
}));

// 커스텀 테마 생성
const theme = createTheme({
  palette: {
    mode: 'light',
    primary: {
      main: '#424242',
    },
    secondary: {
      main: '#757575',
    },
    background: {
      default: '#f5f5f5',
      paper: '#ffffff',
    },
  },
  typography: {
    fontFamily: " 'Noto Sans KR', sans-serif",
  },
  components: {
    MuiButton: {
      styleOverrides: {
        root: {
          textTransform: 'none',
        },
      },
    },
    MuiCard: {
      styleOverrides: {
        root: {
          boxShadow: '0 4px 6px rgba(0, 0, 0, 0.1)',
          borderRadius: '8px',
        },
      },
    },
  },
});

const StyledBatteryIndicator = styled('span')(({ theme }) => ({
  backgroundColor: theme.palette.grey[200],
  color: theme.palette.grey[800],
  padding: theme.spacing(0.5, 1),
  borderRadius: '16px',
  fontSize: '0.75rem',
  fontWeight: 'medium',
}));

const MapArea = styled(Box)(({ theme }) => ({
  height: '500px',
  backgroundColor: theme.palette.grey[100],
  display: 'flex',
  alignItems: 'center',
  justifyContent: 'center',
  border: `1px solid ${theme.palette.grey[300]}`,
  borderRadius: theme.shape.borderRadius,
  position: 'relative',
}));

const StatusIndicator = styled('span')<{ connected: boolean }>(({ theme, connected }) => ({
  display: 'inline-flex',
  alignItems: 'center',
  color: connected ? theme.palette.success.main : theme.palette.error.main,
  fontWeight: 'bold',
  fontSize : '20px'
}));

const StatusDot = styled('span')<{ connected: boolean }>(({ theme, connected }) => ({
  width: 14,
  height: 14,
  borderRadius: '30%',
  backgroundColor: connected ? theme.palette.success.main : theme.palette.error.main,
  marginRight: theme.spacing(1.5),
}));

export default function HomePage() {
  const [imageSrc, setImageSrc] = useState<string | null>(null);
  const [mapImage, setMapImage] = useState<string|null>(null);
  const [selectedMap, setSelectedMap] = useState<MapType>('warehouse1');
  const [rosConnection, setRosConnection] = useState<boolean>(false);
  const [robots, setRobots] = useState<Robot[]>([
    { namespace: 'tb1', destination: '', battery: 80 },
    { namespace: 'tb2', destination: '', battery: 65 },
    { namespace: 'tb3', destination: '', battery: 90 },
  ]);

  const [ros, setRos] = useState<ROSLIB.Ros | null>(null);
  const [message, setMessage] = useState<string | null>(null);

  function base64ToUint8Array(base64: string): Uint8Array {
    const binaryString = atob(base64);
    const len = binaryString.length;
    const bytes = new Uint8Array(len);
    for (let i = 0; i < len; i++) {
        bytes[i] = binaryString.charCodeAt(i);
    }
    return bytes;
  }
  
  useEffect(() => {
      const rosInstance = new ROSLIB.Ros({
          url: 'ws://localhost:9090'
      });

      rosInstance.on('connection', () => {
          console.log('Connected to ROS');
          setRosConnection(true);
      });

      rosInstance.on('error', (error) => {
          console.log('Error connecting to ROS: ', error);
      });

      rosInstance.on('close', () => {
          console.log('Connection to ROS closed');
      });

      const cameraTopic = new ROSLIB.Topic({
        ros: rosInstance,
        name: '/gazebo/fixed_camera/image_raw',
        messageType: 'sensor_msgs/msg/Image'
      });
  
      cameraTopic.subscribe((message) => {
        const messageData = message as any;
        const base64Data = messageData.data;
        const imageData = base64ToUint8Array(base64Data);
        const width = messageData.width;
        const height = messageData.height;
        const encoding = messageData.encoding;

        const canvas = document.createElement("canvas");
        canvas.width = width;
        canvas.height = height;
        const ctx = canvas.getContext("2d");

        if (ctx) {
            const imgData = ctx.createImageData(width, height);

            let isAllGray = true;
            let minVal = 255, maxVal = 0;

            for (let i = 0; i < width * height; i++) {
                const r = imageData[i * 3];
                const g = imageData[i * 3 + 1];
                const b = imageData[i * 3 + 2];

                imgData.data[i * 4] = r;
                imgData.data[i * 4 + 1] = g;
                imgData.data[i * 4 + 2] = b;
                imgData.data[i * 4 + 3] = 255;

                if (r !== g || g !== b) {
                    isAllGray = false;
                }

                minVal = Math.min(minVal, r, g, b);
                maxVal = Math.max(maxVal, r, g, b);
            }

            ctx.putImageData(imgData, 0, 0);
            setMapImage(canvas.toDataURL());
        }
    });

      
      setRos(rosInstance);
  }, []);

  const handleMapChange = (value: MapType) => {
    setSelectedMap(value);
  };

  const handleDestinationChange = (robotId: string, event: SelectChangeEvent<string>) => {
    const destination = event.target.value;
    setRobots(robots.map(robot => 
      robot.namespace === robotId ? { ...robot, destination } : robot
    ));

    sendGoal(robotId,destination);
  };

  function sendGoal(nameSpace: string, destination : string) {
    console.log("여기")
    if(!ros) return;
    var destinationX = 0.0;
    var destinationY = 0.0;
    const actionName = '/' + nameSpace + '/goal_pose';

    if (destination === 'destination1') {
      destinationX = -1;
      destinationY = -1.5;
    } else if( destination === 'destination2') {
      destinationX = -1;
      destinationY = 1.5;
    }else if( destination === 'destination3') {
      destinationX = 1;
      destinationY = -1.5;
    }else if( destination === 'destination4') {
      destinationX = -3.0;
      destinationY = 1.0;
    }
    else if( destination === 'destination5') {
      destinationX = -3.0;
      destinationY = 0.0;
    }
    else if( destination === 'destination6') {
      destinationX = -3.0;
      destinationY = -1.0;
    }
    
    const goalTopic = new ROSLIB.Topic({
      ros: ros,
      name: actionName,  
      messageType: 'geometry_msgs/PoseStamped'  
    });

    // 목표 메시지를 정의합니다.
    const goalMessage = {
      header: {
        frame_id: 'map'  // 목표를 기준으로 하는 프레임
      },
      pose: {
        position: {
          x: destinationX,  // 목표 위치의 x 좌표
          y: destinationY,  // 목표 위치의 y 좌표
          z: 0.0  // 목표 위치의 z 좌표
        },
        orientation: {
          x: 0.0,  // 목표 방향의 x 성분
          y: 0.0,  // 목표 방향의 y 성분
          z: 0.0,  // 목표 방향의 z 성분
          w: 1.0  // 목표 방향의 w 성분 (쿼터니언)
        }
      }
    };

    // 목표를 전송합니다.
    goalTopic.publish(new ROSLIB.Message(goalMessage));
    goalTopic.unsubscribe();

    console.log('Goal sent:', goalMessage);

    console.log('전송 완료');
  }

  return (
    <ThemeProvider theme={theme}>
      <CssBaseline />
      <Box sx={{ minHeight: '100vh', backgroundColor: 'background.default', width: '100%' }}>
        <AppBar position="static" color="primary" elevation={0}>
          <Toolbar>
            <Typography variant="h6">Fleet Management System</Typography>
          </Toolbar>
        </AppBar>
        <Container maxWidth={false} sx={{ mt: 4, px: 4 }}>
            <Box sx={{ display: 'flex', justifyContent: 'space-between', gap: 2, mb: 3 }}>
              <Box>
                <Button
                  sx={{ px: 4 }}
                  variant={selectedMap === 'warehouse1' ? 'contained' : 'outlined'}
                  onClick={() => handleMapChange('warehouse1')}
                  startIcon={<IconWrapper>1</IconWrapper>}
                >
                  패키징 스테이션
                </Button>
                <Button
                  sx={{ px: 4 }}
                  variant={selectedMap === 'warehouse2' ? 'contained' : 'outlined'}
                  onClick={() => handleMapChange('warehouse2')}
                  startIcon={<IconWrapper>2</IconWrapper>}
                  disabled
                >
                  입하장
                </Button>
              </Box>

              <StatusIndicator connected={rosConnection}>
                <StatusDot connected={rosConnection} />
                {rosConnection ? '연결중' : '연결 없음'}
              </StatusIndicator>
            </Box>

          <Grid container spacing={3}>
            <Grid item xs={12} lg={8}>
              <Card>
                <CardHeader
                  title={
                    <Typography variant="h6">
                      <IconWrapper><Map size={20} /></IconWrapper>
                      ROS2 맵
                    </Typography>
                  }
                />
                <CardContent sx={{ p: 2 }}>
                  <MapArea>
                    {mapImage && (
                        <img
                          src={mapImage}
                          alt="Robot Map"
                          style={{ height: '100%', width: 'auto', margin: 'auto', position: 'absolute',top: 0, bottom: 0, left: 0, right: 0 }}
                        />
                      )}
                  </MapArea>
                </CardContent>
              </Card>
            </Grid>
            
            <Grid item xs={12} lg={4}>
              <Card>
                <CardHeader
                  title={
                    <Typography variant="h6">
                      <IconWrapper><Navigation size={20} /></IconWrapper>
                      로봇 관리
                    </Typography>
                  }
                />
                <CardContent>
                  {robots.map(robot => (
                    <Box key={robot.namespace} sx={{ mb: 3, '&:last-child': { mb: 0 } }}>
                      <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 1 }}>
                        <Typography variant="subtitle1" sx={{ fontWeight: 'medium' }}>
                          {robot.namespace}
                        </Typography>
                        <StyledBatteryIndicator>
                          배터리 {robot.battery}%
                        </StyledBatteryIndicator>
                      </Box>
                      <Select
                        value={robot.destination}
                        onChange={(event: SelectChangeEvent<string>) => handleDestinationChange(robot.namespace, event)}
                        fullWidth
                        size="small"
                      >
                        <MenuItem value="">목적지 선택</MenuItem>
                        <MenuItem value="destination1">A구역</MenuItem>
                        <MenuItem value="destination2">B구역</MenuItem>
                        <MenuItem value="destination3">C구역</MenuItem>
                        <MenuItem value="destination4">E구역</MenuItem>
                        <MenuItem value="destination5">F구역</MenuItem>
                        <MenuItem value="destination6">G구역</MenuItem>
                      </Select>
                    </Box>
                  ))}
                </CardContent>
              </Card>
            </Grid>
          </Grid>
        </Container>
      </Box>
    </ThemeProvider>
  );
};
