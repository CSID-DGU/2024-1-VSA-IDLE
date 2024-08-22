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
  id: number;
  name: string;
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
    { id: 1, name: 'Robot 1', destination: '', battery: 80 },
    { id: 2, name: 'Robot 2', destination: '', battery: 65 },
    { id: 3, name: 'Robot 3', destination: '', battery: 90 },
    { id: 4, name: 'Robot 4', destination: '', battery: 90 },
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

        console.log("Image received:", { width, height, encoding, dataLength: imageData.length });

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

  const handleDestinationChange = (robotId: number, event: SelectChangeEvent<string>) => {
    const destination = event.target.value;
    setRobots(robots.map(robot => 
      robot.id === robotId ? { ...robot, destination } : robot
    ));

    sendGoal(destination);
  };

  function sendGoal(destination : string) {
    if(!ros) return;


    const actionClient = new ROSLIB.ActionClient({
        ros: ros,
        serverName: '/navigate_to_pose',
        actionName: 'nav2_msgs/action/NavigateToPose'
    });

    const goal = new ROSLIB.Goal({
        actionClient: actionClient,
        goalMessage: {
            pose: {
                header: {
                    frame_id: 'map'
                },
                pose: {
                    position: {
                        x: 0.0,
                        y: 10.0,
                        z: 0.0
                    },
                    orientation: {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                        w: 1.0
                    }
                }
            }
        }
    });

    goal.on('status', (status) => {
      console.log('Goal status: ' + status.status);
      if (status.status === 5) { // 5 is the status code for REJECTED
          console.error('Goal was rejected');
      }
  });

    goal.on('result', function (result: any) {
        console.log('Goal result: ', result);
    });

    goal.on('feedback', (feedback) => {
      console.log('Received feedback:', feedback);
  });

    goal.on('timeout', () => {
        console.error('Action did not complete before timeout');
    });

    goal.send();

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
                    <Box key={robot.id} sx={{ mb: 3, '&:last-child': { mb: 0 } }}>
                      <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 1 }}>
                        <Typography variant="subtitle1" sx={{ fontWeight: 'medium' }}>
                          {robot.name}
                        </Typography>
                        <StyledBatteryIndicator>
                          배터리 {robot.battery}%
                        </StyledBatteryIndicator>
                      </Box>
                      <Select
                        value={robot.destination}
                        onChange={(event: SelectChangeEvent<string>) => handleDestinationChange(robot.id, event)}
                        fullWidth
                        size="small"
                      >
                        <MenuItem value="">목적지 선택</MenuItem>
                        <MenuItem value="destination1">목적지 1</MenuItem>
                        <MenuItem value="destination2">목적지 2</MenuItem>
                        <MenuItem value="destination3">목적지 3</MenuItem>
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
