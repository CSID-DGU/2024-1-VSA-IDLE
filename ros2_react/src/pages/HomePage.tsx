import React, { useState } from 'react';
import { 
  AppBar, Toolbar, Typography, Container, Grid, Card, CardHeader, CardContent,
  Button, Select, MenuItem, SelectChangeEvent, ThemeProvider, createTheme, 
  CssBaseline, Box
} from '@mui/material';
import { styled } from '@mui/system';
import { Map, Maximize2, Minimize2, Navigation } from 'lucide-react';

type MapType = 'narrow' | 'wide';

interface Robot {
  id: number;
  name: string;
  destination: string;
  battery: number;
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
}));

export default function HomePage() {
    
  const [selectedMap, setSelectedMap] = useState<MapType>('narrow');
  const [robots, setRobots] = useState<Robot[]>([
    { id: 1, name: 'Robot 1', destination: '', battery: 80 },
    { id: 2, name: 'Robot 2', destination: '', battery: 65 },
    { id: 3, name: 'Robot 3', destination: '', battery: 90 },
    { id: 4, name: 'Robot 4', destination: '', battery: 90 },
  ]);

  const handleMapChange = (value: MapType) => {
    setSelectedMap(value);
  };

  const handleDestinationChange = (robotId: number, event: SelectChangeEvent<string>) => {
    const destination = event.target.value;
    setRobots(robots.map(robot => 
      robot.id === robotId ? { ...robot, destination } : robot
    ));
  };

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
            <Box sx={{ display: 'flex', gap: 2, mb: 3 }}>
                        <Button
                        sx={{ px: 4 }}
                        variant={selectedMap === 'narrow' ? 'contained' : 'outlined'}
                        onClick={() => handleMapChange('narrow')}
                        startIcon={<IconWrapper><Minimize2 size={20} /></IconWrapper>}
                        >
                        좁은 맵
                        </Button>
                        <Button
                        sx={{ px: 4 }}
                        variant={selectedMap === 'wide' ? 'contained' : 'outlined'}
                        onClick={() => handleMapChange('wide')}
                        startIcon={<IconWrapper><Maximize2 size={20} /></IconWrapper>}
                        >
                        넓은 맵
                        </Button>
                
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
                    <Typography variant="body1" sx={{ color: 'text.secondary', fontWeight: 'medium' }}>
                      {selectedMap === 'narrow' ? '좁은 맵' : '넓은 맵'} 표시 영역
                    </Typography>
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
