import {useEffect, useState} from 'react';
import ROSLIB from 'roslib';

import {Container} from "@mui/material";

export default function MainPage() {
    const [ros, setRos] = useState<ROSLIB.Ros | null>(null);
    const [message, setMessage] = useState<string | null>(null);

    useEffect(() => {
        const rosInstance = new ROSLIB.Ros({
            url: 'ws://localhost:9090'
        });

        rosInstance.on('connection', () => {
            console.log('Connected to ROS');
        });

        rosInstance.on('error', (error) => {
            console.log('Error connecting to ROS: ', error);
        });

        rosInstance.on('close', () => {
            console.log('Connection to ROS closed');
        });

        const listener = new ROSLIB.Topic({
            ros: rosInstance,
            name : '/move_base/NavfnROS/plan',
            messageType : 'nav_msgs/Path'
        });

        const moveBaseFB = new ROSLIB.Topic ({
            ros : rosInstance,
            name : '/move_base/feedback',
            messageType : 'move_base_msgs/MoveBaseActionFeedback'
            });

        listener.subscribe((msg: ROSLIB.Message) => {
            setMessage((msg as any).data);
        });

        setRos(rosInstance);

        return () => {
            listener.unsubscribe();
            rosInstance.close();
        };

    }, []);

    return (
        <Container>
            <h1>ROS2 시뮬레이션 데이터</h1>
            <p>{message ? `Received message: ${message}` : 'No data received yet.'}</p>
        </Container>
    );
}