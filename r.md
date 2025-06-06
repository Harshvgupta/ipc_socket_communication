# IMU IPC Driver

A high-performance Inter-Process Communication (IPC) driver for streaming IMU sensor data between publisher and consumer applications using Unix domain sockets.

## Overview

This project implements a robust publisher-consumer architecture for IMU (Inertial Measurement Unit) data transmission with:
- **Publisher**: Generates and streams random IMU sensor data (accelerometer, gyroscope, magnetometer)
- **Consumer**: Receives IMU data, processes it, and computes orientation (Euler angles + quaternions)
- **Real-time capable**: Configurable frequency up to 500+ Hz
- **Fault-tolerant**: Automatic reconnection, timeout handling, malformed data recovery

## Features

### Core Functionality
- Unix domain socket IPC communication
- 12-value IMU payload matching industry-standard struct format
- Real-time orientation computation (roll, pitch, yaw + quaternions)
- Sensor fusion from accelerometer and magnetometer data
- Gimbal lock detection and warnings

### Robustness
- Comprehensive error handling for network failures, timeouts, malformed data
- Automatic reconnection on publisher/consumer disconnect
- Configurable retry logic and timeout management
- Extensive logging with rotating file handlers
- Built-in test mode for robustness validation

### Configurability
- Fully configurable via CLI arguments
- Adjustable transmission frequency (1-1000+ Hz)
- Multiple log levels (DEBUG, INFO, WARNING, ERROR, CRITICAL)
- Optional CSV output for data analysis
- Heartbeat monitoring support

## Requirements

- **OS**: Debian-based Linux (Ubuntu, Debian)
- **Python**: 3.6+ (uses standard library only)
- **Dependencies**: None (pure Python implementation)

## Quick Start

### 1. Clone Repository
```bash
git clone https://github.com/Harshvgupta/ipc_socket_communication.git
cd ipc_socket_communication
```

### 2. Basic Usage
```bash
# Terminal 1: Start Publisher
python3 publisher.py --socket-path /tmp/imu_socket --frequency-hz 500 --log-level INFO

# Terminal 2: Start Consumer
python3 consumer.py --socket-path /tmp/imu_socket --timeout-ms 100 --log-level INFO
```

## Detailed Usage

### Publisher Commands

#### Basic Publisher
```bash
python3 publisher.py --socket-path /tmp/imu_socket --frequency-hz 100
```

#### Advanced Publisher with Logging
```bash
python3 publisher.py \
    --socket-path /tmp/imu_socket \
    --frequency-hz 500 \
    --log-level DEBUG \
    --log-dir ./logs \
    --heartbeat-every 1000 \
    --test-mode true
```

### Consumer Commands

#### Basic Consumer
```bash
python3 consumer.py --socket-path /tmp/imu_socket --timeout-ms 100
```

#### Advanced Consumer with CSV Output
```bash
python3 consumer.py \
    --socket-path /tmp/imu_socket \
    --timeout-ms 100 \
    --log-level DEBUG \
    --log-dir ./logs \
    --output-csv ./imu_data.csv
```

## Configuration Parameters

### Publisher Parameters

| Parameter | Required | Default | Description |
|-----------|----------|---------|-------------|
| --socket-path | Yes | - | Unix domain socket file path |
| --frequency-hz | No | 100 | IMU data transmission rate (Hz) |
| --log-level | No | INFO | Logging verbosity level |
| --log-dir | No | "" | Directory for rotating log files |
| --heartbeat-every | No | 0 | Send heartbeat every N packets (0=disabled) |
| --test-mode | No | false | Inject malformed data for testing |

### Consumer Parameters

| Parameter | Required | Default | Description |
|-----------|----------|---------|-------------|
| --socket-path | Yes | - | Unix domain socket file path |
| --timeout-ms | No | 100 | Socket read timeout (milliseconds) |
| --log-level | No | INFO | Logging verbosity level |
| --log-dir | No | "" | Directory for rotating log files |
| --output-csv | No | "" | CSV file path for orientation output |

### Log Levels
- `DEBUG`: Detailed packet-level information
- `INFO`: General operational messages  
- `WARNING`: Non-critical issues (timeouts, malformed data)
- `ERROR`: Recoverable errors
- `CRITICAL`: Fatal errors requiring restart

## Data Formats

### IMU Payload Structure
The publisher generates 12-value CSV packets matching this C struct:
```c
typedef struct {
    float xAcc;           // Acceleration X [mg]
    float yAcc;           // Acceleration Y [mg] 
    float zAcc;           // Acceleration Z [mg]
    uint32_t timestampAcc; // Accelerometer timestamp [ms]
    int32_t xGyro;        // Gyro X [mDeg/s]
    int32_t yGyro;        // Gyro Y [mDeg/s]
    int32_t zGyro;        // Gyro Z [mDeg/s] 
    uint32_t timestampGyro; // Gyroscope timestamp [ms]
    float xMag;           // Magnetometer X [mGauss]
    float yMag;           // Magnetometer Y [mGauss]
    float zMag;           // Magnetometer Z [mGauss]
    uint32_t timestampMag; // Magnetometer timestamp [ms]
} __attribute__((packed)) Payload_IMU_t;
```

### Example Payload
```
0.4343,0.9443,0.2225,1721931959,546,797,221,1721931959,0.9756,0.1212,0.8567,1721931959
```

### Sensor Value Ranges
- **Accelerometer**: ±2000 mg (±2g)
- **Gyroscope**: ±250,000 mDeg/s (±250°/s)  
- **Magnetometer**: ±4800 mGauss (±4.8 Gauss)
- **Timestamps**: Milliseconds since Unix epoch

### Consumer Output Formats

#### Console Log Output
```
[2024-06-06 10:30:15] [INFO] IMUConsumer: Orientation | AccTs=1721931959 | Roll=15.23°, Pitch=-8.45°, Yaw=185.67° | Quat=(w=0.9876, x=0.1234, y=-0.0567, z=0.0890)
```

#### CSV Output (if --output-csv specified)
```csv
timestamp_ms,roll_deg,pitch_deg,yaw_deg
1721931959,15.2300,-8.4500,185.6700
1721931960,15.1800,-8.5200,185.7100
```

## Architecture and Design

### Communication Flow
```
Publisher → Unix Socket → Consumer
    ↓             ↓           ↓
 Generate      Transport   Process
 IMU Data      via IPC     & Filter
    ↓             ↓           ↓
 Random        Low-latency  Compute
 Sensors       Reliable     Orientation
```

### Key Design Decisions

1. **Unix Domain Sockets**: Chosen over TCP for lower latency local IPC
2. **Text-based Protocol**: CSV format for easier debugging and cross-platform compatibility
3. **Automatic Reconnection**: Consumer reconnects on publisher restart
4. **Sensor Fusion**: Accelerometer for roll/pitch, magnetometer for yaw
5. **Gimbal Lock Handling**: Detection and warnings when pitch approaches ±90°

### Error Handling Strategy
- **Network Errors**: Automatic reconnection with exponential backoff
- **Malformed Data**: Skip invalid packets, log warnings, continue processing
- **Timeouts**: Configurable timeout with graceful degradation
- **Resource Cleanup**: Proper socket cleanup on shutdown signals

## Testing

### Unit Tests
```bash
# Run comprehensive unit tests
python3 -m unittest test_imu_ipc.py -v
```

### Integration Testing
```bash
# Test with realistic load
python3 publisher.py --socket-path /tmp/test --frequency-hz 500 --test-mode true &
python3 consumer.py --socket-path /tmp/test --timeout-ms 50 --log-level DEBUG
```

### Performance Benchmarking
```bash
# High-frequency stress test
python3 publisher.py --socket-path /tmp/bench --frequency-hz 1000 &
python3 consumer.py --socket-path /tmp/bench --timeout-ms 10
```

## Example Use Cases

### Robotics Applications
```bash
# High-frequency IMU for drone navigation
python3 publisher.py --socket-path /tmp/drone_imu --frequency-hz 500 &
python3 consumer.py --socket-path /tmp/drone_imu --output-csv /var/log/drone_orientation.csv
```

### Development and Debugging
```bash
# Debug mode with detailed logging
python3 publisher.py --socket-path /tmp/debug --frequency-hz 10 --log-level DEBUG --test-mode true &
python3 consumer.py --socket-path /tmp/debug --log-level DEBUG --timeout-ms 1000
```

### Data Collection
```bash
# Long-term data collection
python3 publisher.py --socket-path /tmp/datacollect --frequency-hz 100 &
python3 consumer.py --socket-path /tmp/datacollect --output-csv ./long_term_data.csv --log-dir ./logs
```

## Troubleshooting

### Common Issues

#### "Socket not found" Error
```bash
# Ensure publisher is started first
python3 publisher.py --socket-path /tmp/imu_socket --frequency-hz 100 &
# Wait 1-2 seconds, then start consumer
python3 consumer.py --socket-path /tmp/imu_socket --timeout-ms 100
```

#### Permission Denied
```bash
# Use writable socket path
python3 publisher.py --socket-path ~/imu_socket --frequency-hz 100
```

#### High CPU Usage
```bash
# Reduce frequency for testing
python3 publisher.py --socket-path /tmp/imu_socket --frequency-hz 50
```

#### Missing Log Files
```bash
# Ensure log directory exists and is writable
mkdir -p ./logs
chmod 755 ./logs
python3 consumer.py --socket-path /tmp/imu_socket --log-dir ./logs
```

### Debug Commands
```bash
# Check socket existence
ls -la /tmp/imu_socket

# Monitor socket connections
sudo netstat -xa | grep imu_socket

# Check process status
ps aux | grep publisher
ps aux | grep consumer
```

## Performance Characteristics

### Typical Performance
- **Latency**: < 1ms end-to-end at 100Hz
- **Throughput**: 1000+ packets/second sustained
- **Memory**: < 10MB RSS per process
- **CPU**: < 5% on modern systems at 500Hz

### Optimization Tips
1. **High Frequency**: Use dedicated CPU cores for RT performance
2. **Low Latency**: Minimize timeout values (--timeout-ms 10)
3. **Logging**: Use INFO level for production, DEBUG only for development
4. **File I/O**: Place CSV output on SSD for high-frequency logging

## Advanced Features

### Heartbeat Monitoring
```bash
# Send heartbeat every 1000 packets
python3 publisher.py --heartbeat-every 1000 --socket-path /tmp/imu_socket
```

### Test Mode (Robustness Validation)
```bash
# Inject malformed data every 500 packets
python3 publisher.py --test-mode true --socket-path /tmp/imu_socket
```

### Rotating Logs
```bash
# 10MB log files, keep 5 backups
python3 consumer.py --log-dir ./logs --socket-path /tmp/imu_socket
```

## Mathematical Background

### Orientation Computation

#### Roll and Pitch from Accelerometer
```
roll = atan2(yAcc, zAcc)
pitch = atan2(-xAcc, sqrt(yAcc² + zAcc²))
```

#### Yaw from Magnetometer (Tilt-Compensated)
```
magXc = xMag * cos(pitch) + zMag * sin(pitch)
magYc = xMag * sin(roll) * sin(pitch) + yMag * cos(roll) - zMag * sin(roll) * cos(pitch)
yaw = atan2(-magYc, magXc)
```

#### Euler to Quaternion Conversion
```
qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
```

## Assumptions and Limitations

### Assumptions Made
1. **Coordinate System**: Standard NED (North-East-Down) orientation
2. **Sensor Alignment**: All sensors perfectly aligned (no calibration matrix)
3. **Static Calibration**: No runtime bias compensation
4. **Magnetic Declination**: Not compensated (raw magnetic north)
5. **Local Gravity**: Assumes standard gravity (9.81 m/s²)

### Current Limitations
1. **No Real-time Scheduling**: Standard process priority (not RT tasks)
2. **Simple Sensor Fusion**: No Kalman filtering or advanced fusion
3. **No Sensor Calibration**: Raw sensor values used directly
4. **Single Consumer**: Publisher supports one consumer at a time
5. **Text Protocol**: CSV overhead vs. binary protocols

### Future Enhancements
- Real-time task scheduling (SCHED_FIFO)
- Kalman filter implementation
- Multi-consumer support
- Binary protocol option
- Sensor calibration routines
- Web-based monitoring dashboard

## License

This project is provided as-is for educational and development purposes.

## Contributing

1. Fork the repository
2. Create a feature branch (git checkout -b feature/amazing-feature)
3. Commit your changes (git commit -m 'Add amazing feature')
4. Push to the branch (git push origin feature/amazing-feature)
5. Open a Pull Request

## Support

For issues, questions, or contributions, please:
1. Check existing issues in the repository
2. Create a new issue with detailed description
3. Include log outputs and system information
4. Provide minimal reproduction steps

---

**Author**: Harsh Gupta  
**Repository**: https://github.com/Harshvgupta/ipc_socket_communication  
**Last Updated**: June 2024