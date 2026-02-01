# Snowbets - Robot Betting Platform

A web-based interactive platform where users bet "Snow" (virtual currency) on robot performance.

## Features
- **Snow Currency**: Solana-inspired ledger-backed virtual currency
- **Real-time Betting**: Bet on robot sensor data and performance
- **Live Robot Data**: Color sensor, ultrasonic, speed, PID metrics

## Bet Types
1. **Color Detection** - Bet on what color the robot detects next
2. **Distance Threshold** - Bet if ultrasonic reads above/below threshold
3. **Speed Prediction** - Bet on robot speed ranges
4. **Line Follow Success** - Bet if robot stays on line for X seconds

## Setup
```bash
cd Snowbets
pip install -r requirements.txt
python app.py
```

Then open http://localhost:5000

## Architecture
- `app.py` - Flask backend with SocketIO for real-time updates
- `snow_ledger.py` - Virtual currency ledger system
- `betting_engine.py` - Betting logic and odds calculation
- `robot_interface.py` - Serial communication with Arduino robot
- `static/` - Frontend HTML/CSS/JS
- `templates/` - Jinja2 templates
