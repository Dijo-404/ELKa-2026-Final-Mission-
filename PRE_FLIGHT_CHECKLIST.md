# Pre-Flight Checklist

**Critical checks before real flight:**

## Hardware
- [ ] Scout drone powered on (4S battery, ~14-17V)
- [ ] Delivery drone powered on (6S battery, ~21-25V)
- [ ] Both telemetry radios showing solid link LED
- [ ] GPS lock acquired outdoors (wait 2-3 mins)
- [ ] Props secured/balanced
- [ ] Payload mechanism tested

## Software
- [ ] Run `python tests/test_drone_connection.py` - both drones detected
- [ ] Run `python tests/test_rtsp_stream.py` - video feed working
- [ ] KML file uploaded to `config/survey_area.kml`
- [ ] YOLO model present in `models/best.pt`

## Configuration Verified
- Scout altitude: 10m
- Delivery altitude: 10m  
- Drop altitude: 5m
- Scout battery min: 14.0V
- Delivery battery min: 21.0V
- RTSP URL: rtsp://192.168.144.25:8554/main.264

## Mission Commands

**Scout Mission (survey + detection):**
```bash
cd /home/dj/Projects/ELKa-2026-Final-Mission-
conda activate Nidar
python main.py --scout-only
```

**Delivery Mission (payload drops):**
```bash
python main.py --delivery-only --targets output/targets.json
```

**Full Mission (Scout then Delivery):**
```bash
python main.py
```

## Emergency
- Press `Q` during mission = triggers RTL
- `Ctrl+C` = emergency stop with RTL
- Manual RC can override at any time
