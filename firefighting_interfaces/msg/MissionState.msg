uint8 IDLE=0
uint8 PREFLIGHT=1
uint8 TAKEOFF=2
uint8 NAVIGATE_TO_ZONE=3
uint8 SCANNING=4
uint8 FIRE_DETECTED=5
uint8 APPROACHING_FIRE=6
uint8 POSITIONING=7
uint8 DEPLOYING_PAYLOAD=8
uint8 VERIFYING=9
uint8 RTB=10
uint8 LANDING=11
uint8 EMERGENCY=99

uint8 state
string state_description
float32 mission_progress  # 0-100%