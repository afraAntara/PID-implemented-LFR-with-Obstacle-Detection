from controller import Robot

def _clamp(value, limits):
    lower, upper = limits
    if value is None:
        return None
    elif (upper is not None) and (value > upper):
        return upper
    elif (lower is not None) and (value < lower):
        return lower
    return value


TIME_STEP = 32
MAX_SPEED = 3.5
BASE_SPEED = 3

robot = Robot()

#camera

cm=robot.getDevice("camera")
cm.enable(TIME_STEP)
cm.recognitionEnable(TIME_STEP);
recog=False

# ds sensor
ds = []
dsNames = ['ds_right', 'ds_left']
for i in range(2):
    ds.append(robot.getDevice(dsNames[i]))
    ds[i].enable(TIME_STEP)

avoidObstacleCounter = 0


#ir sensor
ir = []
irNames = ['right_sensor','mid_sensor','left_sensor']
for i in range(3):
    ir.append(robot.getDevice(irNames[i]))
    ir[i].enable(TIME_STEP)    
    
wheels = []
wheelsNames = ['right_back_wheel', 'left_back_wheel', 'right_front_wheel', 'left_front_wheel']

for i in range(4):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('+inf'))
    wheels[i].setVelocity(0.0)
    
    
Kp = 20
Ki = 2.5
Kd = 15
P = 0
I = 0
D = 0
lastError = 0

while robot.step(TIME_STEP) != -1:
    motorspeed = 0
    right_ir_val = ir[0].getValue()
    mid_ir_val = ir[1].getValue()
    left_ir_val = ir[2].getValue()

    position = right_ir_val/1000 + mid_ir_val/1000
    error = (left_ir_val/1000 + mid_ir_val/1000) - position
    

    P = error
    I = _clamp((I + error),(-4,4))
    D = _clamp(error - lastError,(-4,4))
    lastError = error
    
    motorspeed = _clamp((P*Kp) + (I*Ki) + (D*Kd),(-6.5,6.5))
    print("P: {}, I: {}, D: {}, speed: {}".format(P,I,D,motorspeed))
    
    
    if (motorspeed < -MAX_SPEED):
        motorspeed = MAX_SPEED
    elif (motorspeed > MAX_SPEED):
        motorspeed = -MAX_SPEED
    else:
        motorspeed = 0
        
    rightSpeed = BASE_SPEED + motorspeed;
    leftSpeed = BASE_SPEED - motorspeed;
    
    for i in range(2):
        if ds[i].getValue() < 950.0:
            avoidObstacleCounter = 210

    if (avoidObstacleCounter > 0):
        avoidObstacleCounter -= 1
        leftSpeed = 1.0
        rightSpeed = -1.0
        wheels[0].setVelocity(leftSpeed)
        wheels[1].setVelocity(rightSpeed)
        wheels[2].setVelocity(leftSpeed)
        wheels[3].setVelocity(rightSpeed)
    
    else:
        
        if (leftSpeed > MAX_SPEED):
            leftSpeed = MAX_SPEED
        if (rightSpeed > MAX_SPEED):
            rightSpeed = MAX_SPEED
        if (leftSpeed < 0):
            leftSpeed = 0
        if (rightSpeed < 0):
            rightSpeed = 0
    
    
                  
        wheels[0].setVelocity(rightSpeed)
        wheels[2].setVelocity(rightSpeed)
        wheels[1].setVelocity(leftSpeed)
        wheels[3].setVelocity(leftSpeed) 
        print(" right: {}, left: {}".format(rightSpeed,leftSpeed))
    