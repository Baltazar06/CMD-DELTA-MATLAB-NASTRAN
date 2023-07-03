a = arduino('COM4', 'Uno', 'Libraries', 'Adafruit/PWMServoDriver')
pwm_shield = addon(a, 'Adafruit/PWMServoDriver')

for i = 0:8:4096
    for servo = 1:4
        pwm_shield.setPWM(servo,0,i+mod((4096/16)*servo,4096))
    end
end