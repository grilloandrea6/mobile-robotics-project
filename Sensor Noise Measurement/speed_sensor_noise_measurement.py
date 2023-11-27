
timer_period[0]=10 #timer1 will fire 100 times per second
toggle0=0

leds_top = [32,0,32]
# Set the motors to a constant speed (in the range [-500, 500])
motor_left_target= -500
motor_right_target= -500
i=0
@onevent
def timer0():
    global i
    i=i+1
    if (i<=5000):
        print(motor_left_speed,motor_right_speed)
    elif (i == 5000 + 1):
        print('Done!')