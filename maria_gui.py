import time
import commands
import sys


#Commands

command_homing1            = b'\x4E\x10\xA7\x9E\x00\x07\x0E\x07\x00\x00\x03\x01\xF4\x00\x00\x03\xE8\x00\x00\x27\x10\xF8\x65'
command_homing2            = b'\x4E\x10\xA7\x9E\x00\x07\x0E\x07\x00\x02\x03\x01\xF4\x00\x00\x03\xE8\x00\x00\x27\x10\x01\xA2'

command_start_roller       = b'\x01\x06\x00\x07\x01\x00\x39\x9B'
command_stop_roller        = b'\x01\x06\x00\x07\x02\x00\x39\x6B'

command_base_back_on       = b'\x01\x06\x00\x04\x01\x00\xC9\x9B'
command_base_back_off      = b'\x01\x06\x00\x04\x02\x00\xC9\x6B'  

command_move_base_front_on   = b'\x01\x06\x00\x08\x01\x00\x09\x98'
command_move_base_front_off  = b'\x01\x06\x00\x08\x02\x00\x09\x68'


command_stepper_press      = b'\x4E\x10\xA7\x9E\x00\x07\x0E\x01\x00\x00\x03\x03\xE8\x00\x00\x4E\x20\x00\x89\x54\x40\xDD\x82'
command_stepper_open      = b'\x4E\x10\xA7\x9E\x00\x07\x0E\x01\x00\x00\x03\x03\xE8\x00\x00\x4E\x20\x00\x00\x00\x00\x33\x58'

command_open_valve1        = b'\x01\x06\x00\x01\x01\x00\xD9\x9A'
command_open_valve2        = b'\x01\x06\x00\x02\x01\x00\x29\x9A'
command_open_valve3        = b'\x01\x06\x00\x03\x01\x00\x78\x5A'
command_open_valve4        = b'\x01\x06\x00\x04\x01\x00\xC9\x9B'
command_open_valve5        = b'\x01\x06\x00\x05\x01\x00\x98\x5B'  # relay 5 ON



command_start_pump         = b'\x01\x06\x00\x06\x01\x00\x68\x5B'
command_stop_pump          = b'\x01\x06\x00\x06\x02\x00\x68\xAB'





command_close_valve1       = b'\x01\x06\x00\x01\x02\x00\xD9\x6A'
command_close_valve2       = b'\x01\x06\x00\x02\x02\x00\x29\x6A'
command_close_valve3       = b'\x01\x06\x00\x03\x02\x00\x78\xAA'
command_close_valve4       = b'\x01\x06\x00\x04\x02\x00\xC9\x6B'
command_close_valve5       = b'\x01\x06\x00\x05\x02\x00\x98\xAB'

# ------- RPIPLC analog out (DAC) helpers -------
# Safe to import even if librpiplc isn't present (it'll just no-op).
try:
    from librpiplc import rpiplc
    _AO_AVAILABLE = True
except Exception:
    rpiplc = None
    _AO_AVAILABLE = False

AO_PIN = "A0.5"   # change if you use another analog output
AO_READY = False  # becomes True after init

def ao_init():
    """Initialize RPIPLC and prepare the analog output pin."""
    global AO_READY
    if not _AO_AVAILABLE:
        print("[AO] librpiplc not available: DAC will be NO-OP.")
        AO_READY = False
        return
    try:
        rpiplc.init("RPIPLC_V6", "RPIPLC_21")
        rpiplc.pin_mode(AO_PIN, rpiplc.OUTPUT)
        AO_READY = True
        print(f"[AO] Ready on {AO_PIN}")
    except Exception as e:
        print(f"[AO] init failed: {e}")
        AO_READY = False

def ao(value: int):
    """Write a DAC value 0..4095 to the AO pin. Clamps automatically."""
    v = max(0, min(4095, int(value)))
    if AO_READY and _AO_AVAILABLE:
        try:
            rpiplc.analog_write(AO_PIN, v)
            # Optional: print(f"[AO] {AO_PIN} = {v}")
        except Exception as e:
            print(f"[AO] write failed: {e}")
    else:
        # Safe no-op when not initialized / library missing
        print(f"[AO] (no-op) {AO_PIN} = {v} (not initialized)")





def initialisation():
    
    
    print("Homing 1")
    commands.write(command_homing1)

    print("Homing 2")
    commands.write(command_homing2)
    
    time.sleep(120)  #give time for plates to open    
    
    
    
    
def filter_loading():    
    
    print("Start roller (relay 7 ON)")
    #commands.write(command_start_roller)
    
    time.sleep(6)

    print("Stop roller (relay 7 OFF)")
    #commands.write(command_stop_roller)

    print("Move base back (relay 4 ON)")
    commands.write(command_base_back_on)
    time.sleep(10)

    print("Stop base movement (relay 4 OFF)")
    commands.write(command_base_back_off)

    print("Start plate pressing")
    commands.write(command_stepper_press)
    
    time.sleep(70)
    
def sample_filtration():

    ao(2000)
       
    print("Open valve 1 (Sample Inlet)")
    commands.write(command_open_valve1)

    print("Open valve 2 (Closed Air Inlet)")
    commands.write(command_open_valve2)
    
    print("Open valve 5 (Sample out)")
    commands.write(command_open_valve5)


    print("Start peristaltic pump")
    commands.write(command_start_pump)
    
    #Need to change this with the pump calculations/characterisations
    time.sleep(60)                        

    print("Stop peristaltic pump")
    commands.write(command_stop_pump)    
    
    print("Close valve 1")
    commands.write(command_close_valve1)
    time.sleep(0.3)
    print("Close valve 2")
    commands.write(command_close_valve2)
    time.sleep(0.3)
    print("Close valve 5")
    commands.write(command_close_valve5)
    time.sleep(0.3)
    
    
    
    
    
def cleaning():
    ao(2000)
    #MSF_empty
    commands.write(command_start_pump)
    time.sleep(120)
    commands.write(command_open_valve3)
    time.sleep(120)
    commands.write(command_open_valve4)
    time.sleep(120)
    commands.write(command_stop_pump)
    time.sleep(60)
    commands.write(command_close_valve3)
    commands.write(command_close_valve4)
    
    
    
    #MSF_cleaning
    commands.write(command_stepper_open)
    time.sleep(30)
    commands.write(command_move_base_front_on)
    time.sleep(9)
    commands.write(command_move_base_front_off)
    commands.write(command_stepper_press)
    time.sleep(30)
    commands.write(command_open_valve2)
    
    commands.write(command_start_pump)
    time.sleep(240)
    commands.write(command_close_valve2)
    time.sleep(120)
    commands.write(command_stop_pump)
    commands.write(command_stepper_open)
    time.sleep(30)
    
     
    
    
     
    
    
    
    
    
    
    
if __name__ == "__main__":
    ao_init()
    initialisation()
    filter_loading()
    sample_filtration()    
    cleaning()