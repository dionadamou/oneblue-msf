##########################################################################################################################################################################################################################################################
# Firmware for Main Control Unit (MCU) of the Sample Preparation Sub-system of the ENVIROMED Wastewater Spectroscopic Analyzer - commands module for serial RS-485 communication (Modbus) between MCU, motor drivers, rotatory valves and flow meter
##########################################################################################################################################################################################################################################################
# VERSION CONTROL:
# Current version:
version_major = 0
version_minor = 0
revision_date = "24-Apr-2024"
author = "Alberto Sposito"

version_string = "Version: " + str(version_major) + "." + str(version_minor) + " (" + revision_date + ")"

print("Firmware for Main Control Unit (MCU) of the Sample Preparation Sub-system of the ENVIROMED Wastewater Spectroscopic Analyzer - commands module for serial RS-485 communication (Modbus) between MCU, motor drivers, rotatory valves and flow meter")
print(version_string)

# Version history:
# V0.0 (24/04/2024 - first draft by Alberto Sposito)
##########################################################################################################################################################################################################################################################

# Import libraries:
import struct
import serial
import binascii
import time
import RPi.GPIO as GPIO

from gpiozero import DigitalInputDevice

# Configure the RS-485 serial port:
ser = serial.Serial(
    port = '/dev/ttySC2',          # Specify the appropriate serial port (ACM0 for RS-485)
    baudrate = 9600,                # Set the baud rate (default: 9600)
    parity = serial.PARITY_NONE,    # Set the parity (none)
    stopbits = serial.STOPBITS_ONE, # Set the stop bits (1 bit)
    bytesize = serial.EIGHTBITS,    # Set the byte size (8 bits)
    timeout = 0.5                   # Set the timeout (1 s)
)

# Module to write command (and read response, in case of DEBUG only!):
def write(command):
    
    # Flush buffer:
    ser.flush()
    
    # Write command:
    ser.write(command)  

    # Read response (for DEBUG only!):

    response = ser.readline()
    
    # Convert response to binary for printing (for DEBUG only!):
    hex_bytes = binascii.hexlify(response)

    # Print response (for DEBUG only!):
    #print("Response (HEX): ")
    #print(hex_bytes)

# Module to send command to read response (Current Position from Rotatory Valve):
def read(command):

    # Flush buffer:
    ser.flush()
    
    # Write command:

    ser.write(command)

    print("Command sent: ",command)

    # Read response: 
    time.sleep(0.1)
    response = ser.readline()
    #print(f"Raw response (bytes): {response}")

    # Print response:
    print("Response (ASCII): ")
    
    print(response)
    
#response = b'/15\r'    
def write_read(command):
    #command = b'/1GO5\r'
    commandS = command[:2] + b'CP\r'        
    while True:
        # Flush buffer:
        ser.flush()
        
        # Write command:

        ser.write(command)
        
        # Read response: 
        time.sleep(0.1)
        
        ser.write(commandS)
        
        # Read response: 
        time.sleep(0.1)
        response = ser.readline()
        print(response)
        
        sendcheck = command[:2] + command[4: ]
        responsecheck = response
        
        if sendcheck == responsecheck:      # Check if the response is equal with the command that has sent
              break
        else:
          print("Valve is at the wrong position")
          print("Resend the command")


# Module to send command to request registers and check if a specific bit (bitN) from statusDword register has the expected Value:
def readBit(command, bitN, Value):   
    while True:  # Keep checking bitN of statusDWord register from motor driver (i.e. motor busy, operation in progress)
        
        # Flush buffer:
        ser.flush()
        
        ser.write(command)
   
        response = ser.read(23) # Read response
        
        # Convert response to binary for printing (for DEBUG only!):
        hex_bytes = binascii.hexlify(response)
        
        # Print response (for DEBUG only!):
        #print("Response: ")
        #print(hex_bytes)

        statusDword = response[3:7]   # Extract statusDword from Response
        
        # Convert statusDword to binary for printing (for DEBUG only!):
        hex_bytes = binascii.hexlify(statusDword)
        
        # Print statusDword (for DEBUG only!):
        #print("StatusDword: ")
        #print(hex_bytes)

        statusDword_int = int.from_bytes(statusDword, byteorder='big')    # Convert statusDword from hexadecimal to integer

        bit = (statusDword_int >> bitN) & 1      #Extract bitN from the statusDword

        # Print bitN from the statusDword (for DEBUG only!):
        #print ("bit " + str(bitN) + " of statusDword register of motor driver:")
        #print (bit)   

        if bit == Value:      # Check if bitN of the statusDword is equal to the expected Value and, if so, stop the while loop by setting i = 1
            break
        
        time.sleep(0.01)   # Delay for 0.01 second

# Module to check feedback from digital signal of motor driver on specified DI pin of MCU:
def readDI(DI_pin):   
    while True:  # Keep checking DI_pin (i.e. bit 15 of statusDWord register from motor driver (i.e. motor busy, operation in progress))

        # Print read value (for DEBUG only!):
        #print ("bit 15 of statusDword register of motor driver:")
        #print (GPIO.input(DI_pin))

        if GPIO.input(DI_pin) == GPIO.HIGH:      # Check if DI_pin is HIGH (i.e. bit 15 of the statusDword is equal to 1) and, if so, stop the while loop:
            break
        
        time.sleep(0.001)   # Delay for 0.001 second

# Module to write command, read response, request registers and check if a specific bit (bitN) from statusDword register has the expected Value:
def write_readBit(commandW, commandR, bitN, Value):

    # Flush buffer:
    ser.flush()

    # Write command: 
    ser.write(commandW)
    # Read response (for DEBUG only!):
    response = ser.readline()
    
    # Convert response to binary for printing (for DEBUG only!):
    hex_bytes = binascii.hexlify(response)

    # Print response (for DEBUG only!):
    #print("Response: ")
    #print(hex_bytes)

    while True:  # Keep checking bitN of statusDWord register from motor driver (i.e. motor busy, operation in progress)

        # Flush buffer:
        ser.flush()

        ser.write(commandR)

        response = ser.read(23) # Read response
        
        # Convert response to binary for printing (for DEBUG only!):
        hex_bytes = binascii.hexlify(response)
        
        # Print response (for DEBUG only!):
        #print("Response: ")
        #print(hex_bytes)

        statusDword = response[3:7]   # Extract statusDword from Response
        
        # Convert statusDword to binary for printing (for DEBUG only!):
        hex_bytes = binascii.hexlify(statusDword)
        
        # Print statusDword (for DEBUG only!):
        #print("StatusDword: ")
        #print(hex_bytes)

        statusDword_int = int.from_bytes(statusDword, byteorder='big')    # Convert statusDword from hexadecimal to integer

        bit = (statusDword_int >> bitN) & 1      #Extract bitN from the statusDword

        # Print bitN from the statusDword (for DEBUG only!):
        #print ("bit " + str(bitN) + " of statusDword register of motor driver:")
        #print (bit)

        if bit == Value:      # Check if bitN of the statusDword is equal to the expected Value and, if so, stop the while loop by setting i = 1
            break
        
        time.sleep(0.01)   # Delay for 0.01 second

# Module to write command, read response and check feedback from digital signal of motor driver on specified DI pin of MCU:
def write_readDI(commandW, DI_pin):

    # Flush buffer:
    ser.flush()

    # Write command:
    ser.write(commandW)

    # Read response (for DEBUG only!):
    response = ser.readline()
    
    # Convert response to binary for printing (for DEBUG only!):
    hex_bytes = binascii.hexlify(response)

    # Print response (for DEBUG only!):
    #print("Response: ")
    #print(hex_bytes)

    while True:  # Keep checking DI_pin (i.e. bit 15 of statusDWord register from motor driver (i.e. motor busy, operation in progress))

        # Print read value (for DEBUG only!):
        #print ("bit 15 of statusDword register of motor driver:")
        #print (GPIO.input(DI_pin))

        if GPIO.input(DI_pin) == GPIO.HIGH:      # Check if DI_pin is HIGH (i.e. bit 15 of the statusDword is equal to 1) and, if so, stop the while loop:
            break
        
        time.sleep(0.001)   # Delay for 0.001 second

# Module to write command, read response and extract specific bytes (for reading flow from flow meter):
def write_readBytes(command):   # IMPORTANT: this commands needs to be fully tested with the flow meter!

    # Flush buffer:
    ser.flush()

    # Write command:
    ser.write(command)

    # Read response:
    response = ser.readline()

    # Print response (for DEBUG only!):
    #print("Response: ")
    #print(response)


    # Convert response to binary for printing (for DEBUG only!):
    hex_bytes = binascii.hexlify(response)

    # Print response (for DEBUG only!):
    #print("Response (HEX): ")
    #print(hex_bytes)

    status = response[3]    # Extract the Device status information for error, i.e byte 3 of response (\x00 for no error)
    
    # Print device status information (for DEBUG only!):
    #print("Status:")
    #print(status)

    flow_hex = response[5:9]    # Extract the data representing the measured flow, i.e. bytes 5 to 8

    # Print extracted data (for DEBUG only!):
    #print("Flow in HEX:")
    #print(flow_hex)

    flow = struct.unpack('>f', flow_hex)[0]     # Convert extracted data from floating point (32-bit single precision) in IEEE-754 format to decimal representation
    
    # Print measured flow in decimal representation:
    #print("Flow:")
    #print(flow)
