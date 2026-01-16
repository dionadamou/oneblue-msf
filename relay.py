from librpiplc import rpiplc

def main():
    rpiplc.init("RPIPLC_V6", "RPIPLC_38AR")
    rpiplc.pin_mode("R1.3", rpiplc.OUTPUT)

    while True:
        rpiplc.digital_write("R1.3", rpiplc.HIGH)
        rpiplc.delay(1000)

        rpiplc.digital_write("R1.3", rpiplc.LOW)
        rpiplc.delay(1000)
        rpiplc.digital_write("R1.3", rpiplc.LOW)

     

if __name__ == "__main__":
      main()