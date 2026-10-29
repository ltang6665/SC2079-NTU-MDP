from communications.stm import STM

stm = STM()
stm.connect()

stm.send("F50\n")
while True:
    print(stm.wait_receive())
