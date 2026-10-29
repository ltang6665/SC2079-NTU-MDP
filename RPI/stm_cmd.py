from communications.stm import STM

stm = STM()
stm.connect()
while True:
    cmd = input("CMD: ")
    stm.send(cmd + "\n")
