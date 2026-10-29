import logging
logging.basicConfig(level=logging.INFO)

from communications.pc_link import PC

if __name__ == "__main__":
    server = PC()
    server.connect()
    try:
        while True:
            msg = server.receive()
            if msg is None:
                break
            logging.info(f"Pi received: {msg}")

            server.send(f"ack: {msg}")
    finally:
        server.disconnect()
