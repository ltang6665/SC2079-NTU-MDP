import os

from dotenv import load_dotenv
load_dotenv()

from bluetooth import (
    BluetoothSocket,
    RFCOMM,
    SERIAL_PORT_CLASS,
    SERIAL_PORT_PROFILE,
    advertise_service,
)

UUID = "94f39d29-7d6d-437d-973b-fba39e49d4ee"
SERVICE_NAME = "MDP-Server"

def main():
    BT_PORT = int(os.getenv("BT_PORT", 1))
    server_sock = BluetoothSocket(RFCOMM)

    server_sock.bind(("", BT_PORT))
    server_sock.listen(1)

    port = server_sock.getsockname()[1]


    advertise_service(
        server_sock,
        SERVICE_NAME,
        service_id=UUID,
        service_classes=[UUID, SERIAL_PORT_CLASS],
        profiles=[SERIAL_PORT_PROFILE],

    )

    print(f"Waiting for connection on RFCOMM channel {port}")

    try:
        client_sock, client_info = server_sock.accept()
        print("Accepted connection from", client_info)

        while True:
            data = client_sock.recv(1024)
            if not data:
                break
            print(f"Received [{data!r}]")
            user_input = input("Press Enter to send response...")
            client_sock.send(bytes(user_input, "utf-8"))
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    except OSError as e:
        print("Socket error:", e)
    finally:
        print("disconnected")
        try:
            client_sock.close()
        except Exception:
            pass
        server_sock.close()
        print("all done")

if __name__ == "__main__":
    main()
