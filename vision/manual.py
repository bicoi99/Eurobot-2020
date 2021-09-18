import socket

s1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# s2.connect(("10.177.133.75", 1235))


while True:
    print("Choose an option...")
    val = input("j - Jack, r - Rose, x - exit\n>>> ")
    if val == "x":
        break
    elif val == "j":
        s1.connect(("10.177.133.75", 1234))
        print("Sent signal to Jack")
        s1.send(b'1')
        print("Return:", s1.recv(1024), "\n")
        s1.close()
    elif val == "r":
        print("Sent signal to Rose")
        s2.sendall(b'1')
        print("Return:", s2.recv(1024), "\n")
    else:
        print("Invalid input\n")

s1.close()
s2.close()
