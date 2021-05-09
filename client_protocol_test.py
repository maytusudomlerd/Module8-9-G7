import socket


disconnected_command = [0xFF,0x2,0x99]
test_msg = [0xff,0x05,0x1,0x99,0xaa,0xbb]

format_msg = 'utf-8'

destination = socket.gethostbyname(socket.gethostname())
port = 6969
addr =(destination,port)

client = socket.socket(socket.AF_INET,socket.SOCK_STREAM)   # ipv4 , type : stream
client.connect(addr)


def send(msg):
    message = msg.encode(format_msg)
    client.send(message)

def listtostr(list_msg):
    message = ';'.join([str(i) for i in list_msg])
    return message

def strtolist(str):
    strlist = str.split(";")
    intlist = [int(i) for i in strlist]
    return intlist

if __name__ == "__main__":
    count = 0
    while count != 3:
        send(listtostr(test_msg))
        recieve_list = strtolist(client.recv(50).decode(format_msg))
        print(recieve_list)
        test_msg[1] = test_msg[1] + 10
        count += 1
    
    

