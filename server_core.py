"""
protocol { header lenght instruction parameter[1] parameter[2] ..... paremeter[n] }
length = n(parameter) + 2
"""

import socket
import threading

port = 6969
server = socket.gethostbyname(socket.gethostname())
addr = (server,port)

format_msg = 'utf-8'

msg = []
disconnected_command = [0xFF,0x2,0x99]
#initialize socket
server = socket.socket(socket.AF_INET,socket.SOCK_STREAM)   # ipv4 , type : stream
server.bind(addr)

def handel_server(conn,addr):
    print(f"NEW CONNECTION {addr} connected. ")

    connected = True
    while connected:
        msg = conn.recv(50).decode(format_msg)
        if(msg):
            list_msg = strtolist(msg)
            if(list_msg == disconnected_command):
                connected = False
            else:
                conn.send(listtostr(list_msg))
                print(f"[RECIVED FROM {addr}] {list_msg}")
                pass
                #implemet here
                
    conn.close()


def start_server():
    server.listen()
    print(f"[LISTENNING] Server is listening on {socket.gethostbyname(socket.gethostname())}")
    while True:
        conn , addr  = server.accept()
        thread = threading.Thread(target=handel_server,args=(conn,addr))
        thread.start()
        print(f'[ACTIVE CONNECTION] {threading.activeCount()-1}')

def listtostr(list_msg):
    message = ';'.join([str(i) for i in list_msg])
    message = message.encode(format_msg)
    return message

def strtolist(str):
    strlist = str.split(";")
    intlist = [int(i) for i in strlist]
    return intlist

if __name__ == "__main__":

    print("\n[STARTING] server is starting ......")
    start_server()