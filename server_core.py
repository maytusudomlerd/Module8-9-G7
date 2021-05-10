"""
protocol { header lenght instruction parameter[1] parameter[2] ..... paremeter[n] }
length = n(parameter) + 2
"""

import socket
import threading

port = 6969
server = socket.gethostbyname(socket.gethostname())
addr = ('localhost',port)

format_msg = 'utf-8'

msg = []
disconnected_command = [255,99]

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
                print("\n[SHUTDOWN] server is shutting down ......")
                connected = False
            else:
                print(f"[RECIVED FROM {addr}] {list_msg}")

                pack_len = str(len(listtostr(disconnected_command))).encode(format_msg)
                conn.send(pack_len)
                conn.send(listtostr(disconnected_command))
                
                #implemet here
                
    conn.close()


def start_server():
    server.listen()
    print(f"[LISTENNING] Server is listening on {socket.gethostbyname('localhost')}")
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