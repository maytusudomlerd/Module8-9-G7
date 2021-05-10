clear all
destination = '127.0.0.1';
port = 6969;

echotcpip("on",port)
sim_client = tcpclient(destination,port);

configureTerminator(sim_client,"CR");
sim_client.Terminator;

connected = 1;
connected_1st = 1;
%%%%%%   command list   %%%%%%%
still_alive = '255;255';
server_disconnect = '255;99';


while connected == 1
    if(connected_1st == 1)
        writeline(sim_client,still_alive);
        connected_1st = 0;
    else
        recieve_msg_length = read(sim_client,1,"string");
        recieve_msg_length = str2intarray(recieve_msg_length);
        recieve_msg = read(sim_client,recieve_msg_length,"string");
        parameter = str2intarray(recieve_msg)
        if(strcmp(server_disconnect,recieve_msg) == 1)
            connected = 0;
        end
        
        %implement other condition here
        
    end

end

echotcpip('off');
clear sim_client;

