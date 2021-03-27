Master = 'localhost';
node = ros.Node('C',Master);
listener = ros.Subscriber(node,'/name',ackcallback);



function ackcallback
    speaker = ros.Publisher(node_2,'last');
    ack_msg = rosmessage(speaker);
    ack_msg.ack = 'ack';
    send(speaker,ack_msg)
end

