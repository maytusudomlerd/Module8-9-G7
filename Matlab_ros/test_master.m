try
    if isvalid(core)
        rosshutdown;
        delete(core)
    end
    rosshutdown;
    clear all;
    core = ros.Core;
    rosinit(erase(erase(core.MasterURI,'http://'),[':' num2str(core.Port)]));
catch
    % initializing new ROS Core
    core = ros.Core;
    rosinit(erase(erase(core.MasterURI,'http://'),[':' num2str(core.Port)]));
end
Master = core.MasterURI;


node_1 = ros.Node('A',Master);
node_2 = ros.Node('B',Master);

rostopic list

sub_1 = ros.Subscriber(node_2,'/last');

pub_1 = rospublisher('/first');
msg_1 = rosmessage(pub_1);

msg_1.info.first_name = 'maytus';
msg_1.info.last_name = 'udomlerd';

send(pub_1,msg_1);
