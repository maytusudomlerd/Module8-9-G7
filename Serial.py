import serial

# Grinex = serial.Serial(port = 'COM3',timeout = None, baudrate=2000000,
#                         xonxoff=0, rtscts=0,bytesize=8, parity='N', stopbits=1)

def Get_Data_From_User():
    
    Traj_flag = 0
    Instruction = []
    Config_Variable = [0,0,0,0]
    Taskspace_Variable = [0,0,0,0]

    print('List of Command : \n     1. Enable Ping Mode \n     2. Set Home \n     3. Control with Jointspace \n    4. Control with Taskspace')
    User_Command = input('Input Your Command (1-4) : ')
    if(User_Command == str(1)):
        Instruction.append(1)
    elif(User_Command == str(2)):
        Instruction.append(2)
    elif(User_Command == str(3)):
        Instruction.append(3)
        print('Input your Configaration Variable \n Note : Revolaut(Joint 1,2,4 : Radian),prismatic(Joint 3 : mm)')

        Config_Variable[0] = input('Joint 1 : ')
        Config_Variable[1] = input('Joint 2 : ')
        Config_Variable[2] = input('Joint 3 : ')
        Config_Variable[3] = input('Joint 4 : ')

        Traj = input('Do you want to use Trajectory ? (Y/N) : ')
        if(Traj.lower() == 'y'):
            Traj_flag = 1
    elif(User_Command== str(4)):
        Instruction.append(4)
        print('Input your Configaration Variable \n Note : End - Effector Position (X,Y,Z,Rotz of 4')

        Taskspace_Variable[0] = input('X : ')
        Taskspace_Variable[1] = input('Y : ')
        Taskspace_Variable[2] = input('Z : ')
        Taskspace_Variable[3] = input('Rotz : ')

        Traj = input('Do you want to use Trajectory ? (Y/N) : ')
        if(Traj.lower() == 'y'):
            Traj_flag = 1;
    else:
        print('Invalid Command Please Try Agian')
            
                
    return Instruction,Traj_flag,Config_Variable,Taskspace_Variable

    
##########################################################################################################################################
##                                                                                                                                      ##
##         Data Package is [ Header | Lenght | Instruction | Parameter1 | Paremeter2 | ........... |  Parameter n | CRC_H | CRC_H ]     ##
##                                                                                                                                      ##                                            
##########################################################################################################################################


def Get_Tx_Package(Instruction = [],Parameter = []):

    Header = [0xFF]
    Instruction = Instruction[0]
    Parameter = Parameter
    Lenght = len(Parameter) + 3
    CRC_H = 99
    CRC_L = 99

    Tx_Package = [Header[0], Lenght, Instruction[0]]


    for i in range(0, len(Parameter)):
        try:
            for j in range(0,len(Parameter[i])):
                Tx_Package.append(int(Parameter[i][j]))
        except:
            Tx_Package.append(Parameter[i])

    Tx_Package.append(CRC_H)
    Tx_Package.append(CRC_L)
    return Tx_Package

# CRC Check


if __name__ == "__main__":
    Instruction,Traj_flag,Config_Variable,Taskspace_Variable = Get_Data_From_User()
    try:
        print(Get_Tx_Package([Instruction],[Traj_flag,Config_Variable,Taskspace_Variable]))
    except:
        pass





