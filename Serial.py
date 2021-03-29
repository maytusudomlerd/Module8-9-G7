import serial
import crc16

# INIT UART PORT
# Grinex = serial.Serial(port='COM5', timeout=None, baudrate=2000000,
#                         xonxoff=0, rtscts=0, bytesize=8, parity='N', stopbits=1)

# CHECK UART PORT IS OPEN
try:
    Grinex.is_open
    print('\nCOMMUNICATION WITH PORT NAME : ' + Grinex.portstr)
    print('UART PORT IS OPEN. READY TO COMMUNICATION \n\n')
except:
    print('UART PORT IS CLOSE. PLEASE TRY AGAIN LATER \n\n')

# GLOBAL VARIBLES
Ping_Flag = 0

# INPUT DATA FROM USER FUNCTION


def Get_Data_From_User():
    global Ping_Flag
    Traj_flag = 0
    Instruction = []
    Parameter = []

    print('List of Command : \n\t1. Reset Board \n\t2. Set Home \n\t3. Ping Mode \n\t4. Jog Joint \n\t5. Jog Cartesian  \n\t6. Move with Jointspace  \n\t7. Move with Taskspcae ')
    User_Command = input('Input Your Command (1-7) : ')
    # Reset
    if(User_Command == str(1)):
        Instruction.append(1)
        print(Get_Tx_Package([Instruction]))
    # Set Home
    elif(User_Command == str(2)):
        Instruction.append(2)
        print(Get_Tx_Package([Instruction]))
    # Ping Mode
    elif(User_Command == str(3)):
        Instruction.append(3)
        if(Ping_Flag == 0):
            Ping_Flag = 1
            print('Ping Mode Enable\n')
            Parameter.append(int(1))
            print(Get_Tx_Package([Instruction],[Parameter]))
        else:
            Ping_Flag = 0
            Parameter.append(int(0))
            print('Ping Mode Disable\n')
            print(Get_Tx_Package([Instruction],[Parameter]))
    # Jog Joint
    elif(User_Command == str(4)):
        JogJoint()
    # Jog Cartesian
    #elif(User_Command == str(4)):
        #JogCartesian()
    # Move Joint
    elif(User_Command == str(6)):
        Instruction.append(6)

        Traj = input('Do you want to use Trajectory ? (Y/N) : ')
        if(Traj.lower() == 'y'):
            Traj_flag = 1
            Parameter.append(int(Traj_flag))
        else:
            Parameter.append(int(Traj_flag))

        print('Input your Configaration Variable \n Note : Revolaut(Joint 1,2,4 : Radian),prismatic(Joint 3 : mm)')
        for i in range(0, 4):
            Joint_Config_Input = input('Joint'+str(i) + ': ')
            Parameter.append(int(Joint_Config_Input))
        print(Get_Tx_Package([Instruction],[Parameter]))
    # Move Task
    elif(User_Command == str(7)):
        Instruction.append(7)

        Traj = input('Do you want to use Trajectory ? (Y/N) : ')
        if(Traj.lower() == 'y'):
            Traj_flag = 1
            Parameter.append(int(Traj_flag))
        else:
            Parameter.append(int(Traj_flag))

        print('Input your Configaration Variable \n Note : End - Effector Position (X,Y,Z,Rotz of 4')
        Task_variable = ['X','Y','Z','RotZ']
        for i in range(0, 4):
            Task__Input = input('Position in' + Task_variable[i] + ': ')
            Parameter.append(int(Task__Input))
        print(Get_Tx_Package([Instruction],[Parameter]))
    else:
        print('Invalid Command Please Try Agian')


def JogJoint():
    Jog_state = 0
    Joint_Select = 0
    Joint_Resolution = 10
    Instruction = [3]
    while(Jog_state != 99):
        print('\nYour Joint Resolution Mode is  ' + str(Joint_Resolution) + '\n')
        Jog_state = input('Joint Jog menu \n\t1. Start Jog \n\t2. Select Resolution of step(defult 10) \n\t99. Finish Jog\n\tSelect your menu here : ')
        if(Jog_state == str(1)):
            while(Joint_Select != str(99)):
                print('Select Joint whitch you want to jog\n\t1. + Joint 1  2. - Joint 1 \n\t3. + Joint 2  4. - Joint 2 \n\t5. + Joint 3  6. - Joint 3 \n\t7. + Joint 4  8. - Joint 4 \n')
                print('\n\t99. Back to Jog Joint menu ')
                Joint_Select = input('Select your Joint Jog here :' )
                if(Joint_Select in ['1','2','3','4','5','6','7','8','99']):
                    print(Get_Tx_Package([Instruction],[Joint_Select,Joint_Resolution]))
                else:
                    print('Invaid command please Input in range 1-8 \n')
            Jog_state = 0
        elif(Jog_state == str(2)):
            print('Select Resolution of step Menu\n NOTE : Joint 1 2 4 is Revolute Joint(degree) and Joint 3 is Prismatic Joint(mm)')
            Joint_Resolution = input('Please Select Resolution of step \n\t1. 1 degree or mm \n\t2. 10 degree or mm\n\t3. 50 degree or mm\n\t99.Back to main Jog Joint Menu ')
            Jog_state = 0
        elif(Jog_state == str(99)):
            break
        else:
            print("Menu select error please Input in range 1-4")
            Jog_state = 0

            





# GET PACKAGE FROM INPUT DATA FUNCTION
##########################################################################################################################################
##                                                                                                                                      ##
##         Data Package is [ Header | Lenght | Instruction | Parameter1 | Paremeter2 | ........... |  Parameter n | CRC_H | CRC_H ]     ##
##                                                                                                                                      ##
##########################################################################################################################################

def Get_Tx_Package(Instruction=[], Parameter=[]):

    Header = [0xFF]
    Instruction = Instruction[0]
    Parameter = Parameter
    Lenght = 0
    Tx_Package = [Header[0], Lenght, Instruction[0]]
    crc_sum = 0

    for i in range(0, len(Parameter)):
        try:
            for j in range(0, len(Parameter[i])):
                Tx_Package.append(int(Parameter[i][j]))

        except:
            Tx_Package.append(int(Parameter[i]))

    # Convert To Byte
    Byte_Tx_Package = Convert_Data_To_Byte(Tx_Package)
    Byte_Tx_Package[1] = len(Byte_Tx_Package)-1

    #CRC Calcualte
    for i in range(0,len(Byte_Tx_Package)):
        crc_sum = crc_sum + i
    crc = crc16.crc16xmodem(b'crc_sum')
    print(crc)
    crc_H = crc // 256
    crc_L = crc % 256
    Byte_Tx_Package.append(crc_H)
    Byte_Tx_Package.append(crc_L)
    Byte_Tx_Package[1] = Byte_Tx_Package[1] + 2

    return Byte_Tx_Package

# Convert to Byte
def Convert_Data_To_Byte(Package = []):
    Byte_Package = []
    for i in Package:
        if(i%256 == i):
            Byte_Package.append(i)
        else:
            Byte_Package.append(i//256)
            Byte_Package.append(i%256)
    return Byte_Package



if __name__ == "__main__":
    while(1):
        try:
            Get_Data_From_User()
        except:
            print('Get Package fail \n\n')
            break
