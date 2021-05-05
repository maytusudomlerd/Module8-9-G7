from serial import Serial
import serial
import numpy as np


# INIT UART PORT    
Supatipunno = Serial(port= 'COM7',timeout = 3, baudrate=256000,
                  xonxoff=0, rtscts=0,bytesize=8, parity='N', stopbits=1)
# CHECK UART PORT IS OPEN
try:
    Supatipunno.is_open
    print('\nCOMMUNICATION WITH PORT NAME : ' + Supatipunno.portstr)
    print('UART PORT IS OPEN. READY TO COMMUNICATION \n')
    
except:
    print('UART PORT IS CLOSE. PLEASE TRY AGAIN LATER \n')

# GLOBAL VARIBLES
Ping_Flag = 0
Position_Robot_Task = [0, 0, 0, 0]
Position_Robot_Joint = [0, 0, 0, 0]
abs_Position_Robot_Joint = [0, 0, 0, 0]
abs_Position_Robot_Task = [0, 0, 0, 0]
Traj_coef = [0,0,0,0,0,0]  #c0,c1,c2,c3,c4,c5 

# INPUT DATA FROM USER FUNCTION
def Get_Data_From_User():
    global Ping_Flag
    global Position_Robot_Task
    Instruction = None
    Parameter = []

    print('\nList of Command : \n\t1. Reset Board \n\t2. Set Home \n\t3. Ping Mode \n\t4. Jog Joint \n\t5. Jog Cartesian  \n\t6. Move  \n\t7. Trajectory Move ')
    User_Command = input('Input Your Command (1-7) : ')
    # Reset
    if(User_Command == '1'):
        Instruction = 1
        print('function is not ready now')
        # print(Get_Tx_Package([Instruction]))
    # Set Home
    elif(User_Command == '2'):
        Instruction = 2
        print(Get_Tx_Package(Instruction,None))
        Supatipunno.write(serial.to_bytes(Get_Tx_Package(Instruction,None)))
        
    # Ping Mode
    elif(User_Command == '3'):
        Instruction = 3
        if(Ping_Flag == 0):
            Ping_Flag = 1
            print('Ping Mode Enable\n')
            Parameter.append(1)
            print(Get_Tx_Package(Instruction, Parameter))
            Supatipunno.write(serial.to_bytes(Get_Tx_Package(Instruction, Parameter)))
        else:
            Ping_Flag = 0
            Parameter.append(0)
            print('Ping Mode Disable\n')
            print(Get_Tx_Package(Instruction, Parameter))
            Supatipunno.write(serial.to_bytes(Get_Tx_Package(Instruction, Parameter)))
    # Jog Joint
    elif(User_Command == '4'):
        JogJoint()
    # Jog Cartesian
    elif(User_Command == '5'):
        JogCartesian()
    # Move Joint
    elif(User_Command == '6'):
       Move() 
    # Trajectory Move
    elif(User_Command == '7'):
        Instruction = 7
        global Traj_coef
        print(Get_Tx_Package(Instruction,Traj_coef))
    elif(User_Command == '99'):
        return True
    else:
        print('Invalid Command Please Try Agian')
    return False

# Jog joint
def JogJoint():
    global Position_Robot_Joint
    global abs_Position_Robot_Joint
    Jog_state = 0
    Joint_Select = 0
    Joint_Resolution = 10
    Instruction = 4
    Joint_signed = 0
    Joint_Number = 0
    Joint_Signed_Num = 0
    while(Jog_state != 99):
        print('\nYour Joint Resolution Mode is  ' + str(Joint_Resolution))
        Jog_state = input(
            'Joint Jog menu \n\t1. Start Jog \n\t2. Select Resolution of step(defult 10) \n\t99. Finish Jog\n\tSelect your menu here : ')
        if(Jog_state == '1'):
            Joint_Select = 0
            while(Joint_Select != '99'):
                print('\nSelect Joint whitch you want to jog\n\t1. + Joint 1  2. - Joint 1 \n\t3. + Joint 2  4. - Joint 2 \n\t5. + Joint 3  6. - Joint 3 \n\t7. + Joint 4  8. - Joint 4')
                print('\t99. Back to Jog Joint menu ')
                print('Your Joint Position is J1 : ' + str(Position_Robot_Joint[0]) + ' J2 :' + str(
                    Position_Robot_Joint[1]) + ' J3 : ' + str(Position_Robot_Joint[2]) + ' J4 : ' + str(Position_Robot_Joint[3]))
                Joint_Select = input('Select your Joint Jog here : ')

                if(Joint_Select == '1'):
                    Position_Robot_Joint[0] = Position_Robot_Joint[0] + Joint_Resolution
                    Joint_Number = 0x08
                elif(Joint_Select == '2'):
                    Position_Robot_Joint[0] = Position_Robot_Joint[0] - Joint_Resolution
                    Joint_Number = 0x08
                elif(Joint_Select == '3'):
                    Position_Robot_Joint[1] = Position_Robot_Joint[1] + Joint_Resolution
                    Joint_Number = 0x04
                elif(Joint_Select == '4'):
                    Position_Robot_Joint[1] = Position_Robot_Joint[1] - Joint_Resolution
                    Joint_Number = 0x04
                elif(Joint_Select == '5'):
                    Position_Robot_Joint[2] = Position_Robot_Joint[2] + Joint_Resolution
                    Joint_Number = 0x02
                elif(Joint_Select == '6'):
                    Position_Robot_Joint[2] = Position_Robot_Joint[2] - Joint_Resolution
                    Joint_Number = 0x02
                elif(Joint_Select == '7'):
                    Position_Robot_Joint[3] = Position_Robot_Joint[3] + Joint_Resolution
                    Joint_Number = 0x01
                elif(Joint_Select == '8'):
                    Position_Robot_Joint[3] = Position_Robot_Joint[3] - Joint_Resolution
                    Joint_Number = 0x01
                elif(Joint_Select == '99'):
                    Jog_state = '99'
                else:
                    print('Invaid command please Input in range 1-8 \n')
                for i in range(0,4):
                    if(Position_Robot_Joint[i] >= 0):
                        Joint_signed = (1 << (3-i)) | Joint_signed
                        #print('i = '+str(i) + '  ' + str(Task_signed))
                    else:
                        Joint_signed = (15-(1 << (3-i))) & Joint_signed
                        #print('i = '+str(i) + '  ' + str(Task_signed))
                #print('Task sign = '+ str(Task_signed) )
                abs_Position_Robot_Joint = [abs(int(Position_Robot_Joint[0])), abs(int(Position_Robot_Joint[1])), abs(int(Position_Robot_Joint[2])), abs(int(Position_Robot_Joint[3]))]
                
                if(Joint_Number == 0x08):
                    Joint_Signed_Num = ((Joint_signed & 0x08)<<1) | Joint_Number
                    print(Get_Tx_Package(Instruction, [Joint_Signed_Num,abs_Position_Robot_Joint[0]]))
                    Supatipunno.write(serial.to_bytes(Get_Tx_Package(Instruction, [Joint_Signed_Num,abs_Position_Robot_Joint[0]])))
                elif(Joint_Number == 0x04):
                    Joint_Signed_Num = ((Joint_signed & 0x04)<<2) | Joint_Number
                    print(Get_Tx_Package(Instruction, [Joint_Signed_Num,abs_Position_Robot_Joint[1]]))
                    Supatipunno.write(serial.to_bytes(Get_Tx_Package(Instruction, [Joint_Signed_Num,abs_Position_Robot_Joint[1]])))
                elif(Joint_Number == 0x02):
                    Joint_Signed_Num = ((Joint_signed & 0x02)<<3) | Joint_Number
                    print(Get_Tx_Package(Instruction, [Joint_Signed_Num,abs_Position_Robot_Joint[2]]))
                    Supatipunno.write(serial.to_bytes(Get_Tx_Package(Instruction, [Joint_Signed_Num,abs_Position_Robot_Joint[2]])))
                elif(Joint_Number == 0x01):
                    Joint_Signed_Num = ((Joint_signed & 0x02)<<4) | Joint_Number
                    print(Get_Tx_Package(Instruction, [Joint_Signed_Num,abs_Position_Robot_Joint[3]]))
                    Supatipunno.write(serial.to_bytes(Get_Tx_Package(Instruction, [Joint_Signed_Num,abs_Position_Robot_Joint[3]])))

                
                #Grinex.write(Serial.to_byte(Get_Tx_Package([Instruction], [Joint_signed,abs_Position_Robot_Joint])))
            Jog_state = 0
        elif(Jog_state == '2'):
            print('Select Resolution of step Menu\nNOTE : Joint 1 2 4 is Revolute Joint(degree) and Joint 3 is Prismatic Joint(mm)')
            resolution = input(
                '\t1. 1 degree or mm \n\t2. 10 degree or mm\n\t3. 50 degree or mm\n\t99.Back to main Jog Joint Menu\n\tPlease Select Resolution of step : ')
            if(resolution == '1'):
                Joint_Resolution = 1
                Jog_state = 0
            elif(resolution == '2'):
                Joint_Resolution = 10
                Jog_state = 0
            elif(resolution == '3'):
                Joint_Resolution = 50
                Jog_state = 0
            elif(resolution == '99'):
                Jog_state = 0
            else:
                print('Invaid command please Input in range 1-3 \n')

        elif(Jog_state == '99'):
            break
        else:
            print("Menu select error please Input in range 1-4")
            Jog_state = 0
# Jog Cartesian
def JogCartesian():
    global Position_Robot_Task
    global abs_Position_Robot_Task
    Jog_state = 0
    Task_Select = 0
    Task_Resolution = 10
    Instruction = 5
    Task_signed = 0
    while(Jog_state != 99):
        print('\nYour Task Resolution Mode is  ' + str(Task_Resolution))
        Jog_state = input(
            'cartesian Jog menu \n\t1. Start Jog \n\t2. Select Resolution of step(defult 10) \n\t99. Finish Jog\n\tSelect your menu here : ')
        if(Jog_state == str(1)):
            Task_Select = 0
            while(Task_Select != str(99)):
                print('\nSelect Taskspace Variables whitch you want to jog\n\t1. + X  2. - X \n\t3. + Y  4. - Y \n\t5. + Z  6. - Z \n\t7. + Rotz  8. - Rotz')
                print('\t99. Back to Cartesian Jog menu ')
                print('Your Joint Position is X : ' + str(Position_Robot_Task[0]) + ' Y :' + str(
                    Position_Robot_Task[1]) + ' Z : ' + str(Position_Robot_Task[2]) + ' Rotz : ' + str(Position_Robot_Task[3]))
                print('Your ABS Joint Position is X : ' + str(abs_Position_Robot_Task[0]) + ' Y :' + str(
                    abs_Position_Robot_Task[1]) + ' Z : ' + str(abs_Position_Robot_Task[2]) + ' Rotz : ' + str(abs_Position_Robot_Task[3]))
                Task_Select = input('Select your Cartesian Jog here : ')

                if(Task_Select == '1'):
                    Position_Robot_Task[0] = Position_Robot_Task[0] + Task_Resolution
                elif(Task_Select == '2'):
                    Position_Robot_Task[0] = Position_Robot_Task[0] - Task_Resolution
                elif(Task_Select == '3'):
                    Position_Robot_Task[1] = Position_Robot_Task[1] + Task_Resolution
                elif(Task_Select == '4'):
                    Position_Robot_Task[1] = Position_Robot_Task[1] - Task_Resolution
                elif(Task_Select == '5'):
                    Position_Robot_Task[2] = Position_Robot_Task[2] + Task_Resolution
                elif(Task_Select == '6'):
                    Position_Robot_Task[2] = Position_Robot_Task[2] - Task_Resolution
                elif(Task_Select == '7'):
                    Position_Robot_Task[3] = Position_Robot_Task[3] + Task_Resolution
                elif(Task_Select == '8'):
                    Position_Robot_Task[3] = Position_Robot_Task[3] - Task_Resolution
                elif(Task_Select == '99'):
                    Jog_state = '99'
                else:
                    print('Invaid command please Input in range 1-8 \n')

                for i in range(0, 4):
                    if(Position_Robot_Task[i] >= 0):
                        Task_signed = (1 << (3-i)) | Task_signed
                        #print('i = '+str(i) + '  ' + str(Task_signed))
                    else:
                        Task_signed = (15-(1 << (3-i))) & Task_signed
                        #print('i = '+str(i) + '  ' + str(Task_signed))
                #print('Task sign = '+ str(Task_signed) )
                abs_Position_Robot_Task = [abs(int(Position_Robot_Task[0])), abs(int(Position_Robot_Task[1])), abs(int(Position_Robot_Task[2])), abs(int(Position_Robot_Task[3]))]
                print(Get_Tx_Package(Instruction, [Task_signed]+abs_Position_Robot_Task))
                Supatipunno.write(serial.to_bytes(Get_Tx_Package(Instruction, [Task_signed]+abs_Position_Robot_Task)))
                # Task -IPK-> config | split sign | send !
            Jog_state = 0
        elif(Jog_state == '2'):
            print('Select Resolution of step Menu\nNOTE : X Y Z is position in Taskspace with Respect to Frame 0 (mm) and Rotz is Thetha is rotage Joint 4 (degree)')
            resolution= input('\t1. 1 degree or mm \n\t2. 10 degree or mm\n\t3. 50 degree or mm\n\t99.Back to main Jog Joint Menu\n\tPlease Select Resolution of step : ')
            if(resolution == '1'):
                Task_Resolution = 1
                Jog_state = 0
            elif(resolution == '2'):
                Task_Resolution = 10
                Jog_state = 0
            elif(resolution == '3'):
                Task_Resolution = 50
                Jog_state = 0
            elif(resolution == '99'):
                Jog_state = 0
            else:
                print('Invaid command please Input in range 1-3 \n')    

        elif(Jog_state == '99'):
            break
        else:
            print("Menu select error please Input in range 1-4")
            Jog_state = 0
# Move
def Move():
    global Position_Robot_Task
    global Position_Robot_Joint
    global abs_Position_Robot_Joint
    global abs_Position_Robot_Task
    Instruction = 6
    Type_flag = 0
    Type_list = ['Configuration Variables','Taskspace Variables']
    Type_of_Input = 0
    Move_state = '0'
    Moving = 0
    Joint_signed = 0
    Task_signed = 0

    while(Move_state != str(99)):
        if(Move_state == str(0)):
            print('Move Menu\nPlease select Input Before Start Move (Defult : Joint space) ')
            print('Your Select Input : ' + Type_list[Type_flag])
            Move_state = input('Select type Mewnu \n\t1. Select Input Type\n\t2. Start Move\n\t99. Back to Main menu\nInput Your Command : ')
        if(Move_state == str(1)):
            Type_of_Input = input('Input your Type of Input \n\t1. Jointspace Variables \n\t2. Taskspace Variables \n\t99. Back\nInput your Type of Input : ')
            if(Type_of_Input == str(1)):
                Type_flag = 0
                Move_state = str(2)
            elif(Type_of_Input == str(2)):
                Type_flag = 1
                Move_state = str(2)
            elif(Type_of_Input == str(99)):
                Move_state = str(0)
            else:
                print('Invaid command please Input in range 1 or 2 \n')
        elif(Move_state == str(2)):
            while(Moving != str(99)):
                if(Type_flag == 0):
                    print('\nInput your Configaration Variable \n Note : Revolaut(Joint 1,2,4 : Radian),prismatic(Joint 3 : mm)')
                    try:
                        for i in range(0, 4):
                            Joint_Config_Input = input('Joint (q) '+str(i+1) + ': ')
                            Position_Robot_Joint[i] = int(Joint_Config_Input)
                        for i in range(0,4):
                            if(Position_Robot_Joint[i] >= 0):
                                Joint_signed = (1 << (3-i)) | Joint_signed
                                #print('i = '+str(i) + '  ' + str(Task_signed))
                            else:
                                Joint_signed = (15-(1 << (3-i))) & Joint_signed
                                #print('i = '+str(i) + '  ' + str(Task_signed))
                                #print('Task sign = '+ str(Task_signed) )
                        abs_Position_Robot_Joint = [abs(int(Position_Robot_Joint[0])), abs(int(Position_Robot_Joint[1])), abs(int(Position_Robot_Joint[2])), abs(int(Position_Robot_Joint[3]))]
                        print(Get_Tx_Package(Instruction, [Joint_signed]+abs_Position_Robot_Joint))
                        Supatipunno.write(serial.to_bytes(Get_Tx_Package(Instruction, [Joint_signed]+abs_Position_Robot_Joint)))
                    except:
                        print('Invalid Parameter\n')
                        pass
                elif(Type_flag == 1):
                    print('\nInput your Taskspace Variable \n Note : End - Effector Position (X,Y,Z,Rotz)')
                    Task_variable = ['X', 'Y', 'Z', 'RotZ']
                    try:
                        index = 0
                        for i in Task_variable:
                            Task__Input = input('Position in ' + i + ' : ')
                            Position_Robot_Task[index] = int(Task__Input)
                            index= index+1
                        for i in range(0,4):
                            if(Position_Robot_Task[i] >= 0):
                                Task_signed = (1 << (3-i)) | Task_signed
                                #print('i = '+str(i) + '  ' + str(Task_signed))
                            else:
                                Task_signed = (15-(1 << (3-i))) & Task_signed
                                #print('i = '+str(i) + '  ' + str(Task_signed))
                                #print('Task sign = '+ str(Task_signed) )
                        abs_Position_Robot_Task = [abs(int(Position_Robot_Task[0])), abs(int(Position_Robot_Task[1])), abs(int(Position_Robot_Task[2])), abs(int(Position_Robot_Task[3]))]
                        print(Get_Tx_Package(Instruction, [Joint_signed]+abs_Position_Robot_Joint))
                        Supatipunno.write(serial.to_bytes(Get_Tx_Package(Instruction, [Joint_signed]+abs_Position_Robot_Joint)))
                        #.write(Serial.to_byte(Get_Tx_Package([Instruction], [Task_signed,Parameter])))
                    except:
                        print('Invalid Parameter\n')
                        pass
                Moving = input('Do you want to continue move (Y/N) : ')
                if(Moving.lower() == 'y'):
                    pass
                else:
                    Move_state = str(0)
                    Moving = 99
                    break
        elif(Move_state == str(99)):
           break
        else:
            print('Invaid command please Input in range 1 or 2 \n')
            Move_state = str(0)
            
# GET PACKAGE FROM INPUT DATA FUNCTION
##########################################################################################################################################
##                                                                                                                                      ##
##         Data Package is [ Header | Lenght | Instruction | Parameter1 | Paremeter2 | ........... |  Parameter n | CRC_H | CRC_H ]     ##
##                                                                                                                                      ##
##########################################################################################################################################

def Get_Tx_Package(Instruction, Parameter = None):

    Header = 0xFF
    Tx_Package = []
    if Parameter != None:
        for i in range(0, len(Parameter)):
            Tx_Package.append(np.uint16(Parameter[i]))     
    # Convert To Byte
    Flag = 0 if Instruction in [2,3,7] else 1
    Byte_Tx_Package = Convert_Data_To_Byte(Tx_Package,Flag = Flag)
    
    Tx_Package.insert(0,Header)
    Tx_Package.insert(1,16)
    Tx_Package.insert(2,Instruction)
    
    while len(Tx_Package) != 10:
        Tx_Package.append(np.uint16(0)
    # CRC Calcualte
    crc = Update_CRC(Target_Data = np.uint16(Tx_Package))
    Byte_Tx_Package[-2] = crc // 256
    Byte_Tx_Package[-1] = crc % 256
    Byte_Tx_Package.insert(0,Header)
    Byte_Tx_Package.insert(1,16)
    Byte_Tx_Package.insert(2,Instruction)
    return Byte_Tx_Package

# Convert to Byte
def Convert_Data_To_Byte(Package,Flag = 0):
    Byte_Package = []
    if(Flag == 1):
        Byte_Package.append(Package[0])
    for i in range(Flag,len(Package)):
        if(Package[i]%256 == Package[i] or Package[i] == 0):
            Byte_Package.append(0)
            Byte_Package.append(Package[i])
        else:
            Byte_Package.append(Package[i]//256)
            Byte_Package.append(Package[i]%256)

    while(len(Byte_Package) != 14):
        Byte_Package.append(0)

    return Byte_Package

def Update_CRC(Result = 0,Target_Data = 0):
    Target_Data_Size = len(Target_Data)
    crc_table = [
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    ]
    for j in range(0,Target_Data_Size):
        i = ((np.uint16(Result >> 8)) ^ Target_Data[j]) & 0xFF
        Result = np.uint16(Result << 8 ) ^ crc_table[i]
    return Result

# WAIT ACKNOWLEDGE 
# def Recieve_Rx_Package():
#     Recieve_Package = []
#     Rx_package = []
#     while(Grinex.in_waiting < 4):
#         pass
#     Rx_package = Grinex.readline(4)
#     Rx_package.append(Grinex.readline(Rx_package[1]-2))
#     try:
#         for i in range(0,len(Rx_package)):
#             for j in range(0,len(Rx_package[i])):
#                 Recieve_Package.append(int(Rx_package[i][j]))
#     except:
#         for i in range(0,len(Rx_package)):
#             Recieve_Package.append(int(Rx_package[i]))
#     Rx_crc = (Recieve_Package[len(Recieve_Package)-2] << 8) + Recieve_Package[len(Recieve_Package)-1]
#     Result_crc = Update_CRC(0,Recieve_Package)
#     if(Rx_crc == Result_crc):
#         return Recieve_Package
#     else:
#         Recieve_Package = []
    
if __name__ == "__main__":
    while(1):
        try:
            is_done = Get_Data_From_User()
            if is_done:
                break    
        except:
            print('Get Package fail \n\n')
            break

