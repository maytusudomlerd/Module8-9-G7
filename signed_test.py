# status = [0]
# perv_status = [0]
# Task_signed = [0]
# dataa = 0
# while(1):
#     data = input('Insert Number : ')
#     dataa = dataa + int(data)
#     if(int(dataa) > 0 ):
#         status[0] = 1
#         if(perv_status[0] == status[0]):
#             status[0] = 1
#             perv_status[0] = status[0]
#     else:
#         data = -1*data
#         status[0] = 0
#     print(dataa)
#     print(status[0])
#     Task_signed[0] = Task_signed[0] + 2
#     print(Task_signed)
Task_signed = 0
Position_Robot_Task = [10,0,10,0]
for i in range(0,4):
    if(Position_Robot_Task[i] > 0):
        Task_signed = (1 << (3-i)) | Task_signed 
        print('i = '+str(i) + '  ' + str(Task_signed))
    else:
        Task_signed = (1-(1 << (3-i))) & Task_signed  
        print('i = '+str(i) + '  ' + str(Task_signed))