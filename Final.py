#!/usr/bin/env python
# coding: utf-8

# In[1]:


def hex2bin(val):
    n = int(val, 16) 
    bStr = ''
    while n > 0:
        bStr = str(n % 2) + bStr
        n = n >> 1    
    return bStr


# In[2]:


def rubix(k, k5, num):
    
    k51 = k5[0:16]
    k52 = k5[16:32]
    k53 = k5[32:48]
    k54 = k5[48:64]
    
    new_list = k[:]
    
    if(num == 1):
        l = 0
        for i in k51:
            if(l%3 == 0):
                if(i == "0" or i == "4" or i == "8" or i == "c"):
                    pass
                elif(i == "1" or i == "7"):
                    new_list[0], new_list[9] = new_list[9], new_list[0]
                    new_list[1], new_list[10] = new_list[10], new_list[1]
                    new_list[2], new_list[11] = new_list[11], new_list[2]
                elif(i == "2" or i == "6"):
                    new_list[0], new_list[15] = new_list[15], new_list[0]
                    new_list[1], new_list[19] = new_list[19], new_list[1]
                    new_list[2], new_list[20] = new_list[20], new_list[2]
                elif(i == "3" or i == "5"):    
                    new_list[0], new_list[27] = new_list[27], new_list[0]
                    new_list[1], new_list[28] = new_list[28], new_list[1]
                    new_list[2], new_list[29] = new_list[29], new_list[2]
                elif(i == "9" or i == "f"):
                    new_list[0], new_list[45] = new_list[45], new_list[0]
                    new_list[3], new_list[48] = new_list[48], new_list[3]
                    new_list[6], new_list[51] = new_list[51], new_list[6]
                elif(i == "a" or i == "e"):
                    new_list[0], new_list[26] = new_list[26], new_list[0]
                    new_list[3], new_list[23] = new_list[23], new_list[3]
                    new_list[6], new_list[20] = new_list[20], new_list[6]
                elif(i == "b" or i == "d"):    
                    new_list[0], new_list[36] = new_list[36], new_list[0]
                    new_list[3], new_list[39] = new_list[39], new_list[3]
                    new_list[6], new_list[42] = new_list[42], new_list[6]
                                          
                                        
            elif(l%3 == 1):      
                if(i == "0" or i == "4" or i == "8" or i == "c"):
                    pass
                elif(i == "1" or i == "7"):
                    new_list[3], new_list[12] = new_list[12], new_list[3]
                    new_list[4], new_list[13] = new_list[13], new_list[4]
                    new_list[5], new_list[14] = new_list[14], new_list[5]
                elif(i == "2" or i == "6"):
                    new_list[3], new_list[21] = new_list[21], new_list[3]
                    new_list[4], new_list[22] = new_list[22], new_list[4]
                    new_list[5], new_list[23] = new_list[23], new_list[5]
                elif(i == "3" or i == "5"):    
                    new_list[3], new_list[30] = new_list[30], new_list[3]
                    new_list[4], new_list[31] = new_list[31], new_list[4]
                    new_list[5], new_list[32] = new_list[32], new_list[5]
                elif(i == "9" or i == "f"):
                    new_list[1], new_list[46] = new_list[46], new_list[1]
                    new_list[4], new_list[49] = new_list[49], new_list[4]
                    new_list[7], new_list[52] = new_list[52], new_list[7]
                elif(i == "a" or i == "e"):
                    new_list[1], new_list[25] = new_list[25], new_list[1]
                    new_list[4], new_list[22] = new_list[22], new_list[4]
                    new_list[7], new_list[19] = new_list[19], new_list[7]
                elif(i == "b" or i == "d"):    
                    new_list[1], new_list[37] = new_list[37], new_list[1]
                    new_list[4], new_list[40] = new_list[40], new_list[4]
                    new_list[7], new_list[43] = new_list[43], new_list[7]
                    
            elif(l%3 == 2):          
                if(i == "0" or i == "4" or i == "8" or i == "c"):
                    pass
                elif(i == "1" or i == "7"):
                    new_list[6], new_list[15] = new_list[15], new_list[6]
                    new_list[7], new_list[16] = new_list[16], new_list[7]
                    new_list[8], new_list[17] = new_list[17], new_list[8]
                elif(i == "2" or i == "6"):
                    new_list[6], new_list[24] = new_list[24], new_list[6]
                    new_list[7], new_list[25] = new_list[25], new_list[7]
                    new_list[8], new_list[26] = new_list[26], new_list[8]
                elif(i == "3" or i == "5"):    
                    new_list[6], new_list[33] = new_list[33], new_list[6]
                    new_list[7], new_list[34] = new_list[34], new_list[7]
                    new_list[8], new_list[35] = new_list[35], new_list[8]
                elif(i == "9" or i == "f"):
                    new_list[2], new_list[47] = new_list[47], new_list[2]
                    new_list[5], new_list[50] = new_list[50], new_list[5]
                    new_list[8], new_list[53] = new_list[53], new_list[8]
                elif(i == "a" or i == "e"):
                    new_list[2], new_list[24] = new_list[24], new_list[2]
                    new_list[5], new_list[21] = new_list[21], new_list[5]
                    new_list[8], new_list[18] = new_list[18], new_list[8]
                elif(i == "b" or i == "d"):    
                    new_list[2], new_list[38] = new_list[38], new_list[2]
                    new_list[5], new_list[41] = new_list[41], new_list[5]
                    new_list[8], new_list[44] = new_list[44], new_list[8]
            l = l+1
            
        l = 0           
        for i in k52:
            if(l%3 == 0):
                if(i == "0" or i == "4" or i == "8" or i == "c"):
                    pass
                elif(i == "1" or i == "7"):
                    new_list[10], new_list[19] = new_list[19], new_list[10]
                    new_list[11], new_list[20] = new_list[20], new_list[11]
                    new_list[12], new_list[21] = new_list[21], new_list[12]
                    
                elif(i == "2" or i == "6"):
                    new_list[10], new_list[28] = new_list[28], new_list[10]
                    new_list[11], new_list[29] = new_list[29], new_list[11]
                    new_list[12], new_list[30] = new_list[30], new_list[12]
                    
                elif(i == "3" or i == "5"):   
                    new_list[10], new_list[37] = new_list[37], new_list[10]
                    new_list[11], new_list[38] = new_list[38], new_list[11]
                    new_list[12], new_list[39] = new_list[39], new_list[12]

                elif(i == "9" or i == "f"):
                    new_list[10], new_list[55] = new_list[55], new_list[10]
                    new_list[13], new_list[58] = new_list[58], new_list[13]
                    new_list[16], new_list[61] = new_list[61], new_list[16]

                elif(i == "a" or i == "e"):
                    new_list[10], new_list[36] = new_list[36], new_list[10]
                    new_list[13], new_list[33] = new_list[33], new_list[13]
                    new_list[16], new_list[30] = new_list[30], new_list[16]
                    
                elif(i == "b" or i == "d"):
                    new_list[10], new_list[46] = new_list[46], new_list[10]
                    new_list[13], new_list[49] = new_list[49], new_list[13]
                    new_list[16], new_list[52] = new_list[52], new_list[16]
                    
            elif(l%3 == 1):      
                if(i == "0" or i == "4" or i == "8" or i == "c"):
                    pass
                elif(i == "1" or i == "7"):
                    new_list[13], new_list[22] = new_list[22], new_list[10]
                    new_list[14], new_list[23] = new_list[23], new_list[11]
                    new_list[15], new_list[24] = new_list[24], new_list[12]
                    
                elif(i == "2" or i == "6"):
                    new_list[13], new_list[31] = new_list[31], new_list[13]
                    new_list[14], new_list[32] = new_list[32], new_list[14]
                    new_list[15], new_list[33] = new_list[33], new_list[15]
                    
                elif(i == "3" or i == "5"):    
                    new_list[13], new_list[40] = new_list[40], new_list[13]
                    new_list[14], new_list[41] = new_list[41], new_list[14]
                    new_list[15], new_list[42] = new_list[42], new_list[15]
                    
                elif(i == "9" or i == "f"):
                    new_list[11], new_list[56] = new_list[56], new_list[10]
                    new_list[14], new_list[59] = new_list[59], new_list[11]
                    new_list[17], new_list[62] = new_list[62], new_list[12]
                    
                elif(i == "a" or i == "e"):
                    new_list[11], new_list[35] = new_list[35], new_list[10]
                    new_list[14], new_list[32] = new_list[32], new_list[11]
                    new_list[17], new_list[29] = new_list[29], new_list[12]
                    
                elif(i == "b" or i == "d"):    
                    new_list[11], new_list[47] = new_list[47], new_list[10]
                    new_list[14], new_list[50] = new_list[50], new_list[11]
                    new_list[17], new_list[53] = new_list[53], new_list[12]
                    
            elif(l%3 == 2):          
                if(i == "0" or i == "4" or i == "8" or i == "c"):
                    pass
                elif(i == "1" or i == "7"):
                    new_list[16], new_list[25] = new_list[25], new_list[16]
                    new_list[17], new_list[26] = new_list[26], new_list[17]
                    new_list[18], new_list[27] = new_list[27], new_list[18]
                    
                elif(i == "2" or i == "6"):
                    new_list[16], new_list[34] = new_list[34], new_list[16]
                    new_list[17], new_list[35] = new_list[35], new_list[17]
                    new_list[18], new_list[36] = new_list[36], new_list[18]
                
                elif(i == "3" or i == "5"):   
                    new_list[16], new_list[43] = new_list[43], new_list[16]
                    new_list[17], new_list[44] = new_list[44], new_list[17]
                    new_list[18], new_list[45] = new_list[45], new_list[18]
                    
                elif(i == "9" or i == "f"):
                    new_list[12], new_list[57] = new_list[57], new_list[12]
                    new_list[15], new_list[60] = new_list[60], new_list[15]
                    new_list[18], new_list[63] = new_list[63], new_list[18]
                    
                elif(i == "a" or i == "e"):
                    new_list[12], new_list[34] = new_list[34], new_list[12]
                    new_list[15], new_list[31] = new_list[31], new_list[15]
                    new_list[18], new_list[28] = new_list[28], new_list[18]
                    
                elif(i == "b" or i == "d"):   
                    new_list[12], new_list[48] = new_list[48], new_list[12]
                    new_list[15], new_list[51] = new_list[51], new_list[15]
                    new_list[18], new_list[54] = new_list[54], new_list[18]
            l = l+1
                    
            for i in [0, 10, 19, 28, 37, 46, 55]:
                if new_list[i] == "0":
                    new_list[i] = "8"
                elif new_list[i] == "1":
                    new_list[i] = "9"
                elif new_list[i] == "2":
                    new_list[i] = "a"
                elif new_list[i] == "3":
                    new_list[i] = "b"
                elif new_list[i] == "4":
                    new_list[i] = "c"
                elif new_list[i] == "5":
                    new_list[i] = "d"
                elif new_list[i] == "6":
                    new_list[i] = "e"
                elif new_list[i] == "7":
                    new_list[i] = "f"
                elif new_list[i] == "f":
                    new_list[i] = "7"
                elif new_list[i] == "e":
                    new_list[i] = "6"
                elif new_list[i] == "d":
                    new_list[i] = "5"
                elif new_list[i] == "c":
                    new_list[i] = "4"
                elif new_list[i] == "b":
                    new_list[i] = "3"
                elif new_list[i] == "a":
                    new_list[i] = "2"
                elif new_list[i] == "9":
                    new_list[i] = "1"
                elif new_list[i] == "8":
                    new_list[i] = "0"
                    
                    
            
    elif(num == 2):
        l = 0
        for i in k53:
            if(l%3 == 0):
                if(i == "0" or i == "4" or i == "8" or i == "c"):
                    pass
                if(i == "1" or i == "7"):
                    new_list[0], new_list[9] = new_list[9], new_list[0]
                    new_list[1], new_list[10] = new_list[10], new_list[1]
                    new_list[2], new_list[11] = new_list[11], new_list[2]
                elif(i == "2" or i == "6"):
                    new_list[0], new_list[15] = new_list[15], new_list[0]
                    new_list[1], new_list[19] = new_list[19], new_list[1]
                    new_list[2], new_list[20] = new_list[20], new_list[2]
                elif(i == "3" or i == "5"):    
                    new_list[0], new_list[27] = new_list[27], new_list[0]
                    new_list[1], new_list[28] = new_list[28], new_list[1]
                    new_list[2], new_list[29] = new_list[29], new_list[2]
                elif(i == "9" or i == "f"):
                    new_list[0], new_list[45] = new_list[45], new_list[0]
                    new_list[3], new_list[48] = new_list[48], new_list[3]
                    new_list[6], new_list[51] = new_list[51], new_list[6]
                elif(i == "a" or i == "e"):
                    new_list[0], new_list[26] = new_list[26], new_list[0]
                    new_list[3], new_list[23] = new_list[23], new_list[3]
                    new_list[6], new_list[20] = new_list[20], new_list[6]
                elif(i == "b" or i == "d"):    
                    new_list[0], new_list[36] = new_list[36], new_list[0]
                    new_list[3], new_list[39] = new_list[39], new_list[3]
                    new_list[6], new_list[42] = new_list[42], new_list[6]
                                          
                                        
            elif(l%3 == 1):      
                if(i == "0" or i == "4" or i == "8" or i == "c"):
                    pass
                if(i == "1" or i == "7"):
                    new_list[3], new_list[12] = new_list[12], new_list[3]
                    new_list[4], new_list[13] = new_list[13], new_list[4]
                    new_list[5], new_list[14] = new_list[14], new_list[5]
                elif(i == "2" or i == "6"):
                    new_list[3], new_list[21] = new_list[21], new_list[3]
                    new_list[4], new_list[22] = new_list[22], new_list[4]
                    new_list[5], new_list[23] = new_list[23], new_list[5]
                elif(i == "3" or i == "5"):    
                    new_list[3], new_list[30] = new_list[30], new_list[3]
                    new_list[4], new_list[31] = new_list[31], new_list[4]
                    new_list[5], new_list[32] = new_list[32], new_list[5]
                elif(i == "9" or i == "f"):
                    new_list[1], new_list[46] = new_list[46], new_list[1]
                    new_list[4], new_list[49] = new_list[49], new_list[4]
                    new_list[7], new_list[52] = new_list[52], new_list[7]
                elif(i == "a" or i == "e"):
                    new_list[1], new_list[25] = new_list[25], new_list[1]
                    new_list[4], new_list[22] = new_list[22], new_list[4]
                    new_list[7], new_list[19] = new_list[19], new_list[7]
                elif(i == "b" or i == "d"):    
                    new_list[1], new_list[37] = new_list[37], new_list[1]
                    new_list[4], new_list[40] = new_list[40], new_list[4]
                    new_list[7], new_list[43] = new_list[43], new_list[7]
                    
            elif(l%3 == 2):          
                if(i == "0" or i == "4" or i == "8" or i == "c"):
                    pass
                elif(i == "1" or i == "7"):
                    new_list[6], new_list[15] = new_list[15], new_list[6]
                    new_list[7], new_list[16] = new_list[16], new_list[7]
                    new_list[8], new_list[17] = new_list[17], new_list[8]
                elif(i == "2" or i == "6"):
                    new_list[6], new_list[24] = new_list[24], new_list[6]
                    new_list[7], new_list[25] = new_list[25], new_list[7]
                    new_list[8], new_list[26] = new_list[26], new_list[8]
                elif(i == "3" or i == "5"):    
                    new_list[6], new_list[33] = new_list[33], new_list[6]
                    new_list[7], new_list[34] = new_list[34], new_list[7]
                    new_list[8], new_list[35] = new_list[35], new_list[8]
                elif(i == "9" or i == "f"):
                    new_list[2], new_list[47] = new_list[47], new_list[2]
                    new_list[5], new_list[50] = new_list[50], new_list[5]
                    new_list[8], new_list[53] = new_list[53], new_list[8]
                elif(i == "a" or i == "e"):
                    new_list[2], new_list[24] = new_list[24], new_list[2]
                    new_list[5], new_list[21] = new_list[21], new_list[5]
                    new_list[8], new_list[18] = new_list[18], new_list[8]
                elif(i == "b" or i == "d"):    
                    new_list[2], new_list[38] = new_list[38], new_list[2]
                    new_list[5], new_list[41] = new_list[41], new_list[5]
                    new_list[8], new_list[44] = new_list[44], new_list[8]
            l = l+1
            
        l = 0            
        for i in k54:
            if(l%3 == 0):
                if(i == "0" or i == "4" or i == "8" or i == "c"):
                    pass
                elif(i == "1" or i == "7"):
                    new_list[10], new_list[19] = new_list[19], new_list[10]
                    new_list[11], new_list[20] = new_list[20], new_list[11]
                    new_list[12], new_list[21] = new_list[21], new_list[12]
                    
                elif(i == "2" or i == "6"):
                    new_list[10], new_list[28] = new_list[28], new_list[10]
                    new_list[11], new_list[29] = new_list[29], new_list[11]
                    new_list[12], new_list[30] = new_list[30], new_list[12]
                    
                elif(i == "3" or i == "5"):   
                    new_list[10], new_list[37] = new_list[37], new_list[10]
                    new_list[11], new_list[38] = new_list[38], new_list[11]
                    new_list[12], new_list[39] = new_list[39], new_list[12]

                elif(i == "9" or i == "f"):
                    new_list[10], new_list[55] = new_list[55], new_list[10]
                    new_list[13], new_list[58] = new_list[58], new_list[13]
                    new_list[16], new_list[61] = new_list[61], new_list[16]

                elif(i == "a" or i == "e"):
                    new_list[10], new_list[36] = new_list[36], new_list[10]
                    new_list[13], new_list[33] = new_list[33], new_list[13]
                    new_list[16], new_list[30] = new_list[30], new_list[16]
                    
                elif(i == "b" or i == "d"):
                    new_list[10], new_list[46] = new_list[46], new_list[10]
                    new_list[13], new_list[49] = new_list[49], new_list[13]
                    new_list[16], new_list[52] = new_list[52], new_list[16]
                    
            elif(l%3 == 1):      
                if(i == "0" or i == "4" or i == "8" or i == "c"):
                    pass
                elif(i == "1" or i == "7"):
                    new_list[13], new_list[22] = new_list[22], new_list[10]
                    new_list[14], new_list[23] = new_list[23], new_list[11]
                    new_list[15], new_list[24] = new_list[24], new_list[12]
                    
                elif(i == "2" or i == "6"):
                    new_list[13], new_list[31] = new_list[31], new_list[13]
                    new_list[14], new_list[32] = new_list[32], new_list[14]
                    new_list[15], new_list[33] = new_list[33], new_list[15]
                    
                elif(i == "3" or i == "5"):    
                    new_list[13], new_list[40] = new_list[40], new_list[13]
                    new_list[14], new_list[41] = new_list[41], new_list[14]
                    new_list[15], new_list[42] = new_list[42], new_list[15]
                    
                elif(i == "9" or i == "f"):
                    new_list[11], new_list[56] = new_list[56], new_list[10]
                    new_list[14], new_list[59] = new_list[59], new_list[11]
                    new_list[17], new_list[62] = new_list[62], new_list[12]
                    
                elif(i == "a" or i == "e"):
                    new_list[11], new_list[35] = new_list[35], new_list[10]
                    new_list[14], new_list[32] = new_list[32], new_list[11]
                    new_list[17], new_list[29] = new_list[29], new_list[12]
                    
                elif(i == "b" or i == "d"):    
                    new_list[11], new_list[47] = new_list[47], new_list[10]
                    new_list[14], new_list[50] = new_list[50], new_list[11]
                    new_list[17], new_list[53] = new_list[53], new_list[12]
                    
            elif(l%3 == 2):          
                if(i == "0" or i == "4" or i == "8" or i == "c"):
                    pass
                elif(i == "1" or i == "7"):
                    new_list[16], new_list[25] = new_list[25], new_list[16]
                    new_list[17], new_list[26] = new_list[26], new_list[17]
                    new_list[18], new_list[27] = new_list[27], new_list[18]
                    
                elif(i == "2" or i == "6"):
                    new_list[16], new_list[34] = new_list[34], new_list[16]
                    new_list[17], new_list[35] = new_list[35], new_list[17]
                    new_list[18], new_list[36] = new_list[36], new_list[18]
                
                elif(i == "3" or i == "5"):   
                    new_list[16], new_list[43] = new_list[43], new_list[16]
                    new_list[17], new_list[44] = new_list[44], new_list[17]
                    new_list[18], new_list[45] = new_list[45], new_list[18]
                    
                elif(i == "9" or i == "f"):
                    new_list[12], new_list[57] = new_list[57], new_list[12]
                    new_list[15], new_list[60] = new_list[60], new_list[15]
                    new_list[18], new_list[63] = new_list[63], new_list[18]
                    
                elif(i == "a" or i == "e"):
                    new_list[12], new_list[34] = new_list[34], new_list[12]
                    new_list[15], new_list[31] = new_list[31], new_list[15]
                    new_list[18], new_list[28] = new_list[28], new_list[18]
                    
                elif(i == "b" or i == "d"):   
                    new_list[12], new_list[48] = new_list[48], new_list[12]
                    new_list[15], new_list[51] = new_list[51], new_list[15]
                    new_list[18], new_list[54] = new_list[54], new_list[18]
            l = l+1
                    
            for i in [0, 10, 19, 28, 37, 46, 55]:
                if new_list[i] == "0":
                    new_list[i] = "8"
                elif new_list[i] == "1":
                    new_list[i] = "9"
                elif new_list[i] == "2":
                    new_list[i] = "a"
                elif new_list[i] == "3":
                    new_list[i] = "b"
                elif new_list[i] == "4":
                    new_list[i] = "c"
                elif new_list[i] == "5":
                    new_list[i] = "d"
                elif new_list[i] == "6":
                    new_list[i] = "e"
                elif new_list[i] == "7":
                    new_list[i] = "f"
                elif new_list[i] == "f":
                    new_list[i] = "7"
                elif new_list[i] == "e":
                    new_list[i] = "6"
                elif new_list[i] == "d":
                    new_list[i] = "5"
                elif new_list[i] == "c":
                    new_list[i] = "4"
                elif new_list[i] == "b":
                    new_list[i] = "3"
                elif new_list[i] == "a":
                    new_list[i] = "2"
                elif new_list[i] == "9":
                    new_list[i] = "1"
                elif new_list[i] == "8":
                    new_list[i] = "0"     
    
    print(new_list)                
    mval = hex2bin("".join(new_list))
    print(mval)
    pval = '{:016b}'.format((int(mval[0:64], 2)^int(mval[64:128], 2)^int(mval[128:192], 2)^int(mval[192:256], 2)))
    kval = '{:016b}'.format((int(pval[0:16], 2)^int(pval[16:32], 2)^int(pval[32:48], 2)^int(pval[48:64], 2)))
    return kval


# In[3]:


def key_gen(ssn_key):
    ssn_key = list(session_key)
    #print(ssn_key)
    ssn_key1 = ssn_key[0:64]
    ssn_key2 = ssn_key[64:128]
    ssn_key3 = ssn_key[128:192]
    ssn_key4 = ssn_key[192:256]
    ssn_key5 = ssn_key[256:320]
    k1 = rubix(ssn_key1, ssn_key5, 1)
    k2 = rubix(ssn_key2, ssn_key5, 2)
    k3 = rubix(ssn_key3, ssn_key5, 1)
    k4 = rubix(ssn_key4, ssn_key5, 2)
    pval = int(k1, 2)^int(k2, 2)^int(k3, 2)^int(k4,2)&int('1111111111111111',2)
    k5 = '{:016b}'.format(pval)
    return k1, k2, k3, k4, k5


# In[4]:


def P(I):
    O=[]
    if(I==['0', '0', '0', '0']):
        O = ['0', '0', '1', '1'] 
    elif(I==['0', '0', '0', '1']):
        O = ['1', '1', '1', '1'] 
    elif(I==['0', '0', '1', '0']):
        O = ['1', '1', '1', '0']
    elif(I==['0', '0', '1', '1']):
        O = ['0', '0', '0', '0'] 
    elif(I==['0', '1', '0', '0']):
        O = ['0', '1', '0', '1'] 
    elif(I==['0', '1', '0', '1']):
        O = ['0', '1', '0', '0'] 
    elif(I==['0', '1', '1', '0']):
        O = ['1', '0', '1', '1']   
    elif(I==['0', '1', '1', '1']):
        O = ['1', '1', '0', '0']   
    elif(I==['1', '0', '0', '0']):
        O = ['1', '1', '0', '1']   
    elif(I==['1', '0', '0', '1']):
        O = ['1', '0', '1', '0'] 
    elif(I==['1', '0', '1', '0']):
        O = ['1', '0', '0', '1']   
    elif(I==['1', '0', '1', '1']):
        O = ['0', '1', '1', '0']    
    elif(I==['1', '1', '0', '0']):
        O = ['0', '1', '1', '1']   
    elif(I==['1', '1', '0', '1']):
        O = ['1', '0', '0', '0']   
    elif(I==['1', '1', '1', '0']):
        O = ['0', '0', '1', '0']   
    elif(I==['1', '1', '1', '1']):
        O = ['0', '0', '0', '1']
    return O


# In[5]:


def Q(I):
    O = []
    if(I==['0', '0', '0', '0']):
        O = ['1', '0', '0', '1']
    elif(I==['0', '0', '0', '1']):
        O = ['1', '1', '1', '0']
    elif(I==['0', '0', '1', '0']):
        O = ['0', '1', '0', '1']
    elif(I==['0', '0', '1', '1']):
        O = ['0', '1', '1', '0']
    elif(I==['0', '1', '0', '0']):
        O = ['1', '0', '1', '0']
    elif(I==['0', '1', '0', '1']):
        O = ['0', '0', '1', '0']
    elif(I==['0', '1', '1', '0']):
        O = ['0', '0', '1', '1']
    elif(I==['0', '1', '1', '1']):
        O = ['1', '1', '0', '0']
    elif(I==['1', '0', '0', '0']):
        O = ['1', '1', '1', '1']
    elif(I==['1', '0', '0', '1']):
        O = ['0', '0', '0', '0']
    elif(I==['1', '0', '1', '0']):
        O = ['0', '1', '0', '0']
    elif(I==['1', '0', '1', '1']):
        O = ['1', '1', '0', '1']
    elif(I==['1', '1', '0', '0']):
        O = ['0', '1', '1', '1']
    elif(I==['1', '1', '0', '1']):
        O = ['1', '0', '1', '1']
    elif(I==['1', '1', '1', '0']):
        O = ['0', '0', '0', '1']
    elif(I==['1', '1', '1', '1']):
        O = ['1', '0', '0', '0']
    return O


# In[6]:


def f_fun(a):
    new_a = a[:]
    x = [j for i in new_a for j in i]
    new_a=[P(new_a[0]),Q(new_a[1]),P(new_a[2]),Q(new_a[3])]    
    new_a=[Q([j for i in [x[0:2],x[4:6]] for j in i]),P([j for i in [x[2:4],x[6:8]] for j in i]),Q([j for i in [x[8:10],x[12:14]] for j in i]),P([j for i in [x[10:12],x[14:16]] for j in i])]
    new_a=[P([j for i in [x[0:2],x[4:6]] for j in i]),Q([j for i in [x[2:4],x[6:8]] for j in i]),P([j for i in [x[8:10],x[12:14]] for j in i]),Q([j for i in [x[10:12],x[14:16]] for j in i])]
    new_a=[j for i in new_a for j in i]
    return new_a


# In[7]:

print("Enter session key : ", end = "")
session_key = str(input())
#"cda5340dc0a9bba1c07d6109b14b9d59d6901ad6e58eb04c23355fb2ee9256ae27d511656a21bfad3750bb6e79dcdf963ce7bca6dc0524be26fbd02b280ea635d6eaffe28552ccbf01a8c9d80577dd62afc7d97fb4a30072fd56ce244095c69819a0a6cdd486b11edf555cae0abdae3de7842bb01c47592a25a036ac3b6b59713a4f7f78b4da9e125b3352ff1aacaeb480013b7bbaed0e07bf6c094dec725984"


# In[8]:


k1, k2, k3, k4, k5 = key_gen(session_key)


# In[9]:


print("".join(k1))
print("".join(k2))
print("".join(k3))
print("".join(k4))
print("".join(k5))


# In[10]:

print("Enter message to be encrypted : ", end = "")
message = str(input())

#message="0010101100100011101101000011010110001110100010100101110100011101"
message=list(message)
print(message)


# In[11]:


#Encryption
t1 = message[0:16]
t2 = message[16:32]
t3 = message[32:48]
t4 = message[48:64]

p1 = '{:016b}'.format(int(bin(~(int("".join(t1), 2) ^ int("".join(k1), 2)) & int('1111111111111111',2)),2))
p2 = '{:016b}'.format(int("".join(f_fun([list(p1[0:4]), list(p1[4:8]), list(p1[8:12]), list(p1[12:16])])),2) ^ int("".join(t3), 2) & int('1111111111111111',2))
p4 = '{:016b}'.format(int(bin(~(int("".join(t4), 2) ^ int("".join(k1), 2)) & int('1111111111111111',2)),2))
p3 = '{:016b}'.format(int("".join(f_fun([list(p4[0:4]), list(p4[4:8]), list(p4[8:12]), list(p4[12:16])])),2) ^ int("".join(t2), 2) & int('1111111111111111',2))

q1 = p2
q2 = p1
q3 = p4
q4 = p3

r1 = '{:016b}'.format(int(bin(~(int("".join(q1), 2) ^ int("".join(k2), 2)) & int('1111111111111111',2)),2))
r2 = '{:016b}'.format(int("".join(f_fun([list(r1[0:4]), list(r1[4:8]), list(r1[8:12]), list(r1[12:16])])),2) ^ int("".join(q2), 2) & int('1111111111111111',2))
r4 = '{:016b}'.format(int(bin(~(int("".join(q4), 2) ^ int("".join(k2), 2)) & int('1111111111111111',2)),2))
r3 = '{:016b}'.format(int("".join(f_fun([list(r4[0:4]), list(r4[4:8]), list(r4[8:12]), list(r4[12:16])])),2) ^ int("".join(q3), 2) & int('1111111111111111',2))

s1 = '{:016b}'.format(int(bin(~(int("".join(r1), 2) ^ int("".join(k3), 2)) & int('1111111111111111',2)),2))
s2 = '{:016b}'.format(int("".join(f_fun([list(s1[0:4]), list(s1[4:8]), list(s1[8:12]), list(s1[12:16])])),2) ^ int("".join(r3), 2) & int('1111111111111111',2))
s4 = '{:016b}'.format(int(bin(~(int("".join(r4), 2) ^ int("".join(k3), 2)) & int('1111111111111111',2)),2))
s3 = '{:016b}'.format(int("".join(f_fun([list(s4[0:4]), list(s4[4:8]), list(s4[8:12]), list(s4[12:16])])),2) ^ int("".join(r2), 2) & int('1111111111111111',2))

a1 = s2
a2 = s1
a3 = s4
a4 = s3

b1 = '{:016b}'.format(int(bin(~(int("".join(a1), 2) ^ int("".join(k4), 2)) & int('1111111111111111',2)),2))
b2 = '{:016b}'.format(int("".join(f_fun([list(b1[0:4]), list(b1[4:8]), list(b1[8:12]), list(b1[12:16])])),2) ^ int("".join(a2), 2) & int('1111111111111111',2))
b4 = '{:016b}'.format(int(bin(~(int("".join(a4), 2) ^ int("".join(k4), 2)) & int('1111111111111111',2)),2))
b3 = '{:016b}'.format(int("".join(f_fun([list(b4[0:4]), list(b4[4:8]), list(b4[8:12]), list(b4[12:16])])),2) ^ int("".join(a3), 2) & int('1111111111111111',2))

lst = [b1, b2, b3, b4]


# In[12]:


cipher = [j for i in lst for j in i]
cipher = "".join(cipher)
print("Cipher Text : ", cipher)


# In[13]:


#Decryption
r1 = '{:016b}'.format(int(bin(~(int("".join(b1), 2) ^ int("".join(k4), 2)) & int('1111111111111111',2)),2))
r2 = '{:016b}'.format(int("".join(f_fun([list(b1[0:4]), list(b1[4:8]), list(b1[8:12]), list(b1[12:16])])),2) ^ int("".join(b2), 2) & int('1111111111111111',2))
r4 = '{:016b}'.format(int(bin(~(int("".join(b4), 2) ^ int("".join(k4), 2)) & int('1111111111111111',2)),2))
r3 = '{:016b}'.format(int("".join(f_fun([list(b4[0:4]), list(b4[4:8]), list(b4[8:12]), list(b4[12:16])])),2) ^ int("".join(b3), 2) & int('1111111111111111',2))

q1 = r2
q2 = r1
q3 = r4
q4 = r3

s1 = '{:016b}'.format(int(bin(~(int("".join(q1), 2) ^ int("".join(k3), 2)) & int('1111111111111111',2)),2))
s3 = '{:016b}'.format(int("".join(f_fun([list(q1[0:4]), list(q1[4:8]), list(q1[8:12]), list(q1[12:16])])),2) ^ int("".join(q2), 2) & int('1111111111111111',2))
s4 = '{:016b}'.format(int(bin(~(int("".join(q4), 2) ^ int("".join(k3), 2)) & int('1111111111111111',2)),2))
s2 = '{:016b}'.format(int("".join(f_fun([list(q4[0:4]), list(q4[4:8]), list(q4[8:12]), list(q4[12:16])])),2) ^ int("".join(q3), 2) & int('1111111111111111',2))

b1 = '{:016b}'.format(int(bin(~(int("".join(s1), 2) ^ int("".join(k2), 2)) & int('1111111111111111',2)),2))
b2 = '{:016b}'.format(int("".join(f_fun([list(s1[0:4]), list(s1[4:8]), list(s1[8:12]), list(s1[12:16])])),2) ^ int("".join(s2), 2) & int('1111111111111111',2))
b4 = '{:016b}'.format(int(bin(~(int("".join(s4), 2) ^ int("".join(k2), 2)) & int('1111111111111111',2)),2))
b3 = '{:016b}'.format(int("".join(f_fun([list(s4[0:4]), list(s4[4:8]), list(s4[8:12]), list(s4[12:16])])),2) ^ int("".join(s3), 2) & int('1111111111111111',2))

a1 = b2
a2 = b1
a3 = b4
a4 = b3

boo1 = '{:016b}'.format(int(bin(~(int("".join(a1), 2) ^ int("".join(k1), 2)) & int('1111111111111111',2)),2))
boo3 = '{:016b}'.format(int("".join(f_fun([list(a1[0:4]), list(a1[4:8]), list(a1[8:12]), list(a1[12:16])])),2) ^ int("".join(a2), 2) & int('1111111111111111',2))
boo4 = '{:016b}'.format(int(bin(~(int("".join(a4), 2) ^ int("".join(k1), 2)) & int('1111111111111111',2)),2))
boo2 = '{:016b}'.format(int("".join(f_fun([list(a4[0:4]), list(a4[4:8]), list(a4[8:12]), list(a4[12:16])])),2) ^ int("".join(a3), 2) & int('1111111111111111',2))

lst = [boo1, boo2, boo3, boo4]


# In[14]:


msg = [j for i in lst for j in i]
msg="".join(msg)
print("Decrypted Text : ",msg)


# In[15]:


m ="".join(message)
print("Original Message : ", m)


# In[16]:


m == msg


# In[17]:


print("Decrypted Text and Original Message is same")


# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:




