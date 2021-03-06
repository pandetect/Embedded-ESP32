import socket
 
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)         
 
s.bind(('192.168.1.39', 3333 ))
            

# k  = 0 
s.listen(1) 

while True:
    client, addr = s.accept()
    # print("AFTER ACCEPT ")
    # print("before")
    #size = client.recv(4)
    #print(size)
    # total = b''
    # while True:
    #     content = client.recv(2*8192)
    #     size = len(content)
    #     if size == 0: 
    #         break

    content = client.recv(2*8192)

    print(f"Content length: {len(content)} bytes")
        
    
        
    # print(content)
    # k = k + 1
    # content = None
                
    
    # print("Closing connection")
    client.close()