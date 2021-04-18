import socket
import select
import time
from PIL import Image
import matplotlib.pyplot as plt 
import io 
from array import array
import numpy as np 
from IPython.display import clear_output

gg = []

def show_video_frame(imageBytes):
    imageBytesArray = bytearray(imageBytes)
    image = Image.open(io.BytesIO(imageBytesArray))
    array = np.array(image)
    plt.imshow(array)
    plt.show()
    clear_output(wait=True)
# Image.frombytes('RGBA', (320, 240), imageBytes)


try:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    plt.figure(figsize=(20, 20))

    
    s.bind(('192.168.1.21', 3333))
    # s.setblocking(False)

    # k  = 0 
    s.listen(1) 

    def timeout_recv(client, buffer_size, timeout):
        before = time.time()
        while (time.time() - before) < timeout:
            try:
                content = client.recv(buffer_size)
                return content
            except:
                pass
        return None

    plt.figure()

    while True:
        print("Server listening...")

        client, addr = s.accept()
        # client.settimeout(0.1)
        client.setblocking(False)    # print("AFTER ACCEPT ")
        # print("before")
        #size = client.recv(4)
        #print(size)
        # total = b''
        # while True:
        #     content = client.recv(2*8192)
        #     size = len(content)
        #     if size == 0: 
        #         break
        imageBytes = b''
        before = time.time()
        num_frames = 0
        max_framerate = 0
        min_framerate = 1000
        SECOND = 1
        count = 0
        while True:
            content = timeout_recv(client, 2**9, 5)

            if not content: break

            if len(content) == 12:
                num_frames += 1
                after = time.time()
                min_framerate = min(min_framerate, num_frames)
                max_framerate = max(max_framerate, num_frames)
                
                if after - before > SECOND: # trigger in SEC sec
                    gg.append(num_frames / SECOND)
                    print(f'Framerate: {num_frames / SECOND} min {min_framerate / SECOND}, max {max_framerate / SECOND}')
                    num_frames = 0
                    before = time.time()
                #shiow image
                if not len(imageBytes) == 0:
                    try:
                        show_video_frame(imageBytes)
                    except:
                        pass
                    # client.close()
                    # print('d')
                    # return imageBytes
                imageBytes = b''
            else:
                imageBytes  = imageBytes + content



            # print(f"Content length: {len(content)} bytes, content: '{str(content)}'")
            # print(f"Content length: {len(content)} bytes")
            
            

            
        # print(content)
        # k = k + 1
        # content = None
                    

        # print("Closing connection")
        client.close()
except Exception as e:
    print('runlarken patladik', e)
    try:
        client.close()
    except:
        print('e yapacak bisey yok artik')



print('Done!')

'''
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
'''
