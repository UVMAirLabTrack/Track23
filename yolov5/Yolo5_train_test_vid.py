
import torch
import cv2

path = '/media/dev/49FE-5759/yolov5'
pathapp = path + '/runs/train/exp20/weights/last.pt'
#model = torch.hub.load('C:\\Users\\luisa\\Dropbox\\UVMstuff\\AIR lab\\Yolo8_Training\\yolov5', 'last', source = 'local')
model = torch.hub.load(path, 'custom', path=pathapp, force_reload=True,source='local') 

cap = cv2.VideoCapture(0) 

while True:
    ret, frame = cap.read()
    if not ret:
        break
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB
    result = model(img)
   # print(result)  

    df = result.pandas().xyxy[0]
    #print(df)  
    for ind in df.index:
        x1, y1 = int(df['xmin'][ind]), int(df['ymin'][ind])
        x2, y2 = int(df['xmax'][ind]), int(df['ymax'][ind])
        label = df['name'][ind]
        cv2.rectangle(img, (x1, y1), (x2,y2), (255, 255, 0), 2)
        cv2.putText(img, label, (x1, y1-5), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 0), 2)
           
    cv2.imshow('IMAGE', img)
    cv2.waitKey(5)
        
    
'''

python train.py --img 416 --batch 16 --epochs 100 --data custom_data.yaml --weights yolov5s.pt --nosave --cache

python detect.py --weights runs/train/exp6/weights/best.pt --img 416 --conf 0.4 --source 'C:\\Users\\luisa\\Dropbox\\UVMstuff\\AIR lab\\Yolo8_Training\\test_BCCD.jpg'


cap = cv2.VideoCapture('video.mp4')

while True:
    img = cap.read()[1]
    if img is None:
        break
    result = model(img)
    df = result.pandas().xyxy[0]

    for ind in df.index:
        x1, y1 = int(df['xmin'][ind]), int(df['ymin'][ind])
        x2, y2 = int(df['xmax'][ind]), int(df['ymax'][ind])
        label = df['name'][ind]
        conf = df['confidence'][ind]
        text = label + ' ' + str(conf.round(decimals= 2))
        cv2.rectangle(img, (x1, y1), (x2, y2), (255, 255, 0), 2)
        cv2.putText(img, text, (x1, y1 - 5), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 0), 2)

    cv2.imshow('Video',img)
    cv2.waitKey(10)
 '''   
