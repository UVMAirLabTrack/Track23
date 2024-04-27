
import torch
import cv2

#model = torch.hub.load('C:\\Users\\luisa\\Dropbox\\UVMstuff\\AIR lab\\Yolo8_Training\\yolov5', 'last', source = 'local')
path = '/media/dev/49FE-5759/yolov5/'
pathapp = path + 'runs/train/exp20/weights/last.pt'
model = torch.hub.load(path, 'custom', path=pathapp, force_reload=True,source='local') 

img = cv2.imread(path+"test_signs_SEED7.jpg")

#img = cv2.rotate(img, cv2.ROTATE_180)

#img = cv2.resize(img, (1000, 650))

result = model(img)
print(result)  

df = result.pandas().xyxy[0]
print(df)  


for ind in df.index:
    x1, y1 = int(df['xmin'][ind]), int(df['ymin'][ind])
    x2, y2 = int(df['xmax'][ind]), int(df['ymax'][ind])
    label = df['name'][ind]
    cv2.rectangle(img, (x1, y1), (x2,y2), (255, 255, 0), 2)
    cv2.putText(img, label, (x1, y1-5), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 0), 2)
    
    
cv2.imshow('IMAGE', img)
cv2.waitKey(0)
    
    
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