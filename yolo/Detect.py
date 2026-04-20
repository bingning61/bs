import warnings
warnings.filterwarnings('ignore')
from ultralytics import YOLO

if __name__ == '__main__':
    model = YOLO('训练出来的权重文件地址') # select your model.pt path
    model.predict(source='想要检测文件的地址',
                  imgsz=640,
                  project='runs/detect',
                  name='exp',
                  save=True,
                  classes=1, # 是否指定检测某个类别.
                )