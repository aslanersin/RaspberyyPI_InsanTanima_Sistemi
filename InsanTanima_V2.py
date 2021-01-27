#Bu program Ersin ASLAN (171816502) tarafından kodlanmıştır.
#Mimari olarak TensorFlow ve OpenCV kullanılmıştır.

#Kullanilacak olan gerekli kutuphane tanimlamalari yapiliyor
import os

#TensorFlow'a ait loglama islemi icin kullanildi.
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'    

import pathlib
import tensorflow as tf
import cv2
import argparse
from threading import Thread

import RPi.GPIO as GPIO

#LCD'yi kullanabilmek icin tanimlandi
import InsanTanimaLCDkutuphane as mylcd
import time

#LCD panel icin gerekli ayarlamalar yapiliyor.
mylcd.pinIoAyarla()
mylcd.lcd_init()


#TensorFlow a ait loglama islemi yapiliyor
tf.get_logger().setLevel('ERROR')          

class VideoStream:
    #Kameraya ait gerekli tanimlamalar yapiliyor.
    
    def __init__(self,resolution=(640,480),framerate=30):
        #Kamera yakalamasi baslatilip gerekli tanimlamalar yapiliyor
        self.stream = cv2.VideoCapture(0)
        ret = self.stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        ret = self.stream.set(3,resolution[0])
        ret = self.stream.set(4,resolution[1])
            
        # Streamdan (canli akisdan) ilk frame okunuyor.
        (self.grabbed, self.frame) = self.stream.read()

        # Kamera stop edilirse ilgili degisken stop ediliyor.
        self.stopped = False

    #Gercek zamanli video akisindan karelerin okunmasi baslatiliyor.
    def start(self):
        Thread(target=self.update,args=()).start()
        return self

    #Kameradan frame alinma durumuna gore islemler yapiliyor.
    def update(self):
        # Thread durdurulana kadar sonsuz donguye sokuluyor
        while True:
            # Kameranin durdurulmasina bagli olarak thread de durduruluyor.
            if self.stopped:
                # Kamera kaynaklari kapatiliyor.
                self.stream.release()
                return

            # Kamera durmadiysa streamden kare alinmaya devam ediliyor.
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
    # En son kare dondurulur
        return self.frame

    def stop(self):
    # Kameranin durdurulmasi ile ilgili degisken set ediliyor.
        self.stopped = True
        

# Goruntu tanimada kullanilmak uzere TensorFlow a ait gerekli degisken ve kaynak tanimlamalari yapiliyor. 
parser = argparse.ArgumentParser()
parser.add_argument('--model', help='Folder that the Saved Model is Located In',
                    default='od-models/my_mobilenet_model')
parser.add_argument('--labels', help='Where the Labelmap is Located',
                    default='models/research/object_detection/data/mscoco_label_map.pbtxt')
parser.add_argument('--threshold', help='Minimum confidence threshold for displaying detected objects',
                    default=0.5)
                    
args = parser.parse_args()


# Goruntu tanima icin kullanilacak modelin yol (path) tanimlamasi yapiliyor.
PATH_TO_MODEL_DIR = args.model

# Modelin kullanacagi etiketlere ait siniflamanin (Insan,Canta, Kalem vb. gibi ) nerden alinacagina ait
# Yol tanimlamasi yapiliyor. 
PATH_TO_LABELS = args.labels

# Modele ait minimum guven esigi tanimlaniyor.
MIN_CONF_THRESH = float(args.threshold)

# Goruntu Islemede kullanilacak modelin yuklenmesi sureci basliyor.

import time
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as viz_utils

PATH_TO_SAVED_MODEL = PATH_TO_MODEL_DIR + "/saved_model"

print('Model Yukleniyor...', end='')
start_time = time.time()

# Onceden kaydedilmis olan model yükleniyor ve algilama islevi olusturuluyor.
detect_fn = tf.saved_model.load(PATH_TO_SAVED_MODEL)

end_time = time.time()
elapsed_time = end_time - start_time
print('Modelin Yuklemesi Tamamlandi, bu islem {} saniye surdu'.format(elapsed_time))


# Etiket verilerinin cizimin uzerine yazdirilmasi icin gereken surec baslatiliyor.

category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS,
                                                                    use_display_name=True)

#Ortamdaki insan sayisi belirli bir degeri gecince buzzer'dan ses cikacak 
def sesUret():

 GPIO.setwarnings(False)
 GPIO.setmode(GPIO.BCM)
 GPIO.setup(4, GPIO.OUT)

 GPIO.output(4,1)
 time.sleep(1)
 GPIO.output(4,0)
 time.sleep(1)

insanSayisiLimiti=2 #Burda belirlenen degere gore sesli uyari (buzzer) verilecek.

import numpy as np
import matplotlib.pyplot as plt
import warnings
warnings.filterwarnings('ignore')   

print('Kamera Calismasi basliyor')

#Gercek zamanli akis icin video stream baslatiliyor
videostream = VideoStream(resolution=(640,480),framerate=30).start()
while True:

    # Nesnelerin gosterilecegi cerceveye (frame) ait ayarlamalar yapiliyor.
    
    frame = videostream.read()
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    frame_expanded = np.expand_dims(frame_rgb, axis=0)
    imH, imW, _ = frame.shape

    # Giris TensorFlow mimarisine donusturuluyor
    input_tensor = tf.convert_to_tensor(frame)
    
    # Model goruntuyu toplu bir sekilde islediginden gerekli ayarlamalar yapiliyor
    input_tensor = input_tensor[tf.newaxis, ...]

    detections = detect_fn(input_tensor)

    
    # Numpy ile gerekli dizi islemleri yapildiktan sonra dizinin ilk degeri alinip 
    # Gerekli detection sureci baslatiliyor. Burda amac ilk detection'la islem yapabilmek
    num_detections = int(detections.pop('num_detections'))
    detections = {key: value[0, :num_detections].numpy()
                   for key, value in detections.items()}
    detections['num_detections'] = num_detections

    # Detection tipi int'e (sayisal) cevriliyor.
    detections['detection_classes'] = detections['detection_classes'].astype(np.int64)

    
    # Goruntuler uzerinde tespit yapabilmek icin minimum esik degeri belirleniyor
    detections['detection_classes'] = detections['detection_classes'].astype(np.int64)
    scores = detections['detection_scores']
    boxes = detections['detection_boxes']
    classes = detections['detection_classes']
    count = 0
    insanSayisi=0
    for i in range(len(scores)):
        if ((scores[i] > MIN_CONF_THRESH) and (scores[i] <= 1.0)):
            
            count += 1
            
            # Sinirlayici cerceve boyutlari ve ona ait degerler aliniyor.
            # Bu islemi amaci akis sonucu gelen goruntunun belirlenen sinir araliklarinin disina cikmasi engelleniyor.
            ymin = int(max(1,(boxes[i][0] * imH)))
            xmin = int(max(1,(boxes[i][1] * imW)))
            ymax = int(min(imH,(boxes[i][2] * imH)))
            xmax = int(min(imW,(boxes[i][3] * imW)))
            
            cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), (10, 255, 0), 2)
            
            # Etiket yazdirma islemi
            # Name degerine gore Modelde bulunan karsiligi araniyor.
            object_name = category_index[int(classes[i])]['name'] 
            
            # Bulunan nesneni adi ve aitlik orani yazdiriliyor. Ornegin; Insan: %72 gibi.
            label = '%s: %d%%' % (object_name, int(scores[i]*100)) 
            labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2) 
            label_ymin = max(ymin, labelSize[1] + 10) 
            cv2.rectangle(frame, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED) # Draw white box to put label text in
            
            #Degerler Frame'e bastiriliyor.
            cv2.putText(frame, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2) 
            
            # Modelde taninan nesne eger insan ise sayi bir arttiriliyor.
            if (object_name=='person'):
                insanSayisi+=1
                #insan sayisi belirlene limiti gecince ses uretiliyor
                if (insanSayisi>=insanSayisiLimiti):
                 sesUret()
            

    cv2.putText (frame,'Bulunan Nesne Sayisi : ' + str(count),(10,25),cv2.FONT_HERSHEY_SIMPLEX,1,(70,235,52),2,cv2.LINE_AA)
    cv2.putText(frame, "Bulunan Insan Sayisi:{}".format(insanSayisi),(10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 3)    
    cv2.imshow('Nesne Bulucu', frame)

    #Sonuclarin LCD panele yazdirilmasi islemleri yapiliyor.
    mylcd.lcd_string("Insan Sayisi:"+str(insanSayisi),mylcd.LCD_LINE_1)
    mylcd.lcd_string("Maske Takin...",mylcd.LCD_LINE_2)



    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

#Yakalama işlemine son veriliyor.
cv2.destroyAllWindows()
print("Islem Tamamlandi")
