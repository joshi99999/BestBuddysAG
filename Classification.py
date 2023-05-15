import cv2
import numpy as np
import xlsxwriter
import os
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from sklearn import svm, preprocessing
from mlxtend.plotting import plot_decision_regions

def datei_open(directory):
    # Liste mit den Dateinamen im Verzeichnis erstellen
    files = os.listdir(directory)

    # Nur Dateinamen mit Bildernamen filtern
    image_names = [os.path.join(directory, file) for file in files if file.lower().endswith(('.png', '.jpg', '.jpeg'))]

    return image_names

def schwerpunkt(img):
    
    yind,xind=np.where(img)
    
    return np.array( (np.mean(yind), np.mean(xind)))

def traegheitstensor(binary_img):
    
    schwerpunkt_y,schwerpunkt_x = schwerpunkt(binary_img)
    matrix_traegheitstensor = np.zeros((2,2))
    y,x = np.where(binary_img)
    y = y-schwerpunkt_y
    x = x-schwerpunkt_x
    anzahl_pixel = float(len(y))
    matrix_traegheitstensor[0,0] = np.sum(y**2) / anzahl_pixel
    matrix_traegheitstensor[1,1] = np.sum(x**2) / anzahl_pixel
    matrix_traegheitstensor[0,1] = -np.sum(x*y) / anzahl_pixel
    matrix_traegheitstensor[1,0] = matrix_traegheitstensor[0,1] 
    
    return matrix_traegheitstensor
  
def Hauptmoment(e_vals):
    major_eigenvalue_idx = np.argmax(e_vals)
    minor_eigenvalue_idx = np.argmin(e_vals)
    major_eigenvalue = e_vals[major_eigenvalue_idx]
    minor_eigenvalue = e_vals[minor_eigenvalue_idx]
    I_x = major_eigenvalue 
    I_y = minor_eigenvalue 
    
    return I_x , I_y

def classify_image(image):
    
    bild = cv2.imread(image)
    
    binary_img=cv2.imread(image, cv2.IMREAD_GRAYSCALE)
    
    gray = cv2.cvtColor(bild, cv2.COLOR_BGR2GRAY)
    
    _,binary_img=cv2.threshold(binary_img, 0, 1, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    edges = cv2.Canny(blur, 300, 525)
   
    num_edges = cv2.countNonZero(edges)

    dst = cv2.cornerHarris(gray, 2, 3, 0.04)
    threshold = 0.02 * dst.max()
    corners = []
    for i in range(dst.shape[0]):
        for j in range(dst.shape[1]):
            if dst[i, j] > threshold:
                corners.append((i, j))
            
    schwerpunkt_y,schwerpunkt_x = schwerpunkt(binary_img)

    cv2.circle(edges, (int(schwerpunkt_x + 0.5), int(schwerpunkt_y + 0.5)), 5, (0,0,255), -1)
    
    e_vals, _= np.linalg.eig( traegheitstensor(binary_img) )
    
    I_x, I_y = Hauptmoment(e_vals)
    
    cv2.imshow('Einhorn', edges)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    features = [num_edges, I_x, I_y, len(corners)]
    return features

def pred(features,model):
    print()
    prediction = model.predict([features])
    if prediction == 1 : 
        print("Einhorn")
    elif prediction == 0 :
        print("Katze")
    else :
        print("-1")
    return 0

def load_train_data(file_path):
    data = pd.read_excel(file_path)
    X = data[[' Anzahl kanten', 'Trägheitsmoment_x', 'Trägheitsmoment_y', 'Anzahl Ecke']].values
    class_mapping = {'Katze': 0, 'Einhorn': 1}
    y = data['Klasse'].map(class_mapping).values
    return X, y
    
# Methode zum Trainieren des SVM-Modells
def train_svm(X_train, y_train):
    clf = svm.SVC(kernel='linear')
    clf.fit(X_train, y_train)
    return clf

train_file_path= 'Katze.xlsx'
X_train, y_train = load_train_data(train_file_path)
svm_model = train_svm(X_train, y_train)

#samples = datei_open('MiauTrain/')
#samples = datei_open('UnicornTrain/')
samples = datei_open('Test/')


for i in samples:
    features = classify_image(i)
    res = pred(features, svm_model)
    while res !=0: 
        pred(features=features, model=svm_model) 
