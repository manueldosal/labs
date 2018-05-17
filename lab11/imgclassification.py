#!/usr/bin/env python

##############
#### Your name:
##############

import numpy as np
import re
from sklearn import svm, metrics
from skimage import io, feature, filters, exposure, color
import time
from sklearn.svm import LinearSVC

#TODO: Test with different values for: sigma, gamma
#TODO: Test other filters: canny
#TODO: test other learning algorithms: SVM

class ImageClassifier:
    
    def __init__(self):
        self.classifer = None

    def imread_convert(self, f):
        return io.imread(f).astype(np.uint8)

    def load_data_from_folder(self, dir):
        # read all images into an image collection
        ic = io.ImageCollection(dir+"*.bmp", load_func=self.imread_convert)
        
        #create one large array of image data
        data = io.concatenate_images(ic)
        
        #extract labels from image names
        labels = np.array(ic.files)
        for i, f in enumerate(labels):
            m = re.search("_", f)
            labels[i] = f[len(dir):m.start()]
        
        return(data,labels)

    def extract_image_features(self, data):
        # Please do not modify the header above
        # extract feature vector from image data
        print("Extracting features started.")
        startTime = time.time()

        feature_data = []
        for image in data:
            # The dimensions of image is: (240, 320, 3)
            
            #Convert image to two dimensions to extract features
            newImage = color.rgb2gray(image)

            # Use a feature extraction method
            #newImage = feature.canny(newImage, sigma=2)

            # Convert matrix to a one dimensional array
            newImage = newImage.flatten()
            feature_data.append(newImage)
        
        print("Extracting features finished. Duration:", time.time() - startTime, "s")

        # Please do not modify the return type below
        return(feature_data)

    def train_classifier(self, train_data, train_labels):
        # Please do not modify the header above
        # train model and save the trained model to self.classifier
        print("Training algorithm started")
        startTime = time.time()

        # self.classifier = svm.SVC(gamma=0.001)
        # self.classifier.fit(train_data, train_labels)

        self.classifier = LinearSVC()
        self.classifier.fit(train_data, train_labels)

        print("Training algorithm finished. Duration:", time.time() - startTime, "s")

    def predict_labels(self, data):
        # Please do not modify the header
        # predict labels of test data using trained model in self.classifier
        # the code below expects output to be stored in predicted_labels
        print("Predicting labels started")
        startTime = time.time()

        predicted_labels = self.classifier.predict(data)

        print("Predicting labels finished. Duration:", time.time() - startTime, "s")
        
        # Please do not modify the return type below
        return predicted_labels

      
def main():

    img_clf = ImageClassifier()

    # load images
    (train_raw, train_labels) = img_clf.load_data_from_folder('./train/')
    (test_raw, test_labels) = img_clf.load_data_from_folder('./test/')
    
    print("train_raw.shape:", train_raw.shape)
    #train_raw.shape: (196, 240, 320, 3)

    # convert images into features
    train_data = img_clf.extract_image_features(train_raw)
    test_data = img_clf.extract_image_features(test_raw)

    print("len(train_data):", len(train_data))
    print("len(train_data[0]):", len(train_data[0]))
    print("train_data[0].shape:", (train_data[0]).shape)
    print("len(test_data[0]):", len(test_data[0]))
    print("len(train_labels):", len(train_labels))
    
    # train model and test on training data
    img_clf.train_classifier(train_data, train_labels)
    predicted_labels = img_clf.predict_labels(train_data)
    print("\nTraining results")
    print("=============================")
    print("Confusion Matrix:\n",metrics.confusion_matrix(train_labels, predicted_labels))
    print("Accuracy: ", metrics.accuracy_score(train_labels, predicted_labels))
    print("F1 score: ", metrics.f1_score(train_labels, predicted_labels, average='micro'))
    
    # test model
    predicted_labels = img_clf.predict_labels(test_data)
    print("\nTesting results")
    print("=============================")
    print("Confusion Matrix:\n",metrics.confusion_matrix(test_labels, predicted_labels))
    print("Accuracy: ", metrics.accuracy_score(test_labels, predicted_labels))
    print("F1 score: ", metrics.f1_score(test_labels, predicted_labels, average='micro'))


if __name__ == "__main__":
    main()
