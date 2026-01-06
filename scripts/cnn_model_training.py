import os

def total_files(folder_path):
    num_files = len([f for f in os.listdir(folder_path) if os.path.isfile(os.path.join(folder_path, f))])
    return num_files

train_files_healthy = r"C:\Users\harsh\OneDrive\Desktop\FYP\Dataset\Train\Train\Healthy"
train_files_powdery = r"C:\Users\harsh\OneDrive\Desktop\FYP\Dataset\Train\Train\Powdery"
train_files_rust = r"C:\Users\harsh\OneDrive\Desktop\FYP\Dataset\Train\Train\Rust"

test_files_healthy = r"C:\Users\harsh\OneDrive\Desktop\FYP\Dataset\Test\Test\Healthy"
test_files_powdery = r"C:\Users\harsh\OneDrive\Desktop\FYP\Dataset\Test\Test\Powdery"
test_files_rust = r"C:\Users\harsh\OneDrive\Desktop\FYP\Dataset\Test\Test\Rust"

valid_files_healthy = r"C:\Users\harsh\OneDrive\Desktop\FYP\Dataset\Validation\Validation\Healthy"
valid_files_powdery = r"C:\Users\harsh\OneDrive\Desktop\FYP\Dataset\Validation\Validation\Powdery"
valid_files_rust = r"C:\Users\harsh\OneDrive\Desktop\FYP\Dataset\Validation\Validation\Rust"

print("Number of healthy leaf images in training set", total_files(train_files_healthy))
print("Number of powder leaf images in training set", total_files(train_files_powdery))
print("Number of rusty leaf images in training set", total_files(train_files_rust))

print("========================================================")

print("Number of healthy leaf images in test set", total_files(test_files_healthy))
print("Number of powder leaf images in test set", total_files(test_files_powdery))
print("Number of rusty leaf images in test set", total_files(test_files_rust))

print("========================================================")

print("Number of healthy leaf images in validation set", total_files(valid_files_healthy))
print("Number of powder leaf images in validation set", total_files(valid_files_powdery))
print("Number of rusty leaf images in validation set", total_files(valid_files_rust))

from PIL import Image
import IPython.display as display

image_path = r'C:\Users\harsh\OneDrive\Desktop\FYP\Dataset\Train\Train\Healthy\8ce77048e12f3dd4.jpg'

with open(image_path, 'rb') as f:
    display.display(display.Image(data=f.read(), width=500))

image_path = r'C:\Users\harsh\OneDrive\Desktop\FYP\Dataset\Train\Train\Rust\80f09587dfc7988e.jpg'

with open(image_path, 'rb') as f:
    display.display(display.Image(data=f.read(), width=500))

import keras
from tensorflow.keras.preprocessing.image import ImageDataGenerator

train_datagen = ImageDataGenerator(rescale=1./255, shear_range=0.2, zoom_range=0.2, horizontal_flip=True)
test_datagen = ImageDataGenerator(rescale=1./255)

train_generator = train_datagen.flow_from_directory(r'C:\Users\harsh\OneDrive\Desktop\FYP\Dataset\Train\Train',
                                                    target_size=(225, 225),
                                                    batch_size=32,
                                                    class_mode='categorical')

validation_generator = test_datagen.flow_from_directory(r'C:\Users\harsh\OneDrive\Desktop\FYP\Dataset\Validation\Validation',
                                                        target_size=(225, 225),
                                                        batch_size=32,
                                                        class_mode='categorical')

from keras.models import Sequential
from keras.layers import Conv2D, MaxPooling2D, Flatten, Dense


model = Sequential()
model.add(Conv2D(32, (3, 3), input_shape=(225, 225, 3), activation='relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(Conv2D(64, (3, 3), activation='relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(Flatten())
model.add(Dense(64, activation='relu'))
model.add(Dense(3, activation='softmax'))

model.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])
history = model.fit(train_generator,
                    batch_size=16,
                    epochs=100,
                    validation_data=validation_generator,
                    validation_batch_size=16
                    )

from matplotlib import pyplot as plt
from matplotlib.pyplot import figure

import seaborn as sns
sns.set_theme()
sns.set_context("poster")

figure(figsize=(7, 7), dpi=100)

plt.plot(history.history['accuracy'])
plt.plot(history.history['val_accuracy'])
plt.title('model accuracy')
plt.ylabel('accuracy')
plt.xlabel('epoch')
plt.legend(['train', 'val'], loc='upper left')
plt.show()
model.save("model.h5")

from tensorflow.keras.preprocessing.image import load_img, img_to_array
import numpy as np
import matplotlib.pyplot as plt  # Make sure to import matplotlib.pyplot
labels = list(train_generator.class_indices.keys())
def preprocess_image(image_path, target_size=(225, 225)):
    img = load_img(image_path, target_size=target_size)
    x = img_to_array(img)
    x = x.astype('float32') / 255.
    x = np.expand_dims(x, axis=0)
    return img, x  # Return both the image and the processed array

image_path = r"C:\Users\harsh\OneDrive\Desktop\FYP\Literature Review\pics\R.jpg"
img, x = preprocess_image(image_path)  # Get both the image and the processed data
predictions = model.predict(x)
predicted_label = labels[np.argmax(predictions)]
print(predicted_label)

plt.imshow(img)
plt.title(f"Disease Type: {predicted_label}")
plt.xticks([])  # Corrected to plt.xticks
plt.yticks([])  # Corrected to plt.yticks
plt.show()
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.metrics import confusion_matrix, classification_report

# Get the actual labels from the validation set
actual_labels = validation_generator.classes

# Get the predicted labels
predictions = model.predict(validation_generator)
predicted_labels = np.argmax(predictions, axis=1)

# Get class labels
class_names = list(validation_generator.class_indices.keys())

# Compute confusion matrix
cm = confusion_matrix(actual_labels, predicted_labels)

# Plot confusion matrix
plt.figure(figsize=(8, 6))
sns.heatmap(cm, annot=True, fmt="d", cmap="Blues", xticklabels=class_names, yticklabels=class_names)
plt.xlabel('Predicted')
plt.ylabel('Actual')
plt.title('Confusion Matrix')
plt.show()

# Print classification report
print("Classification Report:")
print(classification_report(actual_labels, predicted_labels, target_names=class_names))
