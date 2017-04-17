#!/usr/bin/env python

from keras.models import Sequential
from keras.layers import Dense, Dropout, Activation, Flatten,Lambda
from keras.layers import Convolution2D, MaxPooling2D
from keras.layers.normalization import BatchNormalization
from keras.optimizers import Adam
from keras import backend as K
from keras.callbacks import LearningRateScheduler, Callback

## Assumes inputs are RGB 0 to 1 range

class SGDLearningRateTracker(Callback):
    def on_epoch_end(self, epoch, logs={}):
        optimizer = self.model.optimizer
        lr = K.eval(optimizer.lr * (1. / (1. + optimizer.decay * optimizer.iterations)))
        print('\nLearning Rate: {:.6f}\n'.format(lr))


class CarIDModel():
    def __init__(self, loadweights=False):
        self.input_shape = (64, 64, 3)
        self.model = self.getModel()
        if loadweights == True:
            self.loadWeights()

    def getModel(self):
        model = Sequential()
        model.add(Lambda(lambda x: x - 0.5, input_shape=self.input_shape))
        model.add(Convolution2D(1, (1, 1), padding="same"))
        model.add(BatchNormalization())
        model.add(Activation('relu'))
        model.add(Convolution2D(32, (3, 3), padding="same"))
        model.add(BatchNormalization())
        model.add(Activation('relu'))
        model.add(Convolution2D(32, (3, 3), padding="same"))
        model.add(BatchNormalization())
        model.add(Activation('relu'))
        model.add(MaxPooling2D(pool_size=(2, 2)))
        model.add(Dropout(0.25))

        model.add(Convolution2D(64, (3, 3), padding="same"))
        model.add(BatchNormalization())
        model.add(Activation('relu'))
        model.add(Convolution2D(64, (3, 3), padding="same"))
        model.add(BatchNormalization())
        model.add(Activation('relu'))
        model.add(MaxPooling2D(pool_size=(2, 2)))
        model.add(Dropout(0.25))

        model.add(Convolution2D(128, (3, 3), padding="same"))
        model.add(BatchNormalization())
        model.add(Activation('relu'))
        model.add(Convolution2D(128, (3, 3), padding="same"))
        model.add(BatchNormalization())
        model.add(Activation('relu'))
        model.add(MaxPooling2D(pool_size=(2, 2)))
        model.add(Dropout(0.25))

        model.add(Convolution2D(1, (1, 1), activation="relu"))
        model.add(Flatten())
        model.add(Dense(1, activation='sigmoid'))
        adam = Adam(lr=1e-2, beta_1=0.9, beta_2=0.999, epsilon=1e-08, decay=0.001)
        model.compile(adam, 'mse', metrics=['accuracy'])
        return model

    def loadWeights(self):
        self.model.load_weights("CarId_weights.h5")


    def saveWeights(self):
        self.model.save_weights("CarId_weights.h5", overwrite=True)

    def fit(self, X, Y, epochs=20):
        self.model.fit(X, Y, batch_size=256, epochs=epochs, verbose=2, validation_split=0.2, shuffle=True,
                  callbacks=[SGDLearningRateTracker()])

    def predictSingle(self, image):
        pred = self.model.predict(image[None, :, :, :])
        return pred[0][0]

if __name__ == '__main__':
    import glob
    import numpy as np
    import matplotlib.image as mpimg
    from sklearn.utils import shuffle

    cars = glob.glob("./data/image-data/vehicles/*/*.png")
    non_cars = glob.glob("./data/image-data/non-vehicles/*/*.png")

    Y = np.concatenate([np.ones(len(cars)), np.zeros(len(non_cars))])
    X = []
    for name in cars:
        X.append(name)
    for name in non_cars:
        X.append(name)
    X = np.array(X)

    print(X.shape)

    import random
    from sklearn.model_selection import train_test_split

    X_train, X_test, Y_train, Y_test = train_test_split(X, Y, test_size=0.20)


    def train_generator(batch_size=256, purpose='train'):
        data = None
        label = None
        if purpose == 'train':
            data = X_train
            label = Y_train
        else:
            data = X_test
            label = Y_test

        while 1:
            x_sample = []
            y_sample = []
            for i in range(batch_size):
                index = random.randint(0, data.shape[0] - 1)
                x_sample.append(mpimg.imread(data[index]))
                y_sample.append(label[index])
            yield np.stack(x_sample), np.stack(y_sample)

    carIdModel = CarIDModel(loadweights=True)
    carIdModel.model.summary()

    generator_train = train_generator(purpose='train')
    generator_validation = train_generator(purpose='train')
    generator_test = train_generator(purpose='test')

    # carIdModel.model.fit_generator(generator_train, steps_per_epoch=10, epochs=10, verbose=2,
    #                          validation_data=generator_validation, validation_steps=10)

    x, y = next(generator_test)
    score = carIdModel.model.evaluate(x, y, verbose=0)
    print('Test score:', score[0])
    print('Test accuracy:', score[1])

