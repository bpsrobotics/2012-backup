from enum import Enum
from datetime import datetime
import time
import math
import numpy


class CurrentState(Enum):
    ACC = 0
    CONST = 1
    DES = 2
    REST = 3


class PVAData:
    def __init__(self, currentPos, currentVel, targetPos, targetVel, targetAcc, status):
        self.currentPos = currentPos
        self.currentVel = currentVel
        self.targetPos = targetPos
        self.targetVel = targetVel
        self.targetAcc = targetAcc
        self.status = status


class Trap:
    __currentMilSec = lambda self: float(round(time.time() * 1000) / 1000)

    __maxAcc = 0
    __maxVel = 0

    __currentPos = 0
    __currentVel = 0

    __OFFSET = 0

    __currentState = CurrentState.REST
    __lastTime = 0
    __startTime = 0

    __lastPos = 0
    __lastVel = 0

    targetPos = 0

    __lastProf = PVAData(0.0, 0.0, 0.0, 0.0, 0.0, CurrentState.ACC)

    velGetter = lambda: 0.0
    posGetter = lambda: 0.0

    __isFinished = False

    __dec = False

    __sign = numpy.sign(targetPos)

    __PER_OFFSET = 0


    def __init__(self, maxVel, maxAcc, currentVelGetter, currentPosGetter):
        self.__maxAcc = maxAcc
        self.__maxVel = maxVel
        self.velGetter = currentVelGetter
        self.posGetter = currentPosGetter
        self.__startTime = self.__currentMilSec()
        self.__updateProfile()

    def __updateProfile(self):
        self.__lastVel = self.__lastProf.currentVel
        self.__lastPos = self.__lastProf.currentPos
        self.__currentVel = self.__lastProf.targetVel  # TODO get actual feedback
        self.__currentPos = self.__lastProf.targetPos  # TODO get actual feedback
        self.__lastTime = self.__currentMilSec()


    def updateTarget(self, targetPos):
        self.__isFinished = False
        self.__startTime = self.__currentMilSec()
        self.__lastTime = self.__currentMilSec()
        self.targetPos = targetPos
        self.__currentState = CurrentState.ACC
        self.__updateProfile()
        self.__sign = numpy.sign(targetPos)
        self.__PER_OFFSET = targetPos * 0.02

    def __hasToDec(self):
        time = math.fabs(self.__currentVel) / self.__maxAcc
        offSet = self.targetPos - self.__currentPos
        return math.fabs(offSet - self.__OFFSET) <= math.fabs(
            self.__currentVel * time + self.__sign * (-self.__maxAcc / 2) * math.pow(time, 2))

    def __updateCurrentStatus(self):
        print("offset pos", self.targetPos - self.__currentPos)
        print("1%", self.targetPos / 80)
        print("offset vel", self.__currentVel)
        print("1%", self.__maxVel / 80)
        if math.fabs(self.targetPos - self.__currentPos) < self.targetPos / self.__PER_OFFSET and math.fabs(self.__currentVel) < self.__maxVel / self.__PER_OFFSET:
            self.__isFinished = True
            self.__currentState = CurrentState.REST
        elif self.__hasToDec() or self.__dec:
            self.__dec = True
            self.__currentState = CurrentState.DES
        elif math.fabs(self.__maxVel - self.__OFFSET) <= math.fabs(self.__currentVel):
            self.__currentState = CurrentState.CONST
        elif not self.__dec:
            self.__currentState = CurrentState.ACC

    def update(self):
        if self.__isFinished:
            self.__lastProf = PVAData(
                currentPos=self.__currentPos,
                currentVel=self.__currentVel,
                targetPos=self.targetPos,
                targetVel=0.0,
                targetAcc=0.0,
                status=self.__currentState
            )
            return self.__lastProf
        __currentTime = self.__currentMilSec()
        deltaTime = (__currentTime - self.__lastTime)
        self.__lastTime = __currentTime
        self.__updateProfile()
        self.__updateCurrentStatus()
        if self.__currentState == CurrentState.ACC:
            self.__lastProf = PVAData(
                currentPos=self.__currentPos,
                currentVel=self.__currentVel,
                targetPos=self.__currentPos + deltaTime * self.__currentVel + self.__sign * (
                        self.__maxAcc / 2) * math.pow(deltaTime, 2.0),
                targetVel=self.__currentVel + self.__sign * self.__maxAcc * deltaTime,
                targetAcc=self.__sign * self.__maxAcc,
                status=self.__currentState
            )
        elif self.__currentState == CurrentState.CONST:
            self.__lastProf = PVAData(
                currentPos=self.__currentPos,
                currentVel=self.__currentVel,
                targetPos=self.__currentPos + (deltaTime * self.__currentVel),
                targetVel=self.__sign * self.__maxVel,
                targetAcc=0.0,
                status=self.__currentState
            )
        elif self.__currentState == CurrentState.DES:
            self.__lastProf = PVAData(
                currentPos=self.__currentPos,
                currentVel=self.__currentVel,
                targetPos=self.__currentPos + deltaTime * self.__currentVel + self.__sign * (-self.__maxAcc / 2) * (
                        deltaTime ** 2),
                targetVel=self.__currentVel + self.__sign * -self.__maxAcc * deltaTime,
                targetAcc=self.__sign * -self.__maxAcc,
                status=self.__currentState
            )
        else:
            self.__lastProf = PVAData(
                currentPos=self.__currentPos,
                currentVel=self.__currentVel,
                targetPos=self.targetPos,
                targetVel=0.0,
                targetAcc=0.0,
                status=self.__currentState
            )
        return self.__lastProf

    def isFinished(self):
        return self.__isFinished


if __name__ == '__main__':
    with open("text.csv", "w+") as file:
        cPos = 0
        cVel = 0
        pos = lambda: float(cPos)
        vel = lambda: float(cVel)
        test = Trap(400, 500, pos, vel)
        test.updateTarget(50)
        i = 0
        # while not test.isFinished():
        while True:
            it = test.update()
            cPos = it.targetPos
            cVel = it.targetVel
            file.write(str(i) + ", " + str(cPos) + ", " + str(cVel) + ", " + str(it.targetAcc) + "\n")
            print("cvel", cVel)
            print("cpos", cPos)
            print("status", it.status)
            print(test.isFinished())
            time.sleep(0.02)
            i = i + 1
            print("\n\n")
        print("done")
