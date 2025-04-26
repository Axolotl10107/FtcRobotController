def clamp(n, l, u):
    return max(l, min(n, u))

lists = []
maxAccel = float(input("Max. Acceleration: "))
timeStep = float(input("Time Step (in same time units as accel.): "))
maxDV = float(input("Max. delta V each step: "))

initialized = False
lastTime = 0
#oldDeltaV = 0
def request(newVel, currentVel, currentTime):
    global initialized, lastTime
    if initialized:
        loopTime = currentTime - lastTime
        #        print(f"loopTime: {loopTime}")
        requestedDeltaVThisLoop = newVel - currentVel
        #        print(f"requestedDeltaVThisLoop: {requestedDeltaVThisLoop}")
        targetDeltaVThisLoop = maxAccel * loopTime
        #        print(f"targetDeltaVThisLoop: {targetDeltaVThisLoop}")
        safeDeltaVThisLoop = min(targetDeltaVThisLoop, maxDV)
        #        print(f"safeDeltaVThisLoop: {safeDeltaVThisLoop}")
        actualDeltaVThisLoop = clamp(requestedDeltaVThisLoop, -safeDeltaVThisLoop, safeDeltaVThisLoop)
        #        print(f"actualDeltaVThisLoop: {actualDeltaVThisLoop}")
        #        oldDeltaV = actualDeltaVThisLoop
        lastTime = currentTime
        #        print(f"currentVel: {currentVel}")
        #        print(f"currentVel + actualDeltaVThisLoop: {currentVel + actualDeltaVThisLoop}")
        return currentVel + actualDeltaVThisLoop
    else:
        lastTime = currentTime
        initialized = True
        return 0

accelPerTimeStep = clamp(maxAccel * timeStep, -maxDV, maxDV)
print("Say 'done' after entering the last point.")
lastPoint = float(input("First Point: "))
lists.append([lastPoint])
user = input("Next Point: ")
while user != "done":
    user = float(user)
    change = abs(user - lastPoint)
    timeStepsNeeded = int(change / accelPerTimeStep)
    lists.append([user for x in range(timeStepsNeeded)])
    lastPoint = user
    user = input("Next Point: ")
#lists.append([user])

for idx in range(len(lists)-1, -1, -1):
    if idx == len(lists)-1:
        nextList = lists[idx]
    else:
        lists[idx].extend(nextList)
        nextList = lists[idx]

requestList = lists[0]
print(f"requestList: {requestList}")

timeList = [timeStep * x for x in range(len(requestList))]
print(f"timeList: {timeList}")

expectedList = []
lastOutput = 0
for idx, newVel in enumerate(requestList):
    lastOutput = float(request(newVel, lastOutput, timeList[idx]))
    expectedList.append(lastOutput)

print(f"expectedList: {expectedList}\n")

#n = input("Number of random entries: ")
lastRandom = -1
rand = -2
newRequestList = []
newTimeList = []
newExpectedList = []
lastVel = 0
initialized = False
lastTime = 0 #this (and the above line) should reset it for request() above so that time doesn't go backwards
from random import randint
while rand < (len(requestList) - 5):
    rand = randint(lastRandom+1, lastRandom+5)
    lastRandom = rand
    newRequestList.append(requestList[rand])
    newTimeList.append(timeList[rand])
    #    newExpectedList.append(expectedList[rand])
    lastVel = request(requestList[rand], lastVel, timeList[rand])
    newExpectedList.append(lastVel)
print(f"Randomized requestList: {newRequestList}")
print(f"Randomized timeList: {newTimeList}")
print(f"Randomized expectedList: {newExpectedList}")
