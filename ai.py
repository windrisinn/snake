from queue import PriorityQueue
from math import cos, sin


class SnakeAI:
    def __init__(self):
        self.narrowRangeStrategy = NarrowRangeStrategy()
        self.chooseDonutStrategy = ChooseDonutStrategy()
        self.pathFindingStrategy = PathFindingStrategy()

        self.narrowRangeStrategies = []
        self.chooseDonutStrategies = []
        self.pathFindingStrategies = []

        self.worldInfo = None
        self.gameInfo = None

    def onStarted(self, gameInfo, worldInfo) -> None:
        self.gameInfo = gameInfo
        self.worldInfo = worldInfo
        # print(gameInfo)
        # print(worldInfo)

        # 添加选豆范围缩小策略
        # ...
        # 添加选豆策略
        self.chooseDonutStrategies.append(self.chooseDonutStrategy.SmallDistance())
        # 添加寻路策略
        self.pathFindingStrategies.append(self.pathFindingStrategy.AStarMonitor())
        # self.pathFindingStrategies.append(self.pathFindingStrategy.AvoidTouchWall())

    def onEnded(self, score) -> None:
        pass

    def onRound(self, roundIndex, gameInfo, worldInfo):
        # strategyResults = []
        context = {"roundIndex": roundIndex, "worldInfo": worldInfo, "gameInfo": gameInfo}

        # 执行选豆范围缩小策略组
        self.procesBatch(self.narrowRangeStrategies, context)

        # 执行选豆策略组
        self.procesBatch(self.chooseDonutStrategies, context)

        # 执行寻路策略组
        self.procesBatch(self.pathFindingStrategies, context)
        return context.get("vector")

    def procesBatch(self, strategies, context):
        for strategy in strategies:
            if strategy.enable(context):
                strategy.process(context)


# class StrategyResult:
#     def __init__(self, status, data, priority) -> None:
#         self.status = status
#         self.data = data
#         self.priority = priority

def distance(a, b):
    return (a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y)


# 选豆策略组，返回豆坐标
class ChooseDonutStrategy:
    def __init__(self) -> None:
        pass

    # 豆子距离短优先策略
    class SmallDistance:
        # 策略开关
        def enable(self, context):
            return True

        # 策略处理
        def process(self, context):
            worldInfo = context.get("worldInfo")
            mySnake = worldInfo.mySnake

            minDistance = 10000
            finalPosition = 0, 0
            for donut in worldInfo.donuts:
                tempDistance = distance(donut.Position, mySnake.Nodes[0])
                # print(f"tempDistance: {tempDistance}, minDistance: {minDistance}")
                if tempDistance < minDistance:
                    minDistance = tempDistance
                    finalPosition = donut.Position
                    # print(f"finalPosition: {finalPosition}, mySnakePosition: {mySnake.Nodes[0]}")

            # 存放豆坐标结果
            print(finalPosition)
            context["finalPosition"] = finalPosition


class SmallRange:
    def __init__(self, width) -> None:
        self.width = width
        self.leftTop = (0, 0)
        self.rightBottom = (0, 0)

        self.score = 0
        self.donuts = []
        self.distance = 0
        self.snakeNum = 0
        self.snakeLength = 0
        self.otherSnakes = []
        self.rank = -100


# 选豆范围缩小策略组，返回范围缩小的地图
class NarrowRangeStrategy:
    def __init__(self, width, context) -> None:
        # 初始胡时传入小区块的宽度
        self.width = width
        self.worldInfo = context.get("worldInfo")
        self.smallRanges = []
        return

    # 划分区域
    def splitRange(self):
        for x in range(-50, 50, self.width):
            for y in range(25, -25, -self.width):
                smallRange = SmallRange(self.width)
                smallRange.leftTop = (x, y)
                smallRange.rightBottom = (x + self.width, y - self.width)
                smallRange.distance = self.calculateDistance(smallRange)
                self.smallRanges.append(smallRange)

    def calculateDistance(self, smallRange):
        myHead = self.worldInfo.mySnake.Nodes[0]
        x, y = myHead.X, myHead.Y
        boundX = smallRange.leftTop[0]
        boundY = smallRange.rightBottom[1]

        # 判断是否在区域内部
        if boundX <= x <= boundX + self.width and boundY <= y <= boundY + self.width:
            return 0.0
        # 外部区域
        centerX = (smallRange.leftTop[0] + smallRange.rightBottom[0]) / 2
        centerY = (smallRange.leftTop[1] + smallRange.rightBottom[1]) / 2
        return (centerX - x) * (centerX - x) + (centerY - y) * (centerY - y)

    # 分配豆子到指定区域
    def sendDonutsToRange(self):
        for donut in self.worldInfo.donuts:
            score = 10 if donut.Type == 1 else 1
            self.smallRanges[self.getRangeIndex(donut.Position.X, donut.Position.Y)].score += score
            self.smallRanges[self.getRangeIndex(donut.Position.X, donut.Position.Y)].donuts.append(donut)

    # 分配其他 snake 到指定区域
    def sendSnakesToRange(self):
        for snake in self.worldInfo.otherSnakes:
            if not snake.isAlive:
                continue
            self.smallRanges[self.getRangeIndex(snake.Nodes[0].X, snake.Nodes[0].Y)].snakeNum += 1
            self.smallRanges[self.getRangeIndex(snake.Nodes[0].X, snake.Nodes[0].Y)].snakeLength += len(snake.Nodes)
            self.smallRanges[self.getRangeIndex(snake.Nodes[0].X, snake.Nodes[0].Y)].otherSnakes.append(snake)

    # 根据坐标计算对应小区域的索引
    def getRangeIndex(self, x, y):
        offsetX = int(abs(x + 50) / self.width)
        offsetY = int(abs(y - 25) / self.width)

        return offsetX * (50 / self.width) + offsetY

    # 选择区域策略
    def sortSmallRange(self, w1, w2, w3):
        def getRank(smallRange):
            return smallRange.rank
        # 排序
        for i in range(self.smallRanges):
            self.smallRanges[i].rank = w1 * self.smallRanges[i].score - w2 * self.smallRanges[i].snakeLength - w3 * self.smallRanges[i].distance
        self.smallRanges.sort(key=getRank, reverse=True)

    def process(self, w1, w2, w3):
        self.splitRange()
        self.sendDonutsToRange()
        self.sendSnakesToRange()
        self.sortSmallRange(w1, w2, w3)
        return self.smallRanges[0]


# 寻路策略组，返回最终运动向量
class PathFindingStrategy:
    def __init__(self) -> None:
        pass

    # 围墙防撞
    class AvoidTouchWall:
        # 策略开关
        def enable(self, context):
            # 后续添加
            return True

        # 策略处理
        def process(self, context):

            gameInfo = context.get("gameInfo")
            finalPosition = context.get("finalPosition")
            worldInfo = context.get('worldInfo')
            mySnake = worldInfo.mySnake

            m = gameInfo.MapProperty.Size

            safeDis = gameInfo.SnakeProperty.Velocity / gameInfo.SnakeProperty.AngularVelocity * 180

            # 存放运动向量结果
            if finalPosition.X > safeDis and finalPosition.X < m.X - \
                    safeDis and finalPosition.Y > safeDis and finalPosition.Y < m.Y - safeDis:
                print("1234")
                context["vector"] = mySnake.Nodes[0].X - finalPosition.X, mySnake.Nodes[0].Y - finalPosition.Y
            else:
                context["vector"] = mySnake.Nodes[0].X - mySnake.Nodes[1].X, mySnake.Nodes[0].Y - mySnake.Nodes[1].Y
                # context["vector"] = (0, 0)

    # A*管理器
    class AStarMonitor:
        class AStarNode:
            def __init__(self, x, y, dirX, dirY, endX, endY) -> None:
                self.pos = (x, y)
                # 方向向量
                self.dir = (dirX, dirY)
                self.end = (endX, endY)
                self.f = 0
                self.g = 0
                self.h = 0

            def __eq__(self, __o: object) -> bool:
                # 确认是否需要判断方向向量
                return self.pos[0] == object.pos[0] & self.pos[1] == object.pos[1]

            def getF(self) -> float:
                return self.g + self.h

            def getG(self) -> float:
                # 父节点的 g 距离
                tg = self.father.getG if self.father is not None else 0
                return tg + self.g

            def getH(self) -> float:
                # 曼哈顿距离
                return abs(self.end[0] - self.pos[0]) + abs(self.end[1] - self.pos[1])

        def __init__(self) -> None:
            self

        # 返回两点之间的方向向量
        def getDir(start, end):
            return (end[0] - start[0], end[1] - end[0])

        def contains(self, queue, node) -> bool:
            for i in range(queue.count):
                if (queue[i] == node):
                    return True
            return False

        def checkWall(self, node, gameInfo, safeDist) -> bool:
            tWidth = gameInfo.MapProperty.Size[0]
            tHeight = gameInfo.MapProperty.Size[1]
            originX = gameInfo.MapProperty.Origin[0]
            originY = gameInfo.MapProperty.Origin[1]
            return originX + safeDist > node[0] or originX + tWidth - safeDist < node[0] or originY - safeDist < node[
                1] or originY - tHeight + safeDist > node[1]

        # 计算下一个可行走位置，并放入openList中
        def updateOpenList(self, openList: PriorityQueue, curNode: AStarNode, safeDist, gameInfo):
            lineNode = self.AStarNode(curNode.pos[0] + curNode.dir[0], curNode.pos[1] + curNode.dir[1], curNode.dir[0],
                                      curNode.dir[1], curNode.end[0], curNode.end[1])
            lineNode.father = curNode
            lineNode.g = curNode.getG() + 0.5

            dirX = curNode.pos[0] * cos(18) - curNode.pos[1] * sin(18)
            dirY = curNode.pos[0] * sin(18) + curNode.pos[1] * cos(18)
            length = 10 / 3.14 * sin(9)

            leftNode = self.AStarNode(curNode.pos[0] + dirX * length, curNode.pos[1] + dirY * length, dirX, dirY,
                                      curNode.end[0],
                                      curNode.end[1])
            leftNode.father = curNode
            leftNode.g = curNode.getG() + length

            dirX = curNode.pos[0] * cos(-18) - curNode.pos[1] * sin(-18)
            dirY = curNode.pos[0] * sin(-18) + curNode.pos[1] * cos(-18)

            rightNode = self.AStarNode(curNode.pos[0] + dirX * length, curNode.pos[1] + dirY * length, dirX, dirY,
                                       curNode.end[0], curNode.end[1])
            rightNode.father = curNode
            rightNode.g = curNode.getG() + length

            q = openList.queue

            if not self.contains(q, lineNode) and not self.checkWall(lineNode, gameInfo, safeDist):
                openList.put((lineNode.getF(), lineNode))
            if not self.contains(q, leftNode) and not self.checkWall(leftNode, gameInfo, safeDist):
                openList.put((leftNode.getF(), leftNode))
            if not self.contains(q, rightNode) and not self.checkWall(rightNode, gameInfo, safeDist):
                openList.put((rightNode.getF(), rightNode))

        # 获取A*决策接下来要走的方向向量
        def getDirection(self, endPos, gameInfo, worldInfo, safeDist):
            # 开启列表
            openList = PriorityQueue()
            # 关闭列表
            closeList = []

            head = (worldInfo.mySnake.Nodes[0].X, worldInfo.mySnake.Nodes[0].Y)
            curDir = (worldInfo.mySnake.Direction.X, worldInfo.mySnake.Direction.Y)
            node = self.AStarNode(head[0], head[1], curDir[0], curDir[1], endPos[0], endPos[1])
            openList.put((node.getF(), node))

            while not openList.empty():
                curNode = openList.get()
                closeList.append(curNode)
                # 找到临近终点的位置了
                if curNode.pos[0] - curNode.end[0] < 1 and curNode.pos[1] - curNode.end[1] < 1:
                    # 找到最终节点了，开始复现路径返回
                    path = []
                    while curNode.father is not None:
                        path.append(curNode)
                        curNode = curNode.father
                    return path.pop().dir

                self.updateOpenList(openList, curNode, safeDist, gameInfo)

            # 没找到路径
            return None

        def enable(self, context):
            return True

        def process(self, context):
            gameInfo = context.get("gameInfo")
            worldInfo = context.get('worldInfo')
            finalPosition = context.get("finalPosition")
            context["vector"] = self.getDirection(finalPosition, gameInfo, worldInfo, 3.14)


if __name__ == "__main__":
    ai = SnakeAI()
    ai.chooseDonutStrategies.append(ChooseDonutStrategy.SmallDistance())
    ai.onRound(1, 2, 3)
