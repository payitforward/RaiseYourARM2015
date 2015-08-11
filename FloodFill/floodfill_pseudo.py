next = None
init = True
goalReached = 0
def floodfill():
	goal=goals.get(goalReached)#get current goal
	q = []
	q.append(goal)
	for i in range(robot.getMazeWidth()):
		for j in range(robot.getMazeHeight()):
			index[i][j]=255#number every cell an invalid value
	index[goal.getX()][goal.getY()]=0
	while (len(q)!=0):
		c = q[0]
		q.remove(c)
		d = index[c.getX()][c.getY()] 
		if (not(mazeOfRobot.getWall(c, Direction.North).isSet())):
			if (index[c.getX()][c.getY()-1]>d+1):
				index[c.getX()][c.getY()-1]=d+1
				q.append(c.neighbor(Direction.North))
		if (not(mazeOfRobot.getWall(c, Direction.South).isSet())):
			if (index[c.getX()][c.getY()+1]>d+1):
				index[c.getX()][c.getY()+1]=d+1
				q.append(c.neighbor(Direction.South))		
		if (not(mazeOfRobot.getWall(c, Direction.West).isSet())):
			if (index[c.getX()-1][c.getY()]>d+1):
				index[c.getX()-1][c.getY()]=d+1
				q.append(c.neighbor(Direction.West))
		if (not(mazeOfRobot.getWall(c, Direction.East).isSet())):
			if (index[c.getX()+1][c.getY()]>d+1):
				index[c.getX()+1][c.getY()]=d+1
				q.append(c.neighbor(Direction.East))	
	return
def updateMaze():
	x = currentLocation.getX()
	y = currentLocation.getY()
	if realMaze.isWallNorth():
		mazeOfRobot.setWall(x,y,Direction.North)
	if realMaze.isWallSouth():
		mazeOfRobot.setWall(x,y,Direction.South)
	if realMaze.isWallWest():
		mazeOfRobot.setWall(x,y,Direction.West)
	if realMaze.isWallEast():
		mazeOfRobot.setWall(x,y,Direction.East)
	return
#get best direction based on cell's index
def getBestDirection():
	currentD = index[currentLocation.getX()][currentLocation.getY()]
	if ((not robot.isWallFront())):
		forwardCell = currentLocation.neighborCellAtDir(robot.getDirection())
		if (index[forwardCell.getX()][forwardCell.getY()]<currentD):
			return robot.getDirection()
	if ((not (robot.isWallEast())) and (index[location.getX()+1][location.getY()]<currentD)):
		return Direction.East
	elif ((not robot.isWallNorth()) and (index[location.getX()][location.getY()-1]<currentD)):
		return Direction.North
	elif ((not robot.isWallSouth()) and (index[location.getX()][location.getY()+1]<currentD)):
		return Direction.South
	elif ((not robot.isWallWest()) and (index[location.getX()-1][location.getY()]<currentD)):
		return Direction.West
	return None
def nextStep():
	global index,explore,goals,init,location,goalReached
	if (init):
		index = [[255 for i in range(robot.getMazeHeight())] for i in range(robot.getMazeWidth())]
		explore = [[False for i in range(robot.getMazeHeight())] for i in range(robot.getMazeWidth())]
		goals = robot.getPath()
		init = False
		
	if (currentLocation==goals.get(goalReached)):#goals: array of goals
		goalReached = goalReached + 1#goalReached: current goal's index in goals

	if (not explore[location.getX()][location.getY()]):#if we haven't passed this cell
		updateMaze()#get information about its wall
		explore[location.getX()][location.getY()]=True#set a flag to indicate that we have passed this cell
	
	nextDir = getBestDirection()#if we cannot find any direction that we can go
	if (nextDir==None):
		floodfill()#numbering again
		nextDir = getBestDirection()

	if (robot.getOppositeDirection(robot.getDirection())==nextDir):
		return TurnBack
	elif (robot.getLeftDirection(robot.getDirection())==nextDir):
		return TurnLeft
	elif (robot.getRightDirection(robot.getDirection())==nextDir):
		return TurnRight
	else:
		return Forward
		
			