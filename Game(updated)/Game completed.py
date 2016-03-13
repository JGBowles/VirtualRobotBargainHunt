import sys
from tkinter import *
import numpy
from heapq import *
import time
import datetime
import platform
import sqlite3


def heuristic(a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    return abs(dx) + abs(dy)

def astar(array, start, goal):

    neighbors = [(0,1),(0,-1),(1,0),(-1,0)]

    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []

    heappush(oheap, (fscore[start], start))
    
    while oheap:

        current = heappop(oheap)[1]

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j            
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:                
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue
                
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
                
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heappush(oheap, (fscore[neighbor], neighbor))
                
    return False


##nmap1 = numpy.fromfile(file=open("MAP 1.txt"); dtype=int).reshape((100,100,100))
nmap1 = numpy.array([
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1],
    [1,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1],
    [1,0,1,0,1,1,1,1,1,0,1,0,1,1,1,1,1,0,1],
    [1,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,1],
    [1,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,1],
    [1,0,1,1,1,1,1,0,1,0,1,1,1,1,1,0,1,0,1],
    [1,0,0,0,0,1,0,0,1,0,0,0,0,0,1,0,1,0,1],
    [1,0,0,0,0,1,0,0,1,0,0,0,0,0,0,0,1,0,1],
    [1,0,1,1,0,1,1,1,1,0,1,1,1,0,0,0,1,0,0],
    [1,0,0,1,0,0,0,1,0,0,0,0,1,0,1,0,1,0,1],
    [1,0,0,1,0,0,0,1,0,0,0,0,1,1,1,0,0,0,1],
    [1,1,0,1,0,1,0,1,0,1,1,0,1,0,0,0,0,0,1],
    [1,0,0,1,0,1,0,0,0,0,1,0,0,0,0,0,1,1,1],
    [1,0,0,1,0,1,1,0,0,0,1,0,0,0,1,1,1,0,1],
    [1,0,0,1,0,0,0,0,1,0,0,0,1,0,0,0,0,0,1],
    [1,0,0,1,0,0,0,0,1,0,0,0,1,0,0,0,0,0,1],
    [1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1]])
nmap2 = numpy.array([
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1],
    [1,1,1,1,1,0,1,0,1,0,1,0,1,1,1,1,1,0,1],
    [1,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,1,0,1],
    [1,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,1,1,1],
    [1,0,1,0,1,0,1,0,1,1,1,1,0,1,1,0,0,0,1],
    [1,0,1,0,0,0,1,0,0,0,0,1,0,0,1,0,0,0,1],
    [1,0,1,0,0,0,1,0,0,0,0,1,0,0,1,0,1,0,1],
    [0,0,1,1,1,0,1,1,1,1,0,1,1,0,1,0,1,0,0],
    [1,1,1,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,1],
    [1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,1],
    [1,0,0,0,1,0,0,0,1,1,0,0,0,0,1,0,0,0,1],
    [1,0,1,1,1,0,0,0,0,0,0,1,0,0,1,0,0,0,1],
    [1,0,1,0,0,0,1,0,0,0,0,1,1,0,1,1,1,0,1],
    [1,0,0,0,0,0,0,0,1,0,1,1,0,0,0,0,0,0,1],
    [1,0,0,0,1,0,0,0,1,0,1,0,0,0,0,0,0,0,1],
    [1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1]])
nmap3 = numpy.array([
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1],
    [1,0,1,1,1,1,1,1,1,0,1,0,1,1,1,1,1,0,1],
    [1,0,1,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,1],
    [1,0,0,0,1,1,1,1,1,0,1,0,1,1,1,1,1,0,1],
    [1,0,1,1,1,0,0,0,0,0,1,0,0,0,0,0,0,0,1],
    [1,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1],
    [0,0,0,0,0,0,1,1,1,1,1,0,1,1,1,0,1,1,1],
    [1,0,0,0,1,0,0,0,0,0,1,0,0,1,0,0,0,0,1],
    [1,1,1,0,1,0,0,0,0,0,1,0,0,1,1,0,0,0,1],
    [1,0,0,0,1,1,0,1,1,1,1,1,0,0,1,1,1,1,1],
    [1,0,0,0,1,0,0,0,0,1,0,1,0,0,0,0,0,0,1],
    [1,0,1,1,1,0,0,0,0,1,0,1,1,0,0,0,0,0,1],
    [1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,1],
    [1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,1],
    [1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1]])
nmap4 = numpy.array([
    [1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1],
    [1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1],
    [1,0,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,0,1],
    [1,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,1],
    [1,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,1],
    [1,0,0,0,1,1,1,1,1,0,1,0,1,1,1,0,1,0,1],
    [1,0,0,0,1,0,0,0,0,0,1,0,1,0,0,0,1,1,1],
    [1,0,1,0,1,0,0,0,0,0,1,0,1,0,0,0,1,0,0],
    [1,1,1,0,1,0,1,1,1,0,1,0,1,0,0,0,1,0,1],
    [1,0,0,0,1,0,1,1,1,0,1,0,1,0,0,0,1,0,1],
    [1,0,0,0,1,0,1,1,1,1,1,0,1,1,1,1,1,0,1],
    [1,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]])
nmap5 = numpy.array([
    [1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,0,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,0,1],
    [1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1],
    [1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1],
    [1,0,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1],
    [1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1],
    [1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1],
    [0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0],
    [1,0,1,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1],
    [1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,0,1],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1],
    [1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1],
    [1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]])
nmap6 = numpy.array([
    [1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1],
    [1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1],
    [1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,0,1],
    [1,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,1],
    [1,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,1],
    [1,0,1,0,1,1,1,1,1,0,1,0,1,1,1,1,1,0,1],
    [1,0,1,0,1,0,0,0,0,0,1,0,1,0,0,0,0,0,1],
    [1,0,1,0,1,0,0,0,0,0,1,0,1,0,0,0,0,0,1],
    [0,0,1,0,1,0,0,0,1,0,1,0,1,0,0,0,1,0,1],
    [1,0,0,0,1,0,0,0,1,0,1,0,1,0,0,0,1,0,1],
    [1,0,0,0,1,1,1,1,1,0,1,0,1,1,1,1,1,0,1],
    [1,0,1,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,1],
    [1,0,1,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,1],
    [1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,0,1,1,1],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]])

class userInterface():

    
    
    
    def __init__ (self, window):
        # Initializes the first window
        
        self.UserName = ""
        self.window = window
        #self.menuCanvas = Canvas(self.window, width=300, height=300, bg='white')
        #self.menuCanvas.pack(fill=BOTH, expand=1)

        self.background = PhotoImage(file="main.gif")
        self.background.image = Label(self.window, image= self.background)
        self.background.image.pack(fill=BOTH, expand=1)
        
        self.buttonFrame = Frame(self.background.image)
        self.buttonFrame.pack(side="top", pady=25)

        self.start = PhotoImage(file="start.gif")
        self.startButton = self.start
        self.startButton.image = Button(self.background.image, image=self.start, command = lambda: self.preGame())
        self.startButton.image.pack()
        
        self.setting = PhotoImage(file="setting.gif")
        self.settingsButton = self.setting
        self.settingsButton.image = Button(self.background.image, image=self.setting)
        self.settingsButton.image.pack()

        self.exit = PhotoImage(file="exit.gif")
        self.exitButton = self.exit
        self.exitButton.image = Button(self.background.image, image= self.exit, command = lambda: self.window.destroy())
        self.exitButton.image.pack()

    def preGame(self):
        newWindow = Tk()
        newWindow.geometry("600x200+425+250")
        newWindow.resizable(width=FALSE, height=FALSE)
        
        instructLabel = Label(newWindow, text="When you are ready to play, input your name and press play.")
        instructLabel.place(x=135, y=15)

        e = Entry(newWindow)
        e.place(x=225, y=50)
        
        playButton = Button(newWindow, text="Play", command = lambda: self.loadGameArena(newWindow, e))
        playButton.place(x=355, y=46)


        conn = sqlite3.connect('gameDatabase.db')
        c = conn.cursor()

        name = StringVar(newWindow)
        pixels = StringVar(newWindow)
        time = StringVar(newWindow)
        date = StringVar(newWindow)
        
        c.execute("SELECT name from gameStats WHERE id=1")
        for row in c:
            name.set(row)

        c.execute("SELECT pixels from gameStats WHERE id=1")
        for row in c:
            pixels.set(row)

        c.execute("SELECT time from gameStats WHERE id=1")
        for row in c:
            time.set(row)

        c.execute("SELECT date from gameStats WHERE id=1")
        for row in c:
            date.set(row)
        
        conn.commit()
        conn.close()

        lastTime = Label(newWindow, text="LAST TIME YOU PLAYED:")
        lastTime.place(x=230, y = 100)
        nameLabel = Label(newWindow, text="Name:")
        nameLabel.place(x=100, y=125)
        theName = Label(newWindow, textvariable = name)
        theName.place(x=100, y=140)

        pixelsLabel = Label(newWindow, text="Pixels travelled:")
        pixelsLabel.place(x=175, y=125)
        thePixels = Label(newWindow, textvariable = pixels)
        thePixels.place(x=175, y=140)

        timeLabel = Label(newWindow, text="Time elapsed (secs):")
        timeLabel.place(x=300, y=125)
        theTime = Label(newWindow, textvariable = time)
        theTime.place(x=300, y = 140)

        dateLabel = Label(newWindow, text="Date last played:")
        dateLabel.place(x=450, y=125)
        theDate = Label(newWindow, textvariable = date)
        theDate.place(x=450, y = 140)
        
        
        newWindow.mainloop()

        
    def settings(self):
        # Might need to edit this a little. 
        
        self.SettingsWindow = Tk()
        self.label = Label(self.SettingsWindow, text="There should be settings here. ")
        self.label.pack()
        self.SettingsWindow.mainloop()

    def loadGameArena(self, newWindow, e):
        # Eventually this will have a new game/load game option prior to loading the actual game
        self.userName = e.get()
        self.background.image.pack_forget()
        newWindow.withdraw()
        theGame = theRobotGame(self.window)

class theRobotGame():

    pixelsTravelled = 0
    startTime = time.time()
    
    def __init__(self, window):
        self.window = window

        self.window.geometry("1000x580+200+60")
        self.robotLocation = 1
        self.robotCoords = (65, 65)
        self.zone1 = Canvas(self.window, width=380, height = 580, bg="white")
        self.zone1.pack(side=RIGHT, fill=BOTH, expand=1)
        self.labelFrame = Frame(self.zone1)
        self.labelFrame.pack()
        self.ListOfItems = Label(self.labelFrame, text="Items Collected:", font="bold")
        self.ListOfItems.pack()
        self.alist = []
        self.SortButton = Button(self.zone1, text="Sort", command = lambda: self.SortAlgorithm(self.alist))
        self.SortButton.pack()
        self.createMap(self.robotCoords)
        self.labelList = Label(self.zone1, text=self.alist)
        
    def SortAlgorithm(self, alist):
        if len(alist)>1:
            mid = len(alist)//2
            lefthalf = alist[:mid]
            righthalf = alist[mid:]

            self.SortAlgorithm(lefthalf)
            self.SortAlgorithm(righthalf)

            i=0
            j=0
            k=0
            while i < len(lefthalf) and j < len(righthalf):
                if lefthalf[i] < righthalf[j]:
                    alist[k]=lefthalf[i]
                    i=i+1
                else:
                    alist[k]=righthalf[j]
                    j=j+1
                k=k+1

            while i < len(lefthalf):
                alist[k]=lefthalf[i]
                i=i+1
                k=k+1

            while j < len(righthalf):
                alist[k]=righthalf[j]
                j=j+1
                k=k+1
        self.LabelList.pack()
        self.LabelList.config(text=self.alist)
        
        
    def createMap(self, crds):

        self.coords = crds
        # split each if into it's own seperate function: zone1(), zone2() etc...
        if self.robotLocation == 1:
            self.zone = Canvas(self.window, width=630, height=580, bg='cyan')
            self.window.title("Zone 1")
            self.zone.pack(side=LEFT,fill=BOTH)
            self.teleport1 = self.zone.create_line(290, 560, 320, 560, fill="green", width=2)
            self.teleport2 = self.zone.create_line(590, 290, 590, 320, fill ="green", width=2)
            self.photo = PhotoImage(file="FRONT.png")
            self.robot = self.zone.create_image(self.coords, image=self.photo)
            self.canvas1(20,20,50,50)
            self.initiateGameplay(nmap1,1,1,7,6)

        if self.robotLocation == 2:
            self.zone.pack_forget()
            self.zone = Canvas(self.window, width=630, height=580, bg='cyan')
            self.window.title("Zone 2")
            self.zone.pack(fill=BOTH)
            self.teleport2 = self.zone.create_line(20, 290, 20, 320, fill ="green", width=2)
            self.teleport3 = self.zone.create_line(290, 560, 320, 560, fill="green", width=2)
            self.teleport7 = self.zone.create_line(590, 290, 590, 320, fill="green", width=2)
            self.photo = PhotoImage(file="FRONT.png")
            self.robot = self.zone.create_image(self.coords, image=self.photo)
            self.canvas2(20,20,50,50)

        if self.robotLocation == 6:
            self.zone.pack_forget()
            self.zone = Canvas(self.window, width=630, height=580, bg='cyan')
            self.window.title("Zone 6")
            self.zone.pack(fill=BOTH)
            self.teleport1 = self.zone.create_line(290,20,320,20, fill="green", width=2)
            self.teleport6 = self.zone.create_line(20,290,20,320, fill ="green", width=2)
            self.photo = PhotoImage(file="FRONT.png")
            self.robot = self.zone.create_image(self.coords, image=self.photo)
            self.canvas6(20,20,50,50)

        if self.robotLocation == 3:
            self.zone.pack_forget()
            self.zone = Canvas(self.window, width=630, height=580, bg='cyan')
            self.window.title("Zone 3")
            self.zone.pack(fill=BOTH)
            self.teleport3 = self.zone.create_line(20,290,20,320, fill="green", width=2)
            self.teleport4 = self.zone.create_line(290,560,320,560, fill="green", width=2)
            self.photo = PhotoImage(file="FRONT.png")
            self.robot = self.zone.create_image(self.coords, image=self.photo)
            self.canvas3(20,20,50,50)

        if self.robotLocation == 4:
            self.zone.pack_forget()
            self.zone = Canvas(self.window, width=630, height=580, bg='cyan')
            self.window.title("Zone 4")
            self.zone.pack(fill=BOTH)
            self.teleport4 = self.zone.create_line(290, 20, 320, 20, fill="green", width=2)
            self.teleport5 = self.zone.create_line(590, 290, 590, 320, fill ="green", width=2)
            self.photo = PhotoImage(file="FRONT.png")
            self.robot = self.zone.create_image(self.coords, image=self.photo)
            self.canvas4(20,20,50,50)

        if self.robotLocation == 5:
            self.zone.pack_forget()
            self.zone = Canvas(self.window, width=630, height=580, bg='cyan')
            self.window.title("Zone 5")
            self.zone.pack(fill=BOTH)
            self.teleport5 = self.zone.create_line(20,290,20,320, fill ="green", width=2)
            self.teleport6 = self.zone.create_line(290,20,320,20, fill ="green", width=2)
            self.teleport7 = self.zone.create_line(590,290,590,320, fill="green", width=2)
            self.photo = PhotoImage(file="FRONT.png")
            self.robot = self.zone.create_image(self.coords, image=self.photo)
            self.canvas5(20,20,50,50)
    def canvas1(self, topx,topy,botx,boty):
            room = open("MAP 1.txt","r")
            content = room.readlines()
            room.close()
            for i in range(len(content)):
                for symbol in content[i]:
                    if symbol == "W":
                        self.wall = self.zone.create_rectangle(topx,topy,botx,boty, fill="black")
                        botx += 30
                        topx +=30
                    elif symbol=="\n":
                        topy = boty
                        boty += 30
                        botx=50
                        topx=20
                    elif symbol=="0":
                       botx += 30
                       topx = botx - 30
                    elif symbol=="P":
                        self.photo1 = PhotoImage(file="FUEL.png")
                        self.fuel = self.zone.create_image(topx+15, topy+15, image=self.photo1)
                        botx += 30
                        topx = botx - 30
    def canvas2(self, topx,topy,botx,boty):
            room = open("MAP 2.txt","r")
            content = room.readlines()
            room.close()
            for i in range(len(content)):
                for symbol in content[i]:
                    if symbol == "W":
                        self.wall = self.zone.create_rectangle(topx,topy,botx,boty, fill="black")
                        botx += 30
                        topx +=20
                    elif symbol=="\n":
                        topy = boty
                        boty += 30
                        botx=50
                        topx=20
                    elif symbol=="0":
                       botx += 30
                       topx = botx - 30
                    elif symbol=="P":
                        self.photo2 = PhotoImage(file="Antenna.png")
                        self.antenna = self.zone.create_image(topx+15, topy+15, image=self.photo2)
                        botx += 30
                        topx = botx - 30
    def canvas3(self, topx,topy,botx,boty):
            room = open("MAP 3.txt","r")
            content = room.readlines()
            room.close()
            for i in range(len(content)):
                for symbol in content[i]:
                    if symbol == "W":
                        self.wall = self.zone.create_rectangle(topx,topy,botx,boty, fill="black")
                        botx += 30
                        topx +=20
                    elif symbol=="\n":
                        topy = boty
                        boty += 30
                        botx=50
                        topx=20
                    elif symbol=="0":
                       botx += 30
                       topx = botx - 30
                    elif symbol=="P":
                        self.photo3 = PhotoImage(file="BATTERIES.png")
                        self.batteries = self.zone.create_image(topx+15, topy+15, image=self.photo3)
                        botx += 30
                        topx = botx - 30
    def canvas4(self, topx,topy,botx,boty):
            room = open("MAP 4.txt","r")
            content = room.readlines()
            room.close()
            for i in range(len(content)):
                for symbol in content[i]:
                    if symbol == "W":
                        self.wall = self.zone.create_rectangle(topx,topy,botx,boty, fill="black")
                        botx += 30
                        topx +=20
                    elif symbol=="\n":
                        topy = boty
                        boty += 30
                        botx=50
                        topx=20
                    elif symbol=="0":
                       botx += 30
                       topx = botx - 30
                    elif symbol=="P":
                        self.photo4 = PhotoImage(file="DOOR.png")
                        self.door = self.zone.create_image(topx+15, topy+15, image=self.photo4)
                        botx += 30
                        topx = botx - 30
    def canvas5(self, topx,topy,botx,boty):
            room = open("MAP 5.txt","r")
            content = room.readlines()
            room.close()
            for i in range(len(content)):
                for symbol in content[i]:
                    if symbol == "W":
                        self.wall = self.zone.create_rectangle(topx,topy,botx,boty, fill="black")
                        botx += 30
                        topx +=20
                    elif symbol=="\n":
                        topy = boty
                        boty += 30
                        botx=50
                        topx=20
                    elif symbol=="0":
                       botx += 30
                       topx = botx - 30
                    elif symbol=="P":
                        self.photo5 = PhotoImage(file="Propulsion.png")
                        self.propulsion = self.zone.create_image(topx+15, topy+15, image=self.photo5)
                        botx += 30
                        topx = botx - 30
    def canvas6(self, topx,topy,botx,boty):
            room = open("MAP 6.txt","r")
            content = room.readlines()
            room.close()
            for i in range(len(content)):
                for symbol in content[i]:
                    if symbol == "W":
                        self.wall = self.zone.create_rectangle(topx,topy,botx,boty, fill="black")
                        botx += 30
                        topx +=20
                    elif symbol=="\n":
                        topy = boty
                        boty += 30
                        botx=50
                        topx=20
                    elif symbol=="0":
                       botx += 30
                       topx = botx - 30
                    elif symbol=="P":
                        self.photo6 = PhotoImage(file="CPU.png")
                        self.cpu = self.zone.create_image(topx+15, topy+15, image=self.photo6)
                        botx += 30
                        topx = botx - 30

        
        
    def initiateGameplay(self, xmap, x1,x2,y1,y2):
        self.zone.focus_set()
        
        pa,pb = x1,x2
        l = len(astar(xmap, (x1,x2), (y1,y2))) - 1
        while l >= 0:
            self.zone.update()
            a,b = astar(xmap, (x1,x2), (y1,y2)).pop(l)
            print("step %d" %l)
            print(a - pa)
            print(b - pb)
            ##down
            if a - pa == 1 and b - pb == 0:
                self.zone.move(self.robot, 0, 30)
                theRobotGame.pixelsTravelled += 30
                time.sleep(0.1)
                if self.robotLocation == 6:
                    self.ItemTuple6 = self.zone.find_overlapping(500,500,530,530)
                    if self.robot in self.ItemTuple6:
                        self.zone.delete(self.cpu)
                        self.CPUlabel = Label(self.labelFrame, text="CPU")
                        self.CPUlabel.pack()
                        self.alist.append("CPU")
                        self.initiateGameplay(nmap6,16,16,9,0)
                    else:
                        pass
                if self.robotLocation == 1:
                    self.ItemTuple1 = self.zone.find_overlapping(200,230,230,260)
                    if self.robot in self.ItemTuple1:
                        self.zone.delete(self.fuel)
                        self.FuelLabel = Label(self.labelFrame, text="Fuel")
                        self.FuelLabel.pack()
                        self.alist.append("Fuel")
                        self.initiateGameplay(nmap1,8,7,9,18)
                    else:
                        pass

                    self.teleportTuple1 = self.zone.find_overlapping(290,550,320,560)
                    
                    if self.robot in self.teleportTuple1:
                        self.robotLocation = 4
                        self.robotCoords = (305,65)
                        self.createMap(self.robotCoords)
                    else:
                        pass
                if self.robotLocation == 3:
                    self.ItemTuple3 = self.zone.find_overlapping(500,260,530,290)
                    if self.robot in self.ItemTuple3:
                        self.zone.delete(self.batteries)
                        self.BattLabel = Label(self.labelFrame, text="Battery")
                        self.BattLabel.pack()
                        self.alist.append("Battery")
                        self.initiateGameplay(nmap3,8,16,17,9)
                    else:
                        pass

                    self.teleportTuple4 = self.zone.find_overlapping(290,550,320,560)
                    
                    if self.robot in self.teleportTuple4:
                        self.robotLocation = 6
                        self.robotCoords = (305,65)
                        self.createMap(self.robotCoords)
                        self.initiateGameplay(nmap6,1,9,16,16)
                    else:
                        pass

                if self.robotLocation == 2:
                    
                    self.ItemTuple2 = self.zone.find_overlapping(440,80,470,110)
                    if self.robot in self.ItemTuple2:
                        self.zone.delete(self.antenna)
                        self.AntennaLabel = Label(self.labelFrame, text="Antenna")
                        self.AntennaLabel.pack()
                        self.alist.append("Antenna")
                        self.initiateGameplay(nmap2,2,14,9,18)
                    else:
                        pass

                    self.teleportTuple7 = self.zone.find_overlapping(290,550,320,560)

                    if self.robot in self.teleportTuple7:
                        self.robotLocation = 5
                        self.robotCoords= (305,65)
                        self.createMap(self.robotCoords)
                    else:
                        pass
                if self.robotLocation == 4:
                    self.ItemTuple4 = self.zone.find_overlapping(380,500,410,530)
                    if self.robot in self.ItemTuple4:
                        self.zone.delete(self.door)
                        self.DoorLabel = Label(self.labelFrame, text="Door")
                        self.DoorLabel.pack()
                        self.alist.append("Door")
                        time.sleep(5)
                    else:
                        pass
                if self.robotLocation == 5:
                    self.ItemTuple5 = self.zone.find_overlapping(530,500,560,530)
                    if self.robot in self.ItemTuple5:
                        self.zone.delete(self.propulsion)
                        self.PropulsionLabel = Label(self.labelFrame, text="Propulsion")
                        self.PropulsionLabel.pack()
                        self.alist.append("Propulsion")
                        self.initiateGameplay(nmap5,16,17,9,0)
                    else:
                        pass
            ##up
            if a - pa == -1 and b - pb == 0:
                self.zone.move(self.robot, 0, -30)
                theRobotGame.pixelsTravelled += 30
                time.sleep(0.1)
                if self.robotLocation == 1:
                    self.ItemTuple1 = self.zone.find_overlapping(200,230,230,260)
                    if self.robot in self.ItemTuple1:
                        self.zone.delete(self.fuel)
                        self.FuelLabel = Label(self.labelFrame, text="Fuel")
                        self.FuelLabel.pack()
                        self.alist.append("Fuel")
                        self.initiateGameplay(nmap1,8,7,9,18)
                    else:
                        pass

                if self.robotLocation == 6:
                    self.ItemTuple6 = self.zone.find_overlapping(500,500,530,530)
                    if self.robot in self.ItemTuple6:
                        self.zone.delete(self.cpu)
                        self.CPUlabel = Label(self.labelFrame, text="CPU")
                        self.CPUlabel.pack()
                        self.alist.append("CPU")
                        self.initiateGameplay(nmap6,16,16,9,0)
                    else:
                        pass

                    self.teleportTuple1 = self.zone.find_overlapping(290,30,320,20)

                    if self.robot in self.teleportTuple1:
                        self.robotLocation = 3
                        self.zone.pack_forget()
                        self.robotCoords = (305,515)
                        self.createMap(self.robotCoords)
                    else:
                        pass

                if self.robotLocation == 4:

                    self.teleportTuple4 = self.zone.find_overlapping(290,30,320,20)

                    if self.robot in self.teleportTuple4:
                        self.robotLocation = 1
                        self.zone.pack_forget()
                        self.robotCoords = (305,515)
                        self.createMap(self.robotCoords)
                    else:
                        pass

                if self.robotLocation == 5:
                    self.ItemTuple5 = self.zone.find_overlapping(530,500,560,530)
                    if self.robot in self.ItemTuple5:
                        self.zone.delete(self.propulsion)
                        self.PropulsionLabel = Label(self.labelFrame, text="Propulsion")
                        self.PropulsionLabel.pack()
                        self.alist.append("Propulsion")
                        self.initiateGameplay(nmap5,16,17,9,0)

                    self.teleportTuple7 = self.zone.find_overlapping(290,30,320,20)

                    if self.robot in self.teleportTuple7:
                        self.robotLocation = 2
                        self.zone.pack_forget()
                        self.robotCoords = (305,515)
                        self.createMap(self.robotCoords)
                    else:
                        pass
                if self.robotLocation == 2:
                    
                    self.ItemTuple2 = self.zone.find_overlapping(440,80,470,110)
                    if self.robot in self.ItemTuple2:
                        self.zone.delete(self.antenna)
                        self.AntennaLabel = Label(self.labelFrame, text="Antenna")
                        self.AntennaLabel.pack()
                        self.alist.append("Antenna")
                        self.initiateGameplay(nmap2,2,14,9,18)
                    else:
                        pass
                if self.robotLocation == 3:
                    self.ItemTuple3 = self.zone.find_overlapping(500,260,530,290)
                    if self.robot in self.ItemTuple3:
                        self.zone.delete(self.batteries)
                        self.BattLabel = Label(self.labelFrame, text="Battery")
                        self.BattLabel.pack()
                        self.alist.append("Battery")
                        self.initiateGameplay(nmap3,8,16,17,9)
                    else:
                        pass
            ##right
            if a - pa == 0 and b - pb == 1:
                self.zone.move(self.robot, 30, 0)
                theRobotGame.pixelsTravelled += 30
                time.sleep(0.1)
                if self.robotLocation == 6:
                    self.ItemTuple6 = self.zone.find_overlapping(500,500,530,530)
                    if self.robot in self.ItemTuple6:
                        self.zone.delete(self.cpu)
                        self.CPUlabel = Label(self.labelFrame, text="CPU")
                        self.CPUlabel.pack()
                        self.alist.append("CPU")
                        self.initiateGameplay(nmap6,16,16,9,0)
                        
                    else:
                        pass

                if self.robotLocation == 1:
                    self.ItemTuple1 = self.zone.find_overlapping(200,230,230,260)
                    if self.robot in self.ItemTuple1:
                        self.zone.delete(self.fuel)
                        self.FuelLabel = Label(self.labelFrame, text="Fuel")
                        self.FuelLabel.pack()
                        self.alist.append("Fuel")
                        self.initiateGameplay(nmap1,8,7,9,18)
                    else:
                        pass
                    self.teleportTuple2 = self.zone.find_overlapping(580,260,590,320)
                    
                    if self.robot in self.teleportTuple2:
                        self.robotLocation = 2
                        self.zone.pack_forget()
                        self.robotCoords = (65,305)
                        self.createMap(self.robotCoords)
                        self.initiateGameplay(nmap2,9,1,2,14)
                    else:
                        pass

                if self.robotLocation == 2:
                    
                    self.ItemTuple2 = self.zone.find_overlapping(440,80,470,110)
                    if self.robot in self.ItemTuple2:
                        self.zone.delete(self.antenna)
                        self.AntennaLabel = Label(self.labelFrame, text="Antenna")
                        self.AntennaLabel.pack()
                        self.alist.append("Antenna")
                        self.initiateGameplay(nmap2,2,14,9,18)
                    else:
                        pass

                    self.teleportTuple3 = self.zone.find_overlapping(580,260,590,320)

                    if self.robot in self.teleportTuple3:
                        self.robotLocation = 3
                        self.robotCoords = (65,305)
                        self.createMap(self.robotCoords)
                        self.initiateGameplay(nmap3,9,1,8,16)
                    else:
                        pass

                if self.robotLocation == 5:
                    self.ItemTuple5 = self.zone.find_overlapping(530,500,560,530)
                    if self.robot in self.ItemTuple5:
                        self.zone.delete(self.propulsion)
                        self.PropulsionLabel = Label(self.labelFrame, text="Propulsion")
                        self.PropulsionLabel.pack()
                        self.alist.append("Propulsion")
                        self.initiateGameplay(nmap5,16,17,9,0)
                    else:
                        pass

                    self.teleportTuple5 = self.zone.find_overlapping(580,260,590,320)
                    
                    if self.robot in self.teleportTuple5:
                        self.robotLocation = 6
                        self.zone.pack_forget()
                        self.robotCoords = (65,305)
                        self.createMap(self.robotCoords)
                    else:
                        pass

                if self.robotLocation == 4:

                    self.teleportTuple6 = self.zone.find_overlapping(580, 260, 590, 320)

                    if self.robot in self.teleportTuple6:
                        self.robotLocation = 5
                        self.zone.pack_forget()
                        self.robotCoords = (65,305)
                        self.createMap(self.robotCoords)
                    else:
                        pass
                if self.robotLocation == 3:
                    self.ItemTuple3 = self.zone.find_overlapping(500,260,530,290)
                    if self.robot in self.ItemTuple3:
                        self.zone.delete(self.batteries)
                        self.BattLabel = Label(self.labelFrame, text="Battery")
                        self.BattLabel.pack()
                        self.alist.append("Battery")
                        self.initiateGameplay(nmap3,8,16,17,9)
                    else:
                        pass
            ##left
            if a - pa == 0 and b - pb == -1:
                self.zone.move(self.robot, -30, 0)
                theRobotGame.pixelsTravelled += 30
                time.sleep(0.1)
                if self.robotLocation == 1:
                    self.ItemTuple1 = self.zone.find_overlapping(200,230,230,260)
                    if self.robot in self.ItemTuple1:
                        self.zone.delete(self.fuel)
                        self.FuelLabel = Label(self.labelFrame, text="Fuel")
                        self.FuelLabel.pack()
                        self.alist.append("Fuel")
                        self.initiateGameplay(nmap1,7,6,9,18)
                    else:
                        pass

                if self.robotLocation == 2:
                    
                    self.ItemTuple2 = self.zone.find_overlapping(440,80,470,110)
                    if self.robot in self.ItemTuple2:
                        self.zone.delete(self.antenna)
                        self.AntennaLabel = Label(self.labelFrame, text="Antenna")
                        self.AntennaLabel.pack()
                        self.alist.append("Antenna")
                        self.initiateGameplay(nmap2,2,14,9,18)
                    else:
                        pass

                    self.teleportTuple1 = self.zone.find_overlapping(20,290,30,320)

                    if self.robot in self.teleportTuple1:
                        self.robotLocation = 1
                        self.zone.pack_forget()
                        self.robotCoords = (545,305)
                        self.createMap(self.robotCoords)

                if self.robotLocation == 3:
                    self.ItemTuple3 = self.zone.find_overlapping(500,260,530,290)
                    if self.robot in self.ItemTuple3:
                        self.zone.delete(self.batteries)
                        self.BattLabel = Label(self.labelFrame, text="Battery")
                        self.BattLabel.pack()
                        self.alist.append("Battery")
                        self.initiateGameplay(nmap3,8,16,17,9)
                    else:
                        pass

                    self.teleportTuple3 = self.zone.find_overlapping(20,290,30,320)

                    if self.robot in self.teleportTuple3:
                        self.robotLocation = 2
                        self.zone.pack_forget()
                        self.robotCoords = (545,305)
                        self.createMap(self.robotCoords)
                    else:
                        pass

                if self.robotLocation == 6:
                    self.ItemTuple6 = self.zone.find_overlapping(500,500,530,530)
                    if self.robot in self.ItemTuple6:
                        self.zone.delete(self.cpu)
                        self.CPUlabel = Label(self.labelFrame, text="CPU")
                        self.CPUlabel.pack()
                        self.alist.append("CPU")
                        self.initiateGameplay(nmap6,16,16,9,0)
                    else:
                        pass

                    self.teleportTuple5 = self.zone.find_overlapping(20,290,30,320)

                    if self.robot in self.teleportTuple5:
                        self.robotLocation = 5
                        self.zone.pack_forget()
                        self.robotCoords = (545,305)
                        self.createMap(self.robotCoords)
                        self.initiateGameplay(nmap5,9,17,16,17)
                    else:
                        pass

                if self.robotLocation == 5:
                    self.ItemTuple5 = self.zone.find_overlapping(530,500,560,530)
                    if self.robot in self.ItemTuple5:
                        self.zone.delete(self.propulsion)
                        self.PropulsionLabel = Label(self.labelFrame, text="Propulsion")
                        self.PropulsionLabel.pack()
                        self.alist.append("Propulsion")
                        self.initiateGameplay(nmap5,16,17,9,0)

                    self.teleportTuple6 = self.zone.find_overlapping(20,290,30,320)

                    if self.robot in self.teleportTuple6:
                        self.robotLocation = 4
                        self.zone.pack_forget()
                        self.robotCoords = (545,305)
                        self.createMap(self.robotCoords)
                        self.initiateGameplay(nmap4,9,17,16,12)
                    else:
                        pass
            l = l - 1
            pa,pb = a,b
            
        
def main():
    window = Tk()
    window.geometry("300x300+300+300") # Window needs to be centered on each PC screen 
    game = userInterface(window)

    def closing():

# Username
# Pixels travelled
# Time elapsed
# Date?

##        print(game.userName)
##        print(theRobotGame.pixelsTravelled)
        time_elapsed1 = time.time() - theRobotGame.startTime
        timeFormatted = str(time_elapsed1)
        timeFormatted = timeFormatted[0:5]
        print(timeFormatted)

        now = datetime.datetime.now()
        day = str(now.day)
        month = str(now.month)
        year = str(now.year)
        theDate = day + "-" + month + "-" + year
        
        conn = sqlite3.connect('gameDatabase.db')
        c = conn.cursor()
        c.execute("DELETE from gameStats WHERE id=1")
        c.execute("INSERT into gameStats (id, name, pixels, time, date) VALUES (?, ?, ?, ?, ?)", (1, game.userName, theRobotGame.pixelsTravelled, timeFormatted, theDate))  

        
        conn.commit()
        conn.close()

        
        window.destroy()
        
    window.protocol("WM_DELETE_WINDOW", closing)
    window.mainloop()

if __name__=='__main__':
    sys.exit(main())
