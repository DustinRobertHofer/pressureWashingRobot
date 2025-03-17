import multiprocessing
import subprocess

def runScript(scriptName):
    subprocess.run(['python', scriptName])

if __name__ == "__main__":
    UI = 'UI/UserInterface.py'
    Server = 'UI/TestServer.py'

    processUI = multiprocessing.Process(target=runScript, args=(UI,))
    processServer = multiprocessing.Process(target=runScript, args=(Server,))

    processUI.start()
    processServer.start()

    processUI.join()
    processServer.join()

    
