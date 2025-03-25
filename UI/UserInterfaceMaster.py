import multiprocessing
import subprocess
import os
import sys

def runScript(scriptName):
    subprocess.run(['python', scriptName])

if __name__ == "__main__":
    # Get the absolute path to the directory containing this script
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Define paths to UI and Server scripts
    UI = os.path.join(current_dir, 'UserInterface.py')
    Server = os.path.join(current_dir, 'TestServer.py')

    # Create processes
    processUI = multiprocessing.Process(target=runScript, args=(UI,))
    processServer = multiprocessing.Process(target=runScript, args=(Server,))

    # Start server first
    print("Starting TestServer.py...")
    processServer.start()
    
    # Start UI
    print("Starting UserInterface.py...")
    processUI.start()
    
    # Wait for both processes to complete
    processUI.join()
    processServer.join()
    

    
