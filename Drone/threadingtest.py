import threading 
import time 
import sys, termios, tty
try:
    filedescriptors = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin)
    x = 0
    def maine(e): 
        while e.is_set(): 
            for p in range(5):
                print(p) 
                time.sleep(1)
            print("Try again") 
        print('Exiting e') 
    def mainp(p):
        while p.is_set():
            for pe in range(5):
                print(5-pe)
                time.sleep(1)
            print("Try again")
            # Rest of main program
        print('Exiting p')
        # Add some possible cleanup code here
    e = threading.Event()
    p = threading.Event()
    maine_thread = threading.Thread(name='main program',target=maine, args=(e,))
    mainp_thread = threading.Thread(name='main program',target=mainp, args=(p,))
    nothing_is_set=True
    while True:
        print("Started")
        x=sys.stdin.read(1)[0]
        print("You pressed", x)
        if x == "e":
            print("Changing to takeoff...")
            nothing_is_set=False
            e.set()
            if mainp_thread.is_alive():
                p.clear()
                mainp_thread.join()
            if not maine_thread.is_alive():
                maine_thread = threading.Thread(name='main program',target=maine, args=(e,)) 
                maine_thread.start() 
        if x == "p":
            print("If condition is met") 
            nothing_is_set=False
            p.set() 
            if maine_thread.is_alive():
                e.clear()
                maine_thread.join()
            if not mainp_thread.is_alive():
                mainp_thread = threading.Thread(name='main program',target=mainp, args=(p,))
                mainp_thread.start() 
        if x == "q":
            print("If condition is met") 
            e.clear()
            p.clear() 
            print('Terminating program')
            break
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, filedescriptors)
    if not nothing_is_set: 
        if maine_thread.is_alive():
            maine_thread.join() 
        if mainp_thread.is_alive():
            mainp_thread.join()
except KeyboardInterrupt:
    if not nothing_is_set:
        if maine_thread.is_alive():
            maine_thread.join()
        if mainp_thread.is_alive():
            mainp_thread.join()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, filedescriptors)
    
