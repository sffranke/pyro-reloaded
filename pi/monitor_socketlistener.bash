#!/bin/bash  

#   
# crontab: * * * * * /home/pi/monitor_socketlistener.bash  
#   

# Check if socketlistener.py is running
if ! pgrep -f socketlistener.py > /dev/null
then
    # If not running, start it
    python3 /home/pi/socketlistener.py &
fi
