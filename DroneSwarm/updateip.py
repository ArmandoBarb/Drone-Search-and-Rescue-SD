# Python Program to Get IP Address
import socket
import os

hostname = socket.gethostname()
IPAddr = socket.gethostbyname(hostname)

print("**Update paths to settings.json (settingsPath) if needed")

print("\tYour Computer Name is:" + hostname)
print("\tYour Computer IP Address is:" + IPAddr)

cwd = os.getcwd()
cwd = os.path.join(cwd[:cwd.index("DroneSwarm")],'DroneSwarm')

constantsPath=os.path.join(str(cwd),'Constants', 'configDrones.py')

settingsPath="C:/Users/marii/OneDrive/Documents/AirSim/settings.json"

# replace ip in configDrones.py
with open(constantsPath) as file:
    lines1 = file.readlines()

i=0
for line in lines1:
    if "LOCAL_IP" in line:
        print("\tconfig IP updated")
        lines1[i]="LOCAL_IP = \""+str(IPAddr)+"\"\n"
    i+=1

with open(constantsPath, "w") as file:
    for line in lines1:
        file.write(line)

# replace ip in settings.json
with open(settingsPath) as file:
    lines2 = file.readlines()

i=0
for line in lines2:
    if "LocalHostIp" in line:
        print("\tsetting IP updated")
        lines2[i]="      \"LocalHostIp\": \""+str(IPAddr)+"\",\n"
    i+=1

with open(settingsPath, "w") as file:
    for line in lines2:
        file.write(line)
