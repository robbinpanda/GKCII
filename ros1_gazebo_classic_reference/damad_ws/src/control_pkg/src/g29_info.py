import re
import yaml
f = open("devices_info.txt","r",encoding='utf-8')
str = f.readlines()
event = 0
for i in range(len(str)):
    if "Logitech G29 Driving Force Racing Wheel" in str[i]:
        event = int("".join(re.findall(r'event(.*?)js',str[i+4])))
        print(event)
        break
def read_yaml_all():
    with open("src/g29_force_feedback/config/g29.yaml","r",encoding="utf-8") as f:
        data = yaml.load(f,Loader=yaml.FullLoader)
        return data
def update_g29yaml(k,v):
    old_data = read_yaml_all()
    old_data[k] = v
    with open("src/g29_force_feedback/config/g29.yaml","w",encoding="utf-8") as f:
        yaml.dump(old_data,f)
print("/dev/input/event%d"%(event))
update_g29yaml("device_name","/dev/input/event%d"%(event))

