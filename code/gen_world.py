import sys
n = 5
s = ""
for i in range(1, n + 1):
        s += (f'''DEF DRONE{i} Mavic2Pro {{
    translation {-6 * (i - 1)} 0 0.1
    name "drone{i}"
    controller "swarm_drone"
    controllerArgs ["{n}"]
    supervisor TRUE
    customData "line,8"
}}
''')


with open('output.txt', 'w') as f:
    f.write(s)